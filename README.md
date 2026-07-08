# Smart Retrofit RF Bridge: Watts WFHT-RF Emulator

Replacing Watts WFHT-RF legacy underfloor heating thermostats with a headless Home Assistant bridge running on an ESP32 + CC1101.

By intercepting the proprietary RF layer of existing HVAC installations, this bridge modernizes disconnected hydronic underfloor heating (UFH) systems without requiring an expensive mechanical overhaul of the manifold, valves, or pumps.

**Status:** The single-zone live bridge is confirmed end-to-end on real hardware —
the full HA → MQTT → ESP32 → CC1101 → receiver path actuates a real Watts zone.
MQTT-resilience, LWT/availability and the stale-data failsafe are all validated on
hardware. Also shipped: the RX path with off-frame pairing capture, NVS-persisted
multi-zone bindings, captive-portal WiFi/MQTT provisioning, a self-contained web UI
(binding **and** receiver pairing) behind optional HTTP digest auth, HA-driven
manual thermostats for sensor-only zones, and self-announcing per-zone Home
Assistant discovery entities. The debug/test HTTP endpoints are gated behind a
`DEBUG_HTTP` build flag (default off). Remaining: simultaneous five-zone
validation on hardware, plus deferred nice-to-haves (per-zone parental lock, OTA
firmware update) — see [`docs/ROADMAP.md`](/docs/ROADMAP.md).

Hardware — MCU, RF module, wiring and antenna selection — is documented in [`docs/HARDWARE.md`](/docs/HARDWARE.md).

The reverse engineering process of the wireless communication can be found here: [Watts WFHT RF Protocol Reverse Engineering](/docs/WATTS_WFHT_RF_PROTOCOL.md)

All reverse engineering findings for capturing packets have been incorporated into this [forked rtl_433 decoder](https://github.com/StevenBontius/rtl_433). Pending merge with mainline `rtl-433`.

## The Problem vs. The Solution

**The Problem:** Millions of homes run on durable mechanical underfloor heating manifolds installed in the 2000s and 2010s. However, their wireless wall thermostats are outdated and lack smart connectivity. Systems get obsolete and are hard to come by. Fully replacing the control block, actuators, and thermostats is a massive expense.

**The Solution:** A single, low-cost hardware bridge that completely spoofs the original wireless thermostats. The existing manifold receiver is still operating as originally designed extending its lifespan while bringing it into the smart era.

## System Architecture & Data Flow

The bridge functions as a silent translator, decoupling the user interface from the physical heating execution layer.

### How a setpoint reaches the floor

This setup uses an Aqara W100 thermostat, but could use any thermostat that can be coupled to Home Assistant or publish its temperature and setpoint to an MQTT broker.

```text
Aqara W100 (temp + humidity + up/down/scene buttons)
        │
        ▼  (Zigbee via Zigbee2MQTT)
   Home Assistant
        │     measured room temp:  relayed to an MQTT topic
        │     setpoint:            from HA schedule/automation or the local
        │                          W100 buttons (subject to parental lock)
        ▼
   MQTT Broker
        │
        ▼  (subscribed by the standalone ESP32 firmware)
   ESP32 bridge
        │     - caches each bound zone's ambient + setpoint + mode
        │     - builds the 192-bit fixed frame + custom application CRC-8
        │       + radio CRC-16
        │     - call-for-heat byte (20) carries a live P-loop duty flag
        │       (the WFHC-MASTER ignores it — see below)
        ▼
   CC1101 Radio Transceiver
        │     - 0xD391 sync word, A-B-A burst every 154 s (or instantly on change)
        ▼  (433.92 MHz, spoofed Watts WFHT-RF OOK/Manchester packet)
  Watts Central Receiver
        │     - runs its own temperature regulation from ambient vs. setpoint
        ▼
   Manifold servo for the zone opens/closes the water loop
```

### Why the ESP32's control loop doesn't drive the floor

Confirmed on hardware: the **WFHC-MASTER receiver does its own temperature
regulation and ignores the call-for-heat flag** — it actuates on the transmitted
ambient vs. setpoint alone. So the bridge is effectively a pure relay: it transmits
the real ambient + setpoint and the master runs in **Comfort mode** with Home
Assistant as the sole scheduler.

A chronoproportional P-controller (`duty = clip((SP − T)/Bp, 0, 1)`, with
anti-short-cycle clamps) is validated in the Python emulator
(`tools/wfht_emulator.py`) and **now ported to firmware** — it runs per bound zone
and its computed duty is transmitted live as the call-for-heat flag on byte 20 (and
surfaced to HA as a per-zone `Call for heat` sensor). But on the WFHC-MASTER that
flag has **no actuation effect**; the loop is kept only for a hypothetical dumb
receiver that actuates on call-for-heat instead of self-regulating.

### Edge Resiliency

The architecture features layered redundancy to keep the system safe and stable even when components or network connections fail:

* **Inherent Hardware Failsafe**: The physical Watts manifold receiver expects a regular heartbeat. If the ESP32 bridge loses power or suffers a critical hardware failure and stops transmitting its 154-second steady-state broadcasts, the receiver detects the loss (it beeps) and drops the affected zone into its safe, factory-default state.

* **Decoupled Heartbeat (Short-Term Resiliency)**: The 154 s transmit cadence runs independently of MQTT. If Home Assistant restarts or Wi-Fi/the broker drops briefly, the ESP32 keeps transmitting each zone's **last-known ambient + setpoint from cache**, backed by a non-blocking 5 s MQTT reconnect.

* **Stale-Data Failsafe (Extended Network Drop)**: To avoid relaying frozen values indefinitely, the ESP32 tracks the age of each zone's incoming MQTT data. If a source goes quiet past the safety window (`ZONE_STALE_MS`, 60 min), the bridge **stops transmitting that zone** rather than inventing a value. Going silent makes the receiver see a lost thermostat and fall back on its own loss handling, leaning on the hardware's built-in detection. The zone resumes automatically when its source returns.

### Provisioning & Web UI

The bridge is field-provisionable without a reflash. On first boot (or after a
20 s STA-connect timeout) it comes up as an open access point `watts-bridge-XXXX`
with a DNS catch-all, serving a setup form; WiFi + MQTT credentials are saved to
NVS and the device reboots into station mode. `config.h` macros are only first-boot
seed defaults — once the portal saves, NVS is authoritative.

In normal operation the device serves a self-contained config page (no external/CDN
assets) at `GET /` that makes both binding and pairing a no-typing browser task:

* lists current bindings with live state and an Unbind button;
* a dropdown of auto-discovered Z2M thermostats (zero-config — read from the Z2M
  `bridge/devices` inventory);
* a **Capture ID** button that arms RX and auto-fills the Watts ID when a thermostat
  is switched off, showing the captured ambient temperature so you can confirm it's
  the right room before binding;
* a per-zone **Pair** button that initiates the receiver-pairing sequence (see below)
  with a live countdown;
* an **Add manual thermostat** form for zones that have only a temperature sensor
  and no Z2M thermostat (see below).

The config page and its endpoints can be put behind optional HTTP digest auth
(set credentials in `config.h`); the debug/test endpoints (`/tx-test`, `/rx-on`,
…) are separately gated behind the `DEBUG_HTTP` build flag and off by default.

### Headless Home Assistant Integration

The bridge works behind the scenes over MQTT and does **not** create cluttered,
duplicate *climate* dashboard entities, the UI stays focused on the Aqara W100s.
It does self-announce lightweight MQTT-discovery entities: a bridge device
(connectivity + diagnostics: RSSI, uptime, reset reason, last-TX age, IP, firmware)
and one nested device per bound zone (status, transmitted setpoint/ambient, mode,
call-for-heat, last-TX age, spoofed Watts ID), all sharing the bridge's LWT so a
dead bridge greys everything out together.

For a zone that has only a temperature sensor and no Z2M thermostat, the bridge can
own a **manual thermostat**: add it from the web UI with the sensor's MQTT topic,
and the bridge takes over the setpoint/mode command topics, persists the HA-set
target across reboots, and auto-publishes a `climate` discovery so Home Assistant
shows a full thermostat card — no hand-written HA YAML. Then bind it to a Watts ID
like any other zone.

*Deferred (nice-to-have):* a per-zone parental lock that freezes a zone's setpoint
against local W100 button overrides, and OTA firmware update over WiFi.

### Spoofing and Pairing Capabilities

The bridge supports both, and both can be driven from the web UI:

1. **Spoofing existing device IDs:** For thermostats that still work, the ESP32 clones their 3-byte hardware identifier (e.g., `34:9E:48`) — captured off-air by switching the thermostat off. The Watts receiver cannot tell the difference.
2. **Pairing new virtual device IDs:** To replace broken thermostats or subdivide zones, the bridge initiates a pairing sequence. Transmitting at a rapid 2 Hz cadence (~500 ms) with bit 0 of byte 12 set to `1` — registering a brand-new virtual thermostat with the physical Watts receiver. Put the receiver zone into learn mode, then hit **Pair** on that zone in the web UI; the bridge streams pairing frames carrying the zone's live ambient + setpoint for ~30 s.

### Hardware

* **ESP32 DevKit C** with **CC1101** module for processing and transmitting.
* Two Watts WFHT central RF receivers, coupled by a cool/heat signal wire, each driving its own manifold with electrothermal servo valves.
* Downstairs central has a voltage-free humidity contact input (reserved as the cooling-phase emergency stop).
* Watts WFHT-LCD-RF wireless thermostats (currently in service; one is broken and is the immediate replacement target).
* Panasonic Aquarea heat pump with HeishaMon.
* Home Assistant OS on a NUC, with Zigbee2MQTT.
* Aqara W100 climate sensors (one per zone).

## Scope

**In the heating MVP:**

* All four upstairs zones plus the single downstairs zone running through the ESP32 bridge.
* The bridge as a pure ambient + setpoint relay, with the Watts receiver self-regulating in Comfort mode (no per-zone control loop on the ESP32).
* Replacement of the broken thermostat via a newly paired virtual device ID.
* Home Assistant owning setpoints (schedules, presence, holiday mode) with W100 buttons as the per-room local override.
* HA-driven manual thermostats for zones with a temperature sensor but no Z2M thermostat.

**Explicitly deferred to post-MVP phases:**

* Per-zone parental lock (the W100 exposes no `child_lock` in Z2M, so it needs an upstream setpoint-freeze workaround) and OTA firmware update over WiFi.
* Cooling, dewpoint safety, cooking detection, and integration of the downstairs humidity contact. Design exists in `docs/safety/cooling-and-dewpoint.md`. Cooling is a downstairs-only concern.
* Downstairs zone subdivision into multiple separately controlled zones.
* Upstreaming protocol findings to mainline `rtl_433`.
