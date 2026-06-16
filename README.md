# Smart Retrofit RF Bridge: Watts WFHT-RF Emulator

Replacing Watts WFHT-RF legacy underfloor heating thermostats with a headless Home Assistant bridge running on an ESP32 + CC1101.

By intercepting the proprietary RF layer of existing HVAC installations, this bridge modernizes disconnected hydronic underfloor heating (UFH) systems without requiring an expensive mechanical overhaul of the manifold, valves, or pumps.

**Status:** Phase 1 Capture and protocol characterization complete.

[Watts WFHT RF Protocol Reverse Engineering](/docs/WATTS_WFHT_RF_PROTOCOL.md)

All reverse engineering findings for capturing packets have been incorporated into this [forked rtl_433 decoder](https://github.com/StevenBontius/rtl_433). Pending merge with mainline `rtl-433`.

## The Problem vs. The Solution

**The Problem:** Millions of homes run on durable mechanical underfloor heating manifolds installed in the 2000s and 2010s. However, their wireless wall thermostats are outdated and lack smart connectivity. Systems get obsolete and are hard to come by. Fully replacing the control block, actuators, and thermostats is a massive expense.

**The Solution:** A single, low-cost hardware bridge that completely spoofs the original wireless thermostats. The existing manifold receiver is still operating as originally designed extending its lifespan while bringing it into the smart era.

## System Architecture & Data Flow

The bridge functions as a silent translator, decoupling the user interface from the physical heating execution layer. 

### How a setpoint reaches the floor

Uses an Aqara w100 thermostat, but could use any thermostat that can be coupled to Home Assistant or publish its temperature and setpoint to an MQTT broker.

```text
Aqara W100 (temp + humidity + up/down/scene buttons)
        │
        ▼  (Zigbee via Zigbee2MQTT)
   Home Assistant
        │     measured room temp: Relayed to MQTT topic
        │     setpoint:           From HA schedule/automation or local W100 
        │                         buttons (subject to parental lock)
        ▼
   MQTT Broker
        │
        ▼  (Subscribed by standalone ESP32 firmware)
   ESP32 (Edge Intelligence Layer)
        │     - Runs local PI control loop (Bp=2.0°C, Cy=900s, On/Of=120s)
        │     - Calculates active duty fraction and target call-for-heat flag (0x64 or 0x00)
        │     - Calculates custom application CRC-8
        ▼
   CC1101 Radio Transceiver
        │     - Formats 192-bit fixed packet with 0xD391 sync word
        │     - Transmits A-B-A burst every 154 seconds (or instantly on state change)
        ▼  (433.92 MHz, spoofed Watts WFHT-RF OOK Manchester packet)
  Watts Central Receiver
        │
        ▼
   Manifold servo for the zone opens/closes the water loop

```

### Edge Resiliency

The architecture features layered redundancy to ensure the system remains safe and stable even if components or network connections fail:

* **Inherent Hardware Failsafe**: The physical Watts manifold receiver expects a regular heartbeat. If the ESP32 bridge loses power or suffers a critical hardware failure and stops transmitting its 154-second steady-state broadcasts, the Watts receiver detects the signal loss and automatically drops the affected zones into a safe, factory-default state.

* **Edge Intelligence (Short-Term Resiliency)**: Because the chronoproportional PI control loop runs directly on the ESP32 microcontroller, the system gracefully handles brief upstream disconnections. If Home Assistant restarts or the Wi-Fi drops temporarily, the ESP32 continues executing the active PI loop using the last known setpoint and ambient temperature.

* **Software Timeout Failsafe (Extended Network Drop)**: To prevent runaway heating or freezing based on stale data, the ESP32 monitors the age of incoming MQTT payloads. If no valid sensor update or heartbeat is received from Home Assistant within a defined safety window (e.g., 60 minutes), the ESP32 aborts the active control loop. It will explicitly transmit an idle flag (0x00) to the Watts receiver, forcing the system into a failsafe standby or frost-protection mode until network communication is reliably reestablished.

### Headless Integration & Local Input

The bridge works entirely behind the scenes via background MQTT topics. It does not generate cluttered, duplicate climate dashboard entities in Home Assistant, allowing the user interface to remain focused on modern smart sensors like the Aqara W100.

Each zone features a lock toggle in Home Assistant. When a zone is locked, local Zigbee button presses for that zone are ignored, preventing accidental or unauthorized schedule overrides.

### Spoofing and Pairing Capabilities

The bridge requires both capabilities for MVP:

1. **Spoofing existing device IDs:** For thermostats that still work, the ESP32 clones their 3-byte hardware identifier (e.g., `34:94:2B`). The Watts receiver cannot tell the difference.
2. **Pairing new virtual device IDs:** To replace broken thermostats or subdivide zones, the ESP32 can initiate a pairing sequence. It achieves this by transmitting at a rapid 2 Hz cadence (~500 ms) with bit 0 of byte 12 set to `1`, registering a brand-new virtual thermostat with the physical Watts receiver.

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
* Local execution of the chronoproportional PI loop on the ESP32 for edge resiliency.
* Replacement of the broken thermostat via a newly paired virtual device ID.
* Home Assistant owning setpoints (schedules, presence, holiday mode) with W100 buttons as the per-room local override.
* Per-zone parental lock toggling local button input on or off.

**Explicitly deferred to post-MVP phases:**

* Cooling, dewpoint safety, cooking detection, and integration of the downstairs humidity contact. Design exists in `docs/safety/cooling-and-dewpoint.md`. Cooling is a downstairs-only concern.
* Downstairs zone subdivision into multiple separately controlled zones.
* Upstreaming protocol findings to mainline `rtl_433`.