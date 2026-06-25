# Implementation roadmap

Tracks the path from the current HTTP-driven TX proof-of-concept to the headless
multi-zone Home Assistant bridge described in the README.

**Runtime model:** standalone Arduino / PlatformIO firmware (`firmware/`). The ESP
owns WiFi, MQTT, the control loop, the device registry (NVS) and HA discovery
itself — there is no ESPHome layer. (Earlier docs said ESPHome; that was dropped.)

Status legend: `[x]` done · `[ ]` open · `[~]` partial.

---

## M0 — RF proof of concept ✅ (done)

- [x] Protocol characterization + forked rtl_433 decoder (`docs/WATTS_WFHT_RF_PROTOCOL.md`)
- [x] Control model validated in Python (`tools/wfht_emulator.py`)
- [x] Frame build + custom CRC-8 + radio CRC-16 in firmware
- [x] Software Manchester encoder + raw OOK transmit (CC1101 hw Manchester/CRC/whitening off)
- [x] A-B-A burst with the +0.1 °C B-frame twin
- [x] HTTP control surface: `/status`, `/tx-test`, `/tx-watts`, `/tx-pair`
- [x] Non-blocking pairing stream in `loop()` (2 Hz, watchdog-safe)

## M1 — Single-zone live bridge ✅ (done)

Goal: spoof one captured device ID, driven by HA over MQTT, transmitting accurate
ambient + setpoint on the heartbeat, with failsafes — no HTTP, no RX, no
multi-zone yet.

**Note (confirmed on hardware):** the WFHC-MASTER receiver does its own
temperature regulation and **ignores the call-for-heat flag** — it actuates on
the transmitted ambient vs. setpoint alone. So the on-device P-loop is
unnecessary for this stack; the bridge just transmits the real ambient + setpoint
and runs the master in Comfort mode (HA is the sole scheduler). The P-loop /
duty→cfh items below are **now implemented and wired live** (byte 20 carries the
computed flag) — but on the WFHC-MASTER they have **no actuation effect**; they
exist only for a hypothetical dumb receiver that actuates on call-for-heat.

- [x] **MQTT client** — `AsyncMqttClient` wired up: connect + non-blocking 5 s
      reconnect done (authenticates against HA users; broker settings in `config.h`).
      LWT in place: `watts-bridge/status` carries `online`/`offline`, registered as
      a retained will so the broker announces an ungraceful death on our behalf.
- [x] **Characterize Z2M output for the W100** — field reference documented in
      `docs/MQTT_W100.md` from the Z2M device definition (model TH-S04D):
      `temperature` → T, `occupied_heating_setpoint` → SP, `system_mode` → heat/cool/off.
      Confirmed live: two W100s expose a `climate` cluster with `local_temperature`,
      `occupied_heating_setpoint`, `system_mode`.
- [x] **Subscribe** to thermostat state — implemented directly via the M3 Z2M
      auto-discovery path (the hard-coded W100 stepping stone was skipped). Subscribes
      to each discovered thermostat's `zigbee2mqtt/<name>`, caching `local_temperature`
      / `occupied_heating_setpoint` / `system_mode` with change-detection so Z2M's
      redundant full-state republishes (e.g. humidity-only) are ignored. Cached values
      are the inputs the control loop will read.
- [x] **Steady-state scheduler** — multi-zone from the start: per-bound-zone 154 s
      heartbeat in `loop()`, fires immediately on a source change (setpoint/mode),
      maps heat/cool→mode byte and Z2M `off`→setpoint 0.0. One A-B-A burst at a
      time, ≥2 s apart, so zones don't collide on the shared radio. Interim
      `/bind` / `/unbind` / `/bindings` tie a Z2M thermostat to a Watts device ID
      (volatile; M3 persists in NVS)
- [x] **Stale-data failsafe** — if a bound zone's MQTT source goes quiet past
      `ZONE_STALE_MS` (60 min), **stop transmitting it** rather than relay frozen
      values. Going silent makes the receiver see a lost thermostat and fall back
      on its own built-in handling (it beeps). No invented anti-freeze setpoint;
      it leans on the hardware's own loss detection. Resumes automatically when
      the source returns. Threshold must exceed the source's longest quiet
      interval (raise if a healthy zone gets dropped). See [[receiver-beeps-on-lost-thermostat]].
- [x] **Port the P-loop to firmware** — `duty = clip((SP - T)/Bp, 0, 1)` plus the
      anti-short-cycle clamps and demand-onset / SP-drop exceptions, ported from
      the emulator into `cfhDuty` / `cfhOnDuration` / `cfhUpdate` (`Bp=2.1`,
      `Cy=900s`, `On_min=Of_min=120s`). Runs per bound zone every `loop()` tick,
      free-running its own `Cy` phase from first data (no captured anchor); paused
      and re-anchored across a stale drop. Still unnecessary while the receiver
      self-regulates (see note above) — kept for a dumb call-for-heat receiver.
- [x] **Map duty → call-for-heat flag** (0x00 / 0x64) on the cycle boundary —
      `cfhUpdate` returns the flag, transmitted live as byte 20 by the scheduler.
      A flag flip marks the zone dirty so the burst fires promptly instead of
      waiting for the 154 s heartbeat. Surfaced to HA as a per-zone `Call for heat`
      sensor (`call_for_heat`: `calling`/`idle`) in the zone state blob.
- [x] **Bridge observability** — retained `watts-bridge/diag` JSON blob
      republished every 30 s (`rssi`, `uptime`, `reset_reason`, `last_tx_age`,
      `ip`, `fw`), seeded on connect. Self-announcing HA MQTT-discovery configs
      (`homeassistant/.../watts_bridge/*`): a `connectivity` binary_sensor on the
      LWT topic plus diagnostic sensors for the blob fields, all sharing the
      `watts-bridge/status` availability so HA greys them out together. `last_tx_age`
      is the radio-health tell — crossing ~160 s while still `online` means the
      radio is wedged though MQTT/WiFi are fine (the bridge-side mirror of the
      stale-data failsafe). Discovery is republished on every reconnect (idempotent,
      re-seeds if the broker drops its retained set).
- [x] **Per-zone state + HA entities** — each bound zone surfaces as its own HA
      device (nested under the bridge via `via_device`) from a retained
      `watts-bridge/zone/<slug>` blob: `status` (pending/active/stale), transmitted
      `setpoint` + `ambient`, `mode`, `last_tx_age`, and the spoofed `watts_id`.
      Discovery is published on `/bind` and re-advertised on every (re)connect;
      `/unbind` clears it (empty retained config → entity removed). State is pushed
      on every TX, on a source change/recovery, and on the stale-drop transition,
      with a 30 s heartbeat refresh to keep `last_tx_age` moving. Shares the bridge
      LWT for availability, so a dead bridge greys every zone out too.
- [x] **Binding config web UI** — a self-contained page served at `GET /`
      (no external/CDN assets) makes binding a no-typing browser task: it lists
      current bindings with live state + an Unbind button, offers a dropdown of
      discovered thermostats (new `GET /thermostats` endpoint), and a **Capture
      ID** button that arms `/pair-listen` and auto-fills the Watts ID when a
      thermostat is switched off. Drives the existing `/bind` / `/unbind` /
      `/pair-*` endpoints; kept separate from the TX test surface below.
- [x] **Gate the HTTP test endpoints** behind the `DEBUG_HTTP` build flag
      (default 0; gates `/status`, `/tx-test`, `/tx-watts`, `/rx-on`, `/rx-off` —
      not the binding UI / config endpoints, and not `/tx-pair`, which the
      pairing UI uses). The on-device binding/pairing web UI is kept as the
      config surface; HA-native config (M4) is deferred, not a replacement.
- [x] **Field test against the real Watts receiver (one zone)** — confirmed on
      hardware: the full HA → MQTT → ESP32 → CC1101 → receiver path drives the
      real Watts receiver and actuates the zone end-to-end

## M2 — RX path + pairing capture ✅ (done)

Goal: the radio can receive, so new device IDs can be registered by switching a
thermostat off.

- [x] **CC1101 receive config** + GDO interrupt (async OOK, edge-timed pulse capture)
- [x] **Software Manchester decode** of received frames (mirror the encoder) —
      decodes the chip stream, brute-forces pairing phase + byte alignment,
      restores the swallowed trailing low chip, tolerates a bit error in either
      sync word
- [x] **Validate** sync word + both CRCs on RX — CRC-gated, so only clean frames
      surface; verified live against multiple real thermostats incl. off-frames
- [x] **Off-frame detector** — a CRC-valid frame with setpoint 0.0 latches its
      3-byte device ID (the `FF FF FE` shape is implicit: the header is inside
      the CRC-8 coverage). Captures the A-frame; the 0.1 B-frame twin is ignored
- [x] **Pairing-capture flow** — `/pair-listen` arms (auto-enables RX),
      `/pair-status` polls the JSON result, `/pair-cancel` disarms; one-shot, and
      RX goes quiet after capture if pairing was what turned it on
- [x] (Optional) sniff/log real thermostat frames for cross-checking — `/rx-on`
      prints every CRC-valid frame (id, mode, amb, sp, cfh); `RX_DEBUG` flag dumps
      rejected bursts for protocol work

## M3 — Multi-zone + device registry (code complete — five-zone hardware test pending)

Goal: all five MVP zones, persisted, each bound to a Z2M thermostat.

- [x] **NVS device registry** — the `bindings[]` table (name + device ID) is
      persisted in NVS via `Preferences`, written on `/bind` / `/unbind` and
      restored at boot; a version key invalidates stale layouts. Runtime cadence
      state (lastTxAt/dirty) is intentionally not persisted
- [~] **Per-zone transmit instances** (5 zones: 4 up + 1 down) — multi-zone
      scheduler (M1) + persisted bindings done; each zone relays its bound
      thermostat's ambient + setpoint on the heartbeat (no per-zone P-loop).
      Should work; remaining: validate all five zones together on real hardware
- [x] **Z2M discovery** (pulled forward, used to satisfy M1's subscribe step) —
      subscribes to retained `zigbee2mqtt/bridge/devices`, reassembles the fragmented
      payload, filters for `climate`-type devices, and reads `exposes` for the state
      topic + field names (temp / setpoint / mode). Zero-config for arbitrary
      (non-W100) thermostats. Re-runs on every inventory republish. Discovered
      thermostats held in a fixed registry (`MAX_THERMOSTATS`).
- [x] **Binding workflow** — associate a captured Watts channel with a discovered
      Z2M thermostat, done via the binding web UI (`/bind` + the Capture-ID flow)
- [x] **Pair a new virtual device ID** to replace the broken thermostat —
      `/tx-pair` learns a bound zone's spoofed ID into the receiver (web UI button)

## M4 — Home Assistant UX (headless)

- [x] **Captive-portal provisioning** — WiFi + MQTT creds moved out of compile-time
      `config.h` into NVS (`NetConfig`), so the device is field-provisionable without
      a reflash. On boot with no SSID (or a 20 s STA-connect timeout) it comes up as
      an open AP `watts-bridge-XXXX` with a DNS catch-all, serving a setup form;
      `POST /save` persists to NVS and reboots into STA. `config.h` macros are now
      first-boot *seed* defaults only (loaded to RAM, not persisted) — editable by
      reflash only while NVS is empty; once the portal saves, NVS is authoritative.
      `GET /reset-wifi` (and a button on the binding UI) clears creds and reboots
      into the portal. Password fields have a show/hide toggle.
- [x] **Manual thermostats (sensor-only zones)** — for a zone with a temperature
      sensor but no Z2M thermostat. `GET /add-manual` (web UI) takes a name + the
      sensor's MQTT topic (+ optional JSON key); the bridge owns the zone's
      setpoint/mode command topics (`watts-bridge/zone/<slug>/set_setpoint` and
      `/set_mode`), persists the HA-set target across reboots, and auto-publishes a
      `homeassistant/climate/...` discovery so HA shows a full thermostat card with
      no hand-written YAML. Then bind to a Watts ID like any other zone.
- [ ] _(nice-to-have, deferred)_ **Per-zone parental lock** toggle. The W100/TH-S04D
      does **not** expose `child_lock` in Z2M, so the buttons can't be gated directly;
      the only lever is an upstream override — lock snapshots the current setpoint,
      transmits that held value to Watts (heating stays correct), and writes it back to
      the W100 to revert button-originated changes (display snaps back after a brief
      bounce). v1 semantics: lock = frozen setpoint (unlock in HA to adjust). Open
      decision: freeze `system_mode` too (else a kid can still switch the zone off).
      Pairs with the MQTT-discovery `switch` entity below. See [[web-ui-vs-ha-native-config]].
- [ ] **MQTT discovery entities** — expose status + lock per zone without cluttering
      the climate dashboard
- [ ] Document the HA/MQTT topic contract
- [ ] _(nice-to-have)_ **OTA firmware update** — flash over WiFi instead of USB.
      Natural fit: the ESP already owns WiFi + `ESPAsyncWebServer`, so an
      ElegantOTA-style `/update` page slots onto the existing web UI config surface
      (matches [[web-ui-vs-ha-native-config]]). Gates: must sit behind the existing
      HTTP digest auth (an unauthenticated flash endpoint is a LAN-wide foothold);
      needs a dual-app OTA partition table (`board_build.partitions` — check flash
      headroom first); an update blocks the radio for a few seconds, harmless against
      the 154 s heartbeat. First OTA-capable build still goes over USB; subsequent
      flashes go over the air.

## Testing & validation (cross-cutting)

How each layer is checked. The current and primary method is end-to-end against
real hardware, which is strong evidence — these items are about catching *future*
regressions and the failure modes that hardware testing doesn't exercise.

**Current method (works, keep using):**
- [x] Frame/CRC correctness — forked rtl_433 decodes TX frames byte-for-byte
      alongside the real thermostats; actuates the real Watts receiver
- [x] Control model — Python emulator validated against an overnight capture (`tools/wfht_emulator.py`)

**Per-milestone validation to add:**
- [x] **M1** — field test one zone against the real receiver: confirmed
      end-to-end on hardware (HA → MQTT → ESP32 → CC1101 → receiver actuates)
- [x] **M1** — failsafe test: stall a bound zone's source, confirm the stale-data
      timeout stops transmitting it (receiver flags a lost thermostat) — a path
      hardware "happy path" testing never hits. Confirmed on hardware via a
      temporarily lowered `ZONE_STALE_MS` (30 s): the zone logged the stale drop,
      TX ceased, HA reflected it, and it auto-recovered when the source returned.
      Note: boot-with-MQTT-down reaches the same safe end-state by a different path
      (no source data → never transmits → zone reports `pending`), so the
      stale-drop loop intentionally isn't exercised there
- [x] **M1** — MQTT resilience: kill/restart the broker and WiFi, confirm reconnect
      and that the loop keeps running on last-known values — confirmed on hardware:
      broker bounce triggered the non-blocking 5 s reconnect loop, then re-subscribed
      and re-ran Z2M discovery on reconnect; both zones kept transmitting from cache
      mid-outage (heartbeat is decoupled from MQTT, and a broker bounce is far short
      of the 60 min `ZONE_STALE_MS` drop threshold)
- [x] **M1** — LWT/availability: pull power (not a clean disconnect), confirm the
      broker publishes `offline` on `watts-bridge/status` and HA marks the device
      unavailable; on reboot confirm `online` + a fresh `diag` blob + `reset_reason`
      — confirmed on hardware: ungraceful power loss fired the retained will
      (`offline`, HA greyed the bridge + zones out), and reboot republished
      `online` with a fresh diag blob.
- [ ] **M2** — RX loopback: encode → decode roundtrip in firmware; assert recovered
      bytes + CRCs match the source frame
- [x] **M3** — registry persistence: write channels to NVS, reboot, confirm they
      reload — confirmed on hardware (bindings restored from NVS at boot each field test)
- [ ] **M3** — multi-zone isolation: confirm a per-channel device ID change doesn't
      corrupt other zones' frames (the refactor most likely to silently break CRCs)
      — part of the pending five-zone hardware test

**Optional (deferred — manual hardware loop deemed sufficient for now):**
- [ ] Host-side unit tests (`pio test -e native`) for the pure functions
      (`buildFrame`, `crc8_raw`, `crc16_cms`, `manchesterEncode`) asserting against
      known-good captured frames. Cheap regression tripwire; revisit if a silent
      frame/CRC regression ever bites, or before the M3 multi-zone refactor.

## Post-MVP (deferred, design only)

- [ ] Cooling + dewpoint safety + downstairs humidity contact (`docs/safety/cooling-and-dewpoint.md` — not yet written)
- [ ] Downstairs zone subdivision
- [ ] Upstream protocol findings to mainline rtl_433
