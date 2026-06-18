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

## M1 — Single-zone live bridge (in progress)

Goal: spoof one captured device ID, driven by HA over MQTT, with on-device control
and failsafes — no HTTP, no RX, no multi-zone yet.

- [~] **MQTT client** — `AsyncMqttClient` wired up: connect + non-blocking 5 s
      reconnect done (authenticates against HA users; broker settings in `config.h`).
      LWT still to add.
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
- [ ] **Port the P-loop to firmware** — `duty = clip((SP - T)/Bp, 0, 1)` plus the
      anti-short-cycle clamps and demand-onset / SP-drop exceptions from the emulator
- [ ] **Steady-state scheduler** — retransmit every 154 s in `loop()`, and fire
      immediately on a state change (setpoint or call-for-heat flip)
- [ ] **Map duty → call-for-heat flag** (0x00 / 0x64) on the cycle boundary
- [ ] **Stale-data failsafe** — if no valid MQTT update within the safety window
      (≈60 min), force idle (0x00) until data resumes
- [ ] **Publish state** back to MQTT (current cfh, duty, last-tx age) for observability
- [ ] **Retire / gate the HTTP test endpoints** behind a debug build flag
- [ ] Field test against the real Watts receiver (one zone)

## M2 — RX path + pairing capture

Goal: the radio can receive, so new device IDs can be registered by switching a
thermostat off.

- [ ] **CC1101 receive config** + GDO interrupt / FIFO read
- [ ] **Software Manchester decode** of received frames (mirror the encoder)
- [ ] **Validate** sync word + both CRCs on RX
- [ ] **Off-frame detector** — setpoint 0.0 + `FF FF FE` shape → capture device ID (bytes 13..15)
- [ ] **Pairing-capture flow** — arm "listen for off-frame", surface the captured ID
- [ ] (Optional) sniff/log real thermostat frames for cross-checking

## M3 — Multi-zone + device registry

Goal: all five MVP zones, persisted, each bound to a Z2M thermostat.

- [ ] **NVS device registry** — per channel: name, device ID, source topic, field map
- [ ] **Per-zone control loop instances** (5 zones: 4 up + 1 down)
- [x] **Z2M discovery** (pulled forward, used to satisfy M1's subscribe step) —
      subscribes to retained `zigbee2mqtt/bridge/devices`, reassembles the fragmented
      payload, filters for `climate`-type devices, and reads `exposes` for the state
      topic + field names (temp / setpoint / mode). Zero-config for arbitrary
      (non-W100) thermostats. Re-runs on every inventory republish. Discovered
      thermostats held in a fixed registry (`MAX_THERMOSTATS`).
- [ ] **Binding workflow** — associate a captured Watts channel with a discovered Z2M thermostat
- [ ] **Pair a new virtual device ID** to replace the broken thermostat

## M4 — Home Assistant UX (headless)

- [ ] **Per-zone parental lock** toggle — gate W100 local button input on/off
- [ ] **MQTT discovery entities** — expose status + lock per zone without cluttering
      the climate dashboard
- [ ] Document the HA/MQTT topic contract

## Testing & validation (cross-cutting)

How each layer is checked. The current and primary method is end-to-end against
real hardware, which is strong evidence — these items are about catching *future*
regressions and the failure modes that hardware testing doesn't exercise.

**Current method (works, keep using):**
- [x] Frame/CRC correctness — forked rtl_433 decodes TX frames byte-for-byte
      alongside the real thermostats; actuates the real Watts receiver
- [x] Control model — Python emulator validated against an overnight capture (`tools/wfht_emulator.py`)

**Per-milestone validation to add:**
- [ ] **M1** — field test one zone against the real receiver (already listed in M1)
- [ ] **M1** — failsafe test: stall MQTT updates, confirm the stale-data timeout
      forces idle (0x00) — a path hardware "happy path" testing never hits
- [ ] **M1** — MQTT resilience: kill/restart the broker and WiFi, confirm reconnect
      and that the loop keeps running on last-known values
- [ ] **M2** — RX loopback: encode → decode roundtrip in firmware; assert recovered
      bytes + CRCs match the source frame
- [ ] **M3** — registry persistence: write channels to NVS, reboot, confirm they reload
- [ ] **M3** — multi-zone isolation: confirm a per-channel device ID change doesn't
      corrupt other zones' frames (the refactor most likely to silently break CRCs)

**Optional (deferred — manual hardware loop deemed sufficient for now):**
- [ ] Host-side unit tests (`pio test -e native`) for the pure functions
      (`buildFrame`, `crc8_raw`, `crc16_cms`, `manchesterEncode`) asserting against
      known-good captured frames. Cheap regression tripwire; revisit if a silent
      frame/CRC regression ever bites, or before the M3 multi-zone refactor.

## Post-MVP (deferred, design only)

- [ ] Cooling + dewpoint safety + downstairs humidity contact (`docs/safety/cooling-and-dewpoint.md` — not yet written)
- [ ] Downstairs zone subdivision
- [ ] Upstream protocol findings to mainline rtl_433
