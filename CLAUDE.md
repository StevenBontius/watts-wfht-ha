# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

This is a reverse-engineering and firmware project to replace Watts WFHT-RF legacy underfloor heating thermostats with a headless Home Assistant bridge (ESP32 + CC1101). Phase 1 (RF protocol characterization) is complete. The firmware is a **standalone Arduino / PlatformIO app** (`firmware/`) — not ESPHome; the ESP owns WiFi, MQTT, the control loop, the NVS device registry and HA discovery itself. The single-zone live bridge is confirmed on real hardware — the full HA → MQTT → ESP32 → CC1101 → receiver path actuates the real Watts receiver end-to-end. RX/pairing-capture and persisted multi-zone bindings are implemented; remaining work (simultaneous five-zone validation, failsafe/resilience tests, headless HA UX) is tracked in `docs/ROADMAP.md`.

## Running the emulator

```bash
python tools/wfht_emulator.py captures/thermostat.json
```

The emulator reads newline-delimited JSON (output of `rtl_433 -F json`), clusters frames into A-B-A bursts, auto-detects the cycle phase anchor, and prints a match-rate summary against the actual captured flag bytes.

## Capturing RF packets

```bash
rtl_433 -f 433.92M -X 'n=Watts_MC,m=OOK_MC_ZEROBIT,s=460,l=0,r=900' -F json
```

The forked rtl_433 decoder (`StevenBontius/rtl_433`) handles CRC validation and A-B-A burst parsing natively.

## Building the firmware

```bash
cd firmware
cp include/config.example.h include/config.h   # fill in WiFi creds + CC1101 pins
pio run -t upload          # build + flash ESP32 over USB
pio device monitor         # serial @ 115200
```

The firmware is MQTT-driven: it subscribes to Z2M thermostat state and transmits
per bound zone on the 154 s heartbeat (or on change), with RX, pairing capture,
and NVS-persisted bindings. The HTTP server remains as the operational/debug
control surface (not yet gated behind a build flag): `GET /status`, `/tx-test`,
`/tx-watts`, `/tx-pair` (accepts `id=` to pair a bound zone's device ID into the
receiver), `/tx-pair-status`, `/rx-on`, `/rx-off`, `/pair-listen`, `/pair-status`,
`/pair-cancel`, `/bind`, `/unbind`, `/bindings`. `MANCHESTER_ONE_IS_10` in
`src/main.cpp` is the one physical-layer polarity knob — flip it if rtl_433 is
silent or reports bit-inverted CRC failures.

## Architecture

The system has two layers:

**Protocol layer** (fully characterized, documented in `docs/WATTS_WFHT_RF_PROTOCOL.md`):
- 433.92 MHz OOK/Manchester, 192-bit fixed frames, 0xD391 sync word
- Frames transmitted in A-B-A bursts every 154 s in steady state, or immediately on state change
- Byte 12: mode/pairing flags. Bytes 13-15: 3-byte device ID. Bytes 16-19: ambient + setpoint (×10, big-endian). Byte 20: call-for-heat (0x00 or 0x64). Byte 21: custom CRC-8 (`poly=0xE6, xor=0xBE`, then XOR with byte 20). Bytes 22-23: radio CRC-16/CMS

**Control layer** (characterized, implemented in `tools/wfht_emulator.py`):
- Pure P controller: `duty = clip((SP - T) / Bp, 0, 1)`
- Anti-short-cycle clamps: ON clamped to `[On_min, Cy - Of_min]` when duty > 0
- Defaults: `Bp=2.1°C` (effective firmware value), `Cy=900s`, `On_min=Of_min=120s`
- Demand-onset mid-cycle exception: if duty transitions 0→positive mid-cycle, fires an immediate ON pulse for at least `On_min`
- User SP change driving error negative cuts the active ON pulse immediately (bypasses `On_min`)

**Planned runtime stack:**
- Aqara W100 → Zigbee2MQTT → Home Assistant → MQTT broker → ESP32 (standalone Arduino/PlatformIO firmware) → CC1101 → Watts central receiver → manifold servo valves

## Key invariants

- The CRC-8 final XOR includes byte 20. A decoder that omits this step silently passes all idle frames and fails only on active heat-call frames — a real trap.
- The B frame in each A-B-A burst has setpoint + 0.1 °C (one LSB); the A frames carry the real setpoint. When replaying, take the minimum setpoint within a burst as the true value.
- Pairing: set bit 0 of byte 12 to 1 and transmit at 2 Hz (~500 ms cadence) instead of 154 s.
- Spoofing: clone the 3-byte device ID from bytes 13-15 of a captured frame.

## Captures

`captures/` holds representative rtl_433 JSON logs used for protocol characterization and emulator validation. `captures/thermostat.json` is the primary overnight validation dataset.
