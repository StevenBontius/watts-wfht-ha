# Aqara W100 — Zigbee2MQTT field reference

The MQTT-side input contract for the bridge, the analog of
`WATTS_WFHT_RF_PROTOCOL.md` for the radio side. The W100 reaches the ESP as
Zigbee2MQTT JSON on `zigbee2mqtt/<friendly-name>`; the bridge binds to the field
names below.

**Z2M model:** `TH-S04D` (Aqara W100 climate sensor). Fields taken from the
published Z2M device definition — no live capture needed.

## Fields the bridge consumes

| Bridge role | Z2M field | Type / range | Notes |
|---|---|---|---|
| Ambient `T` (control input) | `temperature` | numeric, °C | `local_temperature` carries the same value; use `temperature`. |
| Setpoint `SP` (control input) | `occupied_heating_setpoint` | numeric, 5–30 °C, **settable** | HA schedules/automations can write this too. |
| Heat/cool mode (Watts byte 12 bit) | `system_mode` | enum: `off`/`heat`/`cool`/`auto` | `heat` → 0x02, `cool` → 0x00. |
| Enrollment / failsafe trigger | `system_mode: off` | enum value | The deliberate switch-off |

## Deliberately ignored

- `action` (button/scene events) — **not consumed.** The W100's ±/center buttons
  change `occupied_heating_setpoint` on-device, so local adjustments already reach
  the bridge through that field; the bridge reads the resulting setpoint, never the
  button event. (M4 parental lock would override these device-side setpoint changes
  upstream, not gate button events.)
- `battery` — Z2M/HA diagnostic concern, not the ESP's.
- `humidity` — not used in the heating MVP (relevant only to deferred
  cooling/dewpoint work).

## Notes

- `temperature` and `local_temperature` are identical on this device