# watts-wfht-ha

Replacing Watts WFHT-RF underfloor heating thermostats with a Home
Assistant bridge running on an ESP32 + CC1101. Heating-first MVP, with
cooling and dewpoint safety designed and parked for a later phase.

**Status:** phase 0, capture and protocol characterisation.

## What this project is

The house has two coupled Watts WFHT-RF systems on 433.92 MHz, sharing
a Panasonic Aquarea heat pump:

- **Upstairs:** four zones (three bedrooms and one bathroom), each
  zone driving two manifold loops, controlled by its own Watts central
  receiver.
- **Downstairs:** one zone covering eight manifold loops across the
  ground-floor open-plan area, controlled by its own Watts central
  receiver.

The two centrals are linked by a cool/heat signal wire that propagates
the cooling mode from one unit to the other. The downstairs central
also has a voltage-free humidity contact input, used as an emergency
stop in the cooling phase. The existing damp-floor problem is
downstairs-only, driven by cooking with a recirculating cooker hood
that dumps humid air at floor level.

The bridge keeps every piece of mechanical hardware in place: both
receivers, the servo valves on both manifolds, all the loops. What
changes is the source of the thermostat packets. Instead of physical
Watts thermostats on the wall, an ESP32 with a CC1101 radio transmits
packets on the same device IDs from a central location, and Home
Assistant drives what those packets contain.

### How a setpoint reaches the floor

```
Aqara W100 (temp + humidity + up/down/scene buttons)
        │
        ▼  (Zigbee via Zigbee2MQTT)
   Home Assistant
        │     measured room temp: relayed directly into the packet
        │     setpoint:           from HA schedule or automation,
        │                         or from W100 up/down buttons,
        │                         subject to the per-zone lock
        ▼
   ESP32 + CC1101
        │
        ▼  (433.92 MHz, spoofed Watts WFHT-RF packet)
  Watts central receiver
        │
        ▼
   Manifold servo for the zone, driving its loop(s)
```

The Watts receiver still makes the actual decision (open or close the
valve) by comparing measured temperature with the setpoint inside the
packet, exactly as it does for the original thermostats. Home Assistant
is not reimplementing a thermostat; it is feeding the existing one its
inputs.

### Local input and parental lock

The Aqara W100's up and down buttons are the in-room control for
adjusting the setpoint of that zone. The button presses come into Home
Assistant as Zigbee events and update the HA setpoint, which the next
transmitted Watts packet carries down to the receiver.

Each zone has a lock toggle in Home Assistant. When a zone is locked,
button presses for that zone are ignored, so a child cannot override
the parent's schedule from their bedroom. Unlocking restores normal
button behaviour. The lock can be flipped per zone from any HA
dashboard.

### Spoofing and pairing

The bridge needs both capabilities in the MVP:

1. **Spoofing existing device IDs.** For thermostats that still work,
   we read their device ID from captured packets and the ESP32 sends
   packets carrying the same ID. The Watts receiver cannot tell the
   difference.
2. **Pairing new virtual device IDs.** One existing thermostat is
   currently broken and needs replacement, and the downstairs zone
   will later be subdivided into multiple separately controlled zones.
   Both require new device IDs to be registered with the Watts
   receivers. This is why pairing must be in the MVP, not deferred.

### What rtl_433 already provides

Watts WFHT-RF is mainline rtl_433 protocol 253 since version 24.10
(PR #2648). The decoder yields device ID, measured temperature,
setpoint, and a flags field. No custom decoder needed.

Remaining unknowns at the start of this project:

- Heartbeat timing: how long without a packet before the Watts receiver
  drops a zone into a failsafe state, and what that failsafe state is
- Pairing procedure: how to register a new device ID with the receiver
- Cooling/heating flag bit: whether the mode lives inside the
  thermostat packet, or comes from a separate broadcast by the central
  controller (deferred to the cooling phase)

### Hardware

- Two Watts WFHT central RF receivers, coupled by a cool/heat signal
  wire, each driving its own manifold with electrothermal servo valves
- Downstairs central has a voltage-free humidity contact input
  (reserved as the cooling-phase emergency stop)
- Watts WFHT-LCD-RF wireless thermostats (currently in service; one is
  broken and is the immediate replacement target)
- Panasonic Aquarea heat pump with HeishaMon (Panasonic-native variant,
  not the OpenTherm bridge)
- Home Assistant OS on a NUC, with Zigbee2MQTT and Philips Hue already
  running
- Aqara W100 climate sensors, one per zone, paired via Zigbee2MQTT for
  access to temperature, humidity, and button events
- RTL-SDR V3 dongle for protocol capture, running on a Mac
- Mechanical extraction ventilation without heat recovery
- (planned) ESP32 DevKit C with CC1101 module for transmitting

## Scope

**In the heating MVP:**

- All four upstairs zones plus the single downstairs zone running
  through the bridge
- Replacement of the broken thermostat as the first live deployment,
  via a newly paired virtual device ID
- Home Assistant owning setpoints (schedules, presence, holiday mode)
  with W100 buttons as the per-room local override
- Per-zone parental lock toggling local button input on or off
- Standard failsafes: notifications on missing sensor data, on the
  ESP32 going offline, on temperature drift outside expected envelopes

**Explicitly deferred to post-MVP phases:**

- Cooling, dewpoint safety, cooking detection, and integration of the
  downstairs humidity contact. Design exists in
  [`docs/safety/cooling-and-dewpoint.md`](docs/safety/cooling-and-dewpoint.md).
  Cooling is a downstairs-only concern in practice; the bathroom and
  bedrooms are not cooled.
- Downstairs zone subdivision into multiple separately controlled
  zones (re-uses the pairing capability built for the MVP)
- Upstreaming protocol findings to rtl_433
