# Watts WFHT-RF / WFHC-MASTERH&C-RF Protocol

Reverse-engineering notes for the wireless underfloor heating system sold by
Watts under the WFHC-MASTERH&C-RF central controller and the compatible
WFHT-BASIC-RF and WFHT-LCD-RF (also marketed as WFHTRF 010) thermostats.

## Hardware and methodology

The thermostat is built around a Renesas R5F10WMEA microcontroller paired
with a Texas Instruments CC110L sub-1-GHz transceiver. The CC110L drives the
RF link with its standard preamble, sync word, length-byte, payload, and
CRC-16 framing; the application-layer payload and the application-layer
CRC-8 (byte 21) are produced by the R5F10WMEA firmware.

Reverse engineering was carried out with an RTL-SDR V3 dongle and rtl_433.
Most of the work used a second-hand WFHT-RF unpaired from any receiver,
which broadcasts regardless of pairing state and could therefore be exercised
on the bench without disturbing a household heating system. Where the
control-loop characterization required long unattended logging, the bench
unit ran overnight with one chosen parameter set and the resulting broadcast
log was compared against the output of a Python emulator implementing the
hypothesized control law.

## 1. RF characteristics and capture

| Parameter                 | Value                                             |
| :------------------------ | :------------------------------------------------ |
| Nominal frequency         | 433.92 MHz                                        |
| Practical receive tunings | 433.92 MHz or 434.0 MHz both work                 |
| Modulation                | OOK with Manchester encoding, zero-bit convention |
| Symbol period (short)     | ~460 µs (Manchester half-bit)                     |
| Reset limit               | ~900 µs (inter-frame gap)                         |
| Bitrate                   | ~1085 bps (chip rate ~2170 chips/s)               |
| Frame length              | 192 bits (24 bytes), see note                     |

**Frame length note.** Captures occasionally show a 193rd "phantom" bit at
the trailing edge of the frame, producing a hex tail like `...59d88` instead
of the expected `...59d`. This is a Manchester alignment artifact from the
end of the transmitter's bit clock and carries no payload meaning. A robust
decoder should accept both 192-bit and 193-bit frames and discard the extra
bit.

### rtl_433 flex decoder (for ad-hoc capture)

```bash
rtl_433 -f 433.92M -X 'n=Watts_MC,m=OOK_MC_ZEROBIT,s=460,l=0,r=900' -F json
```

### Example frames

```
{192} 2aaaaaaad391d3910dfffffe0034942b00fd0000009eb313
{193} 2aaaaaaad391d3910dfffffe0034942b00fd00c864a2643e8
```

Interactive pulse diagrams of these captures:

* {192} https://triq.org/pdv/#AAB10301C003802714819191919191919191919191919191919081918090808180918080908081918090808180918080918080809081908080808080808080808080808080808080808080808081808080808080808080809081918091809191808080919190818080808080808090808080808191808080808080808080808080808080808080808080808091809080808191908180908180809180908255
* {193} https://triq.org/pdv/#AAB10301C003802714819191919191919191919191919191919081918090808180918080908081918090808180918080918080809081908080808080808080808080808080808080808080808081808080808080808080809081918091809191808080919190818080808080808090808080808191808080808080809081809180808090818091809191808091809081809180808090808080819255

## 2. Frame structure

All frames observed in normal operation are exactly 192 bits. The byte
layout is:

| Byte  | Example    | Confirmed function                                       |
| :---- | :--------- | :------------------------------------------------------- |
| 0-3   | `2AAAAAAA` | Preamble (CC110L sync alignment)                         |
| 4-7   | `D391D391` | Sync word, the doubled CC1101 default `0xD391`           |
| 8     | `0D`       | Length: 13 bytes follow (bytes 9 through 21)             |
| 9-11  | `FFFFFE`   | Protocol header, static across all observed traffic      |
| 12    | `02`       | Mode/state bit field, see Section 3                      |
| 13-15 | `34942B`   | Device ID, 3-byte unique hardware identifier             |
| 16-17 | `00FD`     | Ambient temperature, 16-bit big-endian, value / 10 in °C |
| 18-19 | `00EC`     | Target setpoint, 16-bit big-endian, value / 10 in °C     |
| 20    | `64`       | Call-for-heat flag, see Section 3                        |
| 21    | `C8`       | Application CRC-8, see Section 3                         |
| 22-23 | `BB27`     | Radio CRC-16/CMS, see Section 3                          |

The CC110L hardware on a paired receiver would normally validate and strip
bytes 22-23 before passing the payload to the firmware. Because rtl_433 sees
raw demodulated bits without the chip-level filtering, it must validate the
CRC-16 itself.

**Steady-state transmission cadence.** A thermostat that is paired and not
being interacted with broadcasts roughly once every **154 seconds** in
steady state. Two events break this cadence:

1. **State change.** Any transition of byte 12 (mode change) or byte 20
   (heat call on/off) triggers an immediate broadcast.
2. **Transmittal pattern.** All transmissions follow the A-B-A pattern, see Section 3.
3. **Pairing mode.** Cadence switches to ~2 Hz, see Section 4.

## 3. Field semantics and algorithms

### Byte 12: mode and state bit field

| Bit | Mask   | Confirmed meaning                              |
| :-- | :----- | :--------------------------------------------- |
| 0   | `0x01` | Pairing in progress (1 = pairing, 0 = normal)  |
| 1   | `0x02` | Heat (1) versus cool (0) selection             |
| 2-7 | `0xFC` | **Open**, observed as 0 in all normal captures |

The decoder exposes byte 12 as both `mode_byte` (hex string, e.g. `0x02`)
and `mode_bits` (8-character MSB-first binary string, e.g. `00000010`) so
that downstream consumers and future RE work can inspect the upper bits
directly without re-parsing.

### Bytes 13-15: device ID

Three bytes of unique hardware identifier, transmitted MSB first. Formatted
in this document and by the decoder as `XX:XX:XX`. Observed values in the
small captured population all begin with `34`, suggesting either a vendor
prefix or a production-batch artifact; this has not been verified against a
broader sample.

### Bytes 16-17 and 18-19: temperatures

Two 16-bit big-endian temperature values, each representing degrees Celsius
multiplied by 10. **Confirmed assignment** based on a controlled setpoint-bump
experiment (see "Setpoint changes" below):

* **Bytes 16-17 (T1) = ambient temperature**, the thermostat's air sensor
  reading.
* **Bytes 18-19 (T2) = target setpoint**, the value configured by the user.

Examples:

| Hex     | Decimal | °C        |
| :------ | :------ | :-------- |
| `00DF`  | 223     | 22.3 °C   |
| `0100`  | 256     | 25.6 °C   |
| `0104`  | 260     | 26.0 °C   |

### The A-B-A burst

All packets are transmitted following the A-B-A pattern. Pairing also shows this pattern.

| Frame | T (ms) | Setpoint value     | CRC-8    | Notes                  |
| :---- | :----- | :----------------- | :------- | :--------------------- |
| A     | 0      | real value         | `hash_A` | First "real" frame     |
| B     | ~400   | real value + 0.1 °C | `hash_B` | Deliberate perturbation |
| A     | ~800   | real value         | `hash_A` | Second "real" frame    |

The middle frame's setpoint differs from the real value by exactly +0.1 °C
(one LSB of the setpoint encoding). Because the CRC-8 covers the setpoint
bytes (Section 3), this single-bit perturbation propagates avalanche-style
into a completely different `hash_B`. Worked example for a setpoint of
26.0 °C on device `34:94:2B` with ambient 25.7 °C:

```
A: ...0101 0104 00 1e...   T2 = 26.0 °C, hash 1e
B: ...0101 0105 00 f8...   T2 = 26.1 °C, hash f8
A: ...0101 0104 00 1e...   T2 = 26.0 °C, hash 1e
```

### System off / standby mode

When the thermostat is manually turned off via its front panel, bytes 18-19
(setpoint) are transmitted as `0x0000` (0.0 °C). Bytes 16-17 continue to
report the real ambient reading. Byte 20 transitions to `0x00` (no heat
call). Byte 12 retains its heat/cool bit from before standby, suggesting
the mode is preserved across off/on cycles rather than reset.

Even when the thermostat is turned off via the switch on the side it will continue sending 0.0 setpoint packages.

### Byte 20: call-for-heat

A single byte representing the thermostat's output to the central:

* `0x00`: idle (no call for heat)
* `0x64`: calling (100% duty in the current cycle window)

These are the only two values observed. The value is **not** a literal
percentage of the cycle: even during chronoproportional operation where the
firmware modulates the ON fraction over time, each individual broadcast
carries either `0x00` or `0x64` and the central infers duty cycle by
counting transitions over a window. See Section 5 for the full control
characterization.

The value of byte 20 is the output of the thermostat's internal control
loop, not a passive sensor reading. The Watts installer menu exposes PI
gains (`Bp`, `Cy`), anti-short-cycle clamps (`On`, `Of`), and a
compensation offset (`Cp`), which are only meaningful if the device runs
its own loop and broadcasts the result. Section 5 details the
characterization of that loop.

### Byte 21: application-layer CRC-8

A custom CRC-8 over bytes 8 through 19, with a constant XOR and a final
mix-in of byte 20:

| Parameter | Value      |
| :-------- | :--------- |
| Polynomial | `0xE6`    |
| Init      | `0x00`     |
| RefIn     | false      |
| RefOut    | false      |
| XorOut    | `0xBE`     |
| Coverage  | bytes 8..19 |
| Post-step | XOR with byte 20 |

```
byte_21 = crc8(bytes_8_to_19, poly=0xE6, init=0x00) XOR 0xBE XOR byte_20
```

The final XOR with byte 20 means a decoder that computes the CRC over
bytes 8..19 only, without mixing byte 20 in at the end, will silently pass
all frames where byte 20 is `0x00` (the idle state, by far the most common
in normal operation) and fail only when the heater is actively calling.
This is a real false-positive trap; the rtl_433 decoder in this repository
includes a test vector specifically to catch it.

### Bytes 22-23: radio-layer CRC-16/CMS

The standard CRC-16/CMS algorithm, identical to what a properly configured
CC1101 / CC110L computes in hardware:

| Parameter | Value         |
| :-------- | :------------ |
| Polynomial | `0x8005`     |
| Init      | `0xFFFF`      |
| RefIn     | false         |
| RefOut    | false         |
| XorOut    | `0x0000`      |
| Coverage  | bytes 8..21   |

Many CRC-16 variants share polynomial `0x8005` and differ only in reflection
and XorOut parameters; CRC-16/MODBUS and CRC-16/ARC are the most common
look-alikes. Implementers should use the exact parameter set above to avoid
silent mismatches.

## 4. Pairing

When the user places a thermostat into pairing mode, it leaves the 154-second
steady-state cadence and transmits at approximately **2 Hz** (one frame every
~500 ms) with **bit 0 of byte 12 set to 1**.

This rapid cadence is itself a strong on-air signature: any frame from this
protocol arriving twice per second is almost certainly a pairing
announcement. A passive listener can detect a pairing attempt by timing
alone, without parsing the payload bit field.

### Byte 12 during pairing

| Bit | Mask   | Value during pairing | Meaning                              |
| :-- | :----- | :------------------- | :----------------------------------- |
| 0   | `0x01` | **1**                | Pairing in progress                  |
| 1   | `0x02` | preserves user mode  | Heat (1) or cool (0) as configured   |
| 2-7 | `0xFC` | observed as 0         | See open questions                   |

A pairing frame from a thermostat configured for heat has byte 12 = `0x03`;
configured for cool, `0x01`.

## 5. Control-loop characterization

The initial working hypothesis was that the thermostat is a passive sensor
and that all control logic lives in the receiver. This hypothesis was wrong.
The installer-menu parameters of the device expose PI gains (`Bp`, `Cy`),
anti-short-cycle constraints (`On`, `Of`), and a compensation offset
(`Cp`). A device exposing such parameters runs its own control loop; byte
20 is the output of that loop. Characterizing it requires reconstructing
not just an encoding but the algorithm that produces it.

During testing it was confirmed that the simple EFHRFR 001 receiver only listens to the call for heat flag for controlling the valve. Actual temperatures and setpoints are not checked. So the EFHRFR 001 receiver relies on the control loop within the thermostat. Further research will need to be performed to see whether the other system does use the temperatures and ignores the call for heat flag. If the receiver does not receive a packet within a certain time (have't timed it but seems like 15 minutes) it will go into error mode. Green light flashing slow.

### Approach

An early test plan listed many small experiments (hysteresis band
measurement, step responses at different setpoints, gain-scaling sweeps,
anti-short-cycle probes). This was replaced by a more economical
methodology: log the device unattended for several hours under one chosen
parameter set, then let a software emulator reproduce its broadcasts.
Where the model matches reality no further investigation is needed; where
it diverges, the divergence localizes the unknown.

A single overnight log exercises many behaviors at once: steady-state cycle
dynamics over many cycles, the full range of duties as ambient drifts, the
anti-short-cycle clamps at both extremes, and the broadcast cadence.

### Overnight chronoproportional capture

Nine hours at `J7 = rEg`, setpoint = 24.5 °C, `Cp = 0`, with ambient
drifting from 24.9 °C down to 22.7 °C as the room cooled.

![Overnight chronoproportional pattern](/references/plots/thermostat_analysis.png)

The duty cycle per 15-minute window tracks the `(SP − T)` error within 2.6
percentage points on average. A proportional model with `Bp = 2.0 °C`,
`Cy = 900 s`, `On_min = Of_min = 120 s` captures the behavior; the `On_min`
and `Of_min` clamps are clearly visible at the duty extremes.

### Integral action ruled out

The overnight result is consistent with a P-only controller but does not
rule out a slow integrator. The direct test: drop the setpoint well below
ambient after many hours of high-duty operation. If an integrator existed,
it would be wound up to saturation and would continue to drive ON pulses
for several cycles afterwards as it drained.

![SP-drop test](/references/plots/sp_drop_test.png)

Zero residual calls in the 28 minutes after the drop. The loop is pure P,
with no memory of past error.

### Emulator validation

A Python implementation of the loop, fed the same setpoint and ambient
stream that the bench unit saw, was compared against the actual broadcasts.

![Emulator validation](/references/plots/emulator_validation.png)

96.2% byte-identical match (254 of 264 broadcasts). The ten mismatches all
fall within 30 seconds of a cycle transition, attributable to a small
firmware overshoot in `Bp`: an effective value of 2.1 instead of the
menu-set 2.0 reproduces the transitions more closely, presumably an
integer-rounding artifact inside the firmware. The red bands in the plot
mark broadcasts where the emulator's prediction disagreed with the actual
byte 20 value.

### Loop in pseudocode

```python
# At every cycle boundary (every Cy seconds, free-running)
duty = clip((SP - T) / Bp, 0, 1)
on_time = duty * Cy
if 0 < on_time < On_min:    on_time = On_min        # anti-short-cycle floor
if on_time > Cy - Of_min:   on_time = Cy - Of_min   # anti-short-cycle ceiling
# Hold flag = 0x64 for on_time seconds, then flag = 0x00 for the rest of the cycle

# Mid-cycle exceptions:
# 1. Demand onset: if duty was 0 last cycle and is now > 0, fire an immediate
#    out-of-cycle ON pulse held for at least On_min, then wait for next boundary.
# 2. User SP change driving error below 0: cut the active ON pulse immediately,
#    bypassing On_min. Loop-driven transitions still honour On_min and Of_min.

# Broadcast scheduling (independent of the loop):
# - every 154 seconds in steady state
# - plus an immediate burst on any state change (flag transition or SP change)
```

Parameter values used for the validated case: `Bp = 2.1 °C` (effective;
menu value 2.0), `Cy = 900 s`, `On_min = Of_min = 120 s`. `J1 = Hot`
(heating direction), `Cp = A0 = 0` (no offset).

### Scope of the model

The model is restricted to `J7 = rEg` (proportional mode). The hysteresis
mode `J7 = hys` is not implemented. Floor sensor inputs, `FL` and `FH`
limits, and the `J1 = CLd` direction inversion are also not implemented;
only the heating path through the air sensor.

The proportional-mode loop is fully characterized and the emulator
reproduces broadcasts byte-identically except for transition-edge effects
accountable to firmware rounding. Behaviors outside proportional mode
require only small targeted experiments to settle but are out of scope here.

## 6. Open questions and known unknowns

Consolidated list of items consistent with the data but not yet directly
confirmed by experiment, in rough priority order for anyone continuing the
work:

1. **Central-to-thermostat traffic.** Every byte described here is a
   thermostat broadcast. A passive SDR session with the central powered and
   no protocol filter would reveal whether the central also transmits.
2. **Hysteresis mode (`J7 = hys`).** Out of scope for the proportional-mode
   characterization; a 1-hour bench session would settle it.
3. **Compensation offset (`Cp` non-zero).** Whether `Cp` shifts byte 16-17
   (the broadcast ambient reading) or only the internal error signal used
   by the loop. Trivial to test: set `Cp` to a non-zero value and compare
   the broadcast value to the LCD display. Out of scope.
4.  **Floor sensor inputs and `FL` / `FH` limits.** Out of scope

## 7. References and supporting materials

* **rtl_433 decoder.** `src/devices/watts_wfht_rf.c` in the rtl_433 source
  tree, registered as `Watts-WFHT-RF`. Includes both CRC algorithms with
  the exact parameters above, the A-B-A burst handling (emitted faithfully
  as three frames), and `mode_bits` / `flag_bits` binary representations
  for ongoing reverse engineering.
* **Test captures.** Stored in the `rtl_433_tests` repository under
  `tests/watts/wfht_rf/` (location TBD until merged), representative
  `.cu8` files for idle, heat call, off, A-B-A setpoint change, and
  pairing burst states.
* **Python emulator.** Implements the proportional-mode loop and broadcast
  scheduling described in Section 5. Source location TBD; intended for
  future inclusion in this repository as `tools/watts_emulator.py`.

