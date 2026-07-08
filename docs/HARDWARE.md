# Hardware

Bill of materials and wiring for the Watts WFHT-RF bridge: an **ESP32 + CC1101**
transmitter/receiver on the 433.92 MHz ISM band. This replaces the legacy Watts
WFHT-RF thermostats by talking directly to the Watts central receiver.

## Bill of materials

| Part      | Choice                                              | Notes                                                |
| --------- | --------------------------------------------------- | ---------------------------------------------------- |
| MCU board | ESP32-WROOM-32, 38-pin DevKit                       | Owns WiFi, MQTT, control loop, RF                    |
| RF module | Ebyte **E07-400M10S** (CC1101, 410–450 MHz, 10 dBm) | Generic CC1101 breakouts also work — same SPI wiring |
| Antenna   | 433 MHz λ/4 whip, SMA female                        | See [Antenna](#antenna)                              |
| Pigtail   | u.FL (IPEX) → SMA, ~10–15 cm                        | Only if the module has a u.FL connector              |

Everything in the RF path is **50 Ω**.

## ESP32-WROOM-32 38-pin DevKit

The 38-pin variant of the DevKitC. Powered over USB (5 V → onboard regulator →
3.3 V rail). **The CC1101 runs from the 3.3 V pin, never 5 V** — its I/O is not
5 V tolerant.

## CC1101 wiring

The ESP32 talks to the CC1101 over **VSPI**. Only `GDO0` is wired for data;
`GDO2` is unused. `GDO0` carries the raw asynchronous OOK envelope in both
directions — the firmware clocks the TX bitstream out on it, and on RX it sets
`IOCFG0=0x0D` (demodulated envelope out) and edge-times the bits with a pin-change
interrupt. Pin numbers are the firmware defaults from
`firmware/include/config.example.h` — change them there if you rewire.

| CC1101 pin | Signal                        | ESP32 GPIO    | Wire   | Macro              |
| ---------- | ----------------------------- | ------------- | ------ | ------------------ |
| VCC        | 3.3 V power                   | **3V3**       | —      | Not 5 V            |
| GND        | Ground                        | GND           | —      |                    |
| SCK        | SPI clock                     | GPIO18        | blue   | `PIN_SCK`          |
| MISO       | SPI data out (a.k.a. GDO1/SO) | GPIO19        | purple | `PIN_MISO`         |
| MOSI       | SPI data in (SI)              | GPIO23        | orange | `PIN_MOSI`         |
| CSN        | SPI chip select (SS)          | GPIO5         | yellow | `PIN_CS`           |
| GDO0       | Raw OOK data (TX+RX)          | GPIO4         | green  | `PIN_GDO0`         |
| GDO2       | —                             | not connected | —      | unused by firmware |

> **Pin choice:** `GDO0` is on GPIO4, a plain GPIO (not a boot-strapping pin), so
> it needs no special handling at flash/boot time. Any free non-strapping GPIO
> works — change `PIN_GDO0` in `config.h` if you rewire.

### E07-400M10S notes

- SMD module, 14×20 mm, 1.27 mm half-hole (castellated) footprint — solder to a
  breakout or reflow onto a carrier board.
- Operating voltage 1.8–3.6 V, 3.3 V logic.
- Pin labels follow the generic CC1101 map above (VCC, GND, SCK, MISO, MOSI,
  CSN, GDO0, GDO2). Confirm against the module's own silkscreen/datasheet before
  soldering.

## Antenna

The E07-400M10S exposes **two** antenna options on the board:

1. **u.FL (IPEX)** connector — snap-on. Use a u.FL→SMA pigtail out to a real antenna.
2. **Stamp hole** — a 50 Ω castellated pad to solder a wire/spring antenna directly.

### Connector reference

| Connector | What it is | Where |
|---|---|---|
| **u.FL / IPEX** (IPX, MHF-1) | Tiny ~2 mm snap-on board-to-cable connector. Fragile, ~30 mating cycles. | On the module |
| **SMA** | Screw-on 50 Ω standard, the "antenna end." | Antenna + far end of pigtail |
| **RP-SMA** | Reverse-polarity SMA — same threads, swapped center pin. **Common on WiFi gear.** | Avoid — see below |

**Traps:**
- **SMA vs RP-SMA:** they screw together but the reversed center contact means
  no connection. Standardize on **plain SMA** for 433 MHz and check every listing.
- **Wrong band:** a generic "3 dBi" stick is tuned for 2.4 GHz WiFi and barely
  radiates at 433 MHz. Buy an antenna explicitly labeled **433 MHz**.
- Standard mating: **SMA-female antenna** onto an **SMA-male pigtail**.

### Recommended: quarter-wave whip

- **λ at 433.92 MHz ≈ 691 mm → λ/4 ≈ 173 mm (17.3 cm).**
- A tuned 433 MHz SMA rubber-duck is already cut to this. For the stamp-hole
  option, a straight **17.3 cm** wire soldered to the pad is a working λ/4 whip
  (benefits from a ground plane / counterpoise of similar length).

### Using one antenna across modules

Generic CC1101 boards ship with one of three antenna interfaces:

1. **u.FL / IPEX** → same pigtail + antenna as the E07. Fully interchangeable.
2. **Onboard SMA** → the SMA antenna screws straight on, no pigtail.
3. **Stamp hole / spring only** → no removable connector; solder a pigtail or a
   bare λ/4 wire.

**Recommendation:** standardize the whole bench on **SMA** as the common
denominator (u.FL→SMA pigtail on every u.FL board), then a single 433 MHz SMA
antenna moves freely between radios — or buy two identical antennas so the
bridge and a sniffer can both stay live. Leave the fragile u.FL mated once inside
the enclosure and do all swapping on the robust SMA outside.

## References

- E07-400M10S product page — https://www.cdebyte.com/products/E07-400M10S/1
- E07-400M10S user manual — https://device.report/manuals/e07-400m10s-user-manual
- RF protocol details — [WATTS_WFHT_RF_PROTOCOL.md](WATTS_WFHT_RF_PROTOCOL.md)
