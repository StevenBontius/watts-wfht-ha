# Watts WFHT-RF LCD Protocol Reverse-Engineering Data

The Watts thermostat has a R5F10WMEA microcontroller and a CC110L 433 MHZ chip.

## 1. RF Characteristics & Capture
Based on raw signal analysis, this specific Watts thermostat variant uses Manchester encoding.

* **Frequency:** 433.92 MHz
* **Modulation:** Manchester (`OOK_MC_ZEROBIT`)
* **Short pulse width:** ~460 µs
* **Long pulse width:** ~900 µs
* **End-of-Transmission Artifact:** Captures frequently show a 193rd "phantom" bit resulting in a trailing hex `8` (e.g., `...59d88`).

**Verified rtl_433 Flex Decoder:**
```bash
rtl_433 -f 433.92M -X 'n=Watts_MC,m=OOK_MC_ZEROBIT,s=460,l=0,r=900'  -F json

```

[Example {192} 2aaaaaaad391d3910dfffffe0034942b00fd0000009eb313](https://triq.org/pdv/#AAB10301C003802714819191919191919191919191919191919081918090808180918080908081918090808180918080918080809081908080808080808080808080808080808080808080808081808080808080808080809081918091809191808080919190818080808080808090808080808191808080808080808080808080808080808080808080808091809080808191908180908180809180908255)

[Example {193}2aaaaaaad391d3910dfffffe0034942b00fd00c864a2643e8](https://triq.org/pdv/#AAB10301C003802714819191919191919191919191919191919081918090808180918080908081918090808180918080918080809081908080808080808080808080808080808080808080808081808080808080808080809081918091809191808080919190818080808080808090808080808191808080808080809081809180808090818091809191808091809081809180808090808080819255)

## 2. Payload Structure

The valid payload is consistently 192 bits (24 bytes) long.

| Byte Index | Example Hex | Verified Function                                      |
| ---------- | ----------- | ------------------------------------------------------ |
| **0-3**    | `2aaaaaaa`  | Static Preamble                                        |
| **4-7**    | `d391d391`  | Static Sync Word (32-bit)                              |
| **8**      | `0d`        | Length Indicator (13 bytes follow: 9 through 21)       |
| **9-11**   | `fffffe`    | Static Protocol Header                                 |
| **12**     | `00`        | **Operating Mode** (`00` = Heating, `02` = Cooling) *  |
| **13-15**  | `34942b`    | **Device ID** (3-byte unique hardware identifier)      |
| **16-17**  | `0100`      | **Ambient Temperature T1** (16-bit big-endian)         |
| **18-19**  | `00ec`      | **Target Setpoint Temperature T2** (16-bit big-endian) |
| **20**     | `00`        | **Relay State** (`00` = Demand/ON, `64` = Rest/OFF)  * |
| **21**     | `c8`        | **Hardware Hash** (CRC-8 XORed with Byte 20)           |
| **22-23**  | `bb27`      | **Radio Checksum** (CRC-16/CMS over Bytes 8..21)       |

* to be confirmed

## 3. Verified Data Mappings & Algorithms

### Temperature Values (Bytes 16-17 & 18-19)

Temperatures are transmitted as 16-bit integers representing the Celsius value multiplied by 10.

* **Calculation:** Combine High Byte and Low Byte, convert Hex to Decimal, divide by 10.
* **Examples:**
* `00df` = 223 ➔ 22.3°C
* `0100` = 256 ➔ 25.6°C

### Adjusting the setpoint on a thermostat

When adjusting the setpoint on a thermostat the thermostat executes a rapid **A-B-A burst transmission**. The thermostat intentionally alters the Target Temperature (T2) by **+0.1°C** for the middle packet. Because Bytes 8-19 run through the heavy CRC-8 polynomial, this single bit-flip completely mutates the resulting Byte 21 Hash.

### Verified Sequence Example (Setting Target to 26.0°C)

*Device ID: `34942b` | Ambient T1: 25.7°C (`0101`)*

| Sequence     | Time       | Hex Snippet (T1 - T2 - Relay - Hash) | Target (T2)         | Hash (B21) |
| :----------- | :--------- | :----------------------------------- | :------------------ | :--------- |
| **A (Real)** | `T=0`      | `...0101 0104 00 1e...`              | **26.0°C** (`0104`) | **`1e`**   |
| **B (Fake)** | `T=~400ms` | `...0101 0105 00 f8...`              | **26.1°C** (`0105`) | **`f8`**   |
| **A (Real)** | `T=~800ms` | `...0101 0104 00 1e...`              | **26.0°C** (`0104`) | **`1e`**   |

## 6. System Off / Standby Mode

When the thermostat is manually turned OFF, the device transmits a hardcoded Target Temperature (T2) of **0.0°C** (`00 00`).

### Operating Mode & Relay State (Bytes 12 & 20)

To be confirmed

* **Byte 12 (Mode):** `00` indicates Heating mode. `02` indicates Cooling mode.
* **Byte 20 (Relay):** * `00`: Active call for operation (ON).
* `64`: Idle / Target reached (OFF).

### Checksums and Hashes (Bytes 21, 22, 23)

The protocol utilizes a two-tier validation system.

**1. Byte 21: Hardware Hash**
Calculated using a standard CRC-8 algorithm *exclusively* over Bytes 8 through 19, followed by an XOR-Out, and a final XOR with Byte 20. 

* **Algorithm:** CRC-8
* **Polynomial:** `0xE6`
* **Init:** `0x00`
* **XorOut:** `0xBE`
* **Formula:** `Byte 21 = CRC8(Bytes 8..19) ^ Byte 20`

**2. Bytes 22-23: Radio Checksum**
A standard industry CRC applied over the core protocol data to protect against RF interference.

* **Algorithm:** CRC-16/CMS
* **Input Data:** Bytes 8 through 21
* **Polynomial:** `0x8005`
* **Init:** `0xFFFF`

## 4. Pending Features & Future Work (To-Do)

While the core logic, temperatures, and security hashes are fully mapped, the following edge cases remain for future capture sessions:

* **Check byte 12 and 20**
* **Pairing / Binding:** Because the 433MHz system is simplex (1-way), pairing is likely handled entirely in the receiver's EEPROM (learning the first heard Device ID). However, tests should confirm if Byte 12 adopts a specific "Discovery" flag (e.g., `FF` or `03`) during the thermostat's manual pairing sequence.
* **Low Battery Indication:** Identifying the specific bit or byte that flags a dying battery.
* **Additional Modes:** Determining values for Holiday, Eco, or Anti-freeze modes if supported by the specific hardware variant.
