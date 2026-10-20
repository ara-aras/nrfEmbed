# IR add-on hardware

Extends the stock nRFBox with a long-range IR transmitter and a receiver for
capture/learn. The stock nRFBox board has **no** IR parts, so this is a small
external module wired to two existing ESP32 pins.

Schematic: [`ir-addon-schematic.svg`](ir-addon-schematic.svg)

> Based on [nRFBox by CiferTech](https://github.com/cifertech/nRFBox) (MIT).
> The original PCB and schematic are CiferTech's work and are not part of this
> file. This document only describes the IR add-on.

## Pins (from `nRFBox/config.h`)

| Signal | ESP32 pin | Notes |
|---|---|---|
| `IR_TX_PIN` | GPIO13 | Drives Q1 base through R1 |
| `IR_RX_PIN` | GPIO34 | Input-only pin, fine for a receiver output |
| 3V3 | 3.3 V rail | Receiver supply via R2/C1 |

## Bill of materials

| Ref | Part | Qty | Purpose |
|---|---|---|---|
| D1-D3 | Vishay TSAL6200, 940 nm, 5 mm, +/-17 deg | 3 | IR emitters |
| Q1 | S8050 NPN (or 2N2222) | 1 | Switches the LED current |
| R1 | 470 ohm | 1 | GPIO13 to base |
| R3-R5 | 22 ohm | 3 | One per LED branch |
| U1 | TSOP38238 (or VS1838B) | 1 | 38 kHz receiver |
| R2 | 100 ohm | 1 | Receiver supply filter |
| C1 | 4.7 uF | 1 | Receiver supply filter |

## Design notes

- **Why a transistor:** an ESP32 pin can source only about 20 mA, which gives
  roughly 2-3 m of range. Q1 lets the LEDs run at 100-150 mA peak.
- **Supply rail:** on the stock board the "+5V" net is fed through the battery
  switch, so on battery it is about 3.7-4.2 V. R3-R5 are sized for the low end.
  At 5 V (USB) each branch peaks near 150 mA.
- **One resistor per LED:** do not series-stack the LEDs. At 3.7 V there is not
  enough headroom for two forward drops plus Q1.
- **Pulse rating:** IR bursts are short and ~33 % duty at 38 kHz, so average LED
  current stays well under the 100 mA continuous rating. Check the datasheet of
  whatever LED you substitute.
- **Coverage:** fan the three LEDs outward for a wide room, or point them
  together for maximum distance.
- **Receiver filter:** R2 + C1 follow the Vishay TSOP application note.
- **Carrier frequency:** the TSOP strips the carrier, so captures cannot report
  it. Replay assumes 38 kHz.
- **Expected range:** roughly 8-15 m line of sight. This is an estimate, not a
  measurement. Test on your own devices.

## Untested

This design has not been built or measured yet. Verify polarity, Q1 pinout
(S8050 in TO-92 is E-B-C) and LED current on a bench supply before closing the
case.
