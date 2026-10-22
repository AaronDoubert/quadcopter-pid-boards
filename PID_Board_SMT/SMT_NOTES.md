# PID_Board_SMT: SMT, cost-reduced, digital-pot version of PID_Board

`PID_Board_SMT.kicad_pro` is a separate project. `PID_Board/` is unchanged.
The circuit topology, net names, sheet split and reference designators follow PID_Board.
This file lists what changed, why, and the alternatives that were considered.

Status:
- **Schematic:** ERC reports 0 violations on all 6 sheets, and the netlist matches all 142 intended nets.
- **PCB:** placed and routed on 100 x 100 mm, 2 layers. DRC reports 0 violations, 0 unconnected items and 0 schematic-parity issues (2026-10-21). See section 7.

Prices are JLCPCB/LCSC qty-1 unit prices, looked up on 2026-10-20. Every BOM part has an
`LCSC` field, so KiCad's BOM export can go straight to JLCPCB assembly.

## 1. Part swaps

| Function | PID_Board (THT) | PID_Board_SMT | Why |
|---|---|---|---|
| Op-amps (7x quad) | MCP6004-E/P DIP-14, $1.32 | **LMV324B-SR** (3PEAK) SOIC-14, $0.14 (C248570) | CMOS rail-to-rail in and out, 1 pA bias, 1 MHz, 2 mV Vos, same pinout. Saves about $8. |
| Trimpots (12x) | Bourns 3296W, about $0.84 | **3x MCP4451-104E/ST** quad 8-bit I2C digital pot, $4.23 (C638704) | See section 2. |
| DACs (5x) | MCP4822-E/P PDIP, $5.49 | **MCP4822-E/SN** SOIC-8, $2.30 (C16040) | Same die, and the cheapest 12-bit option per channel (see section 4). Saves about $16. |
| 5 V supply | Pololu D24V5F5 module, about $6 | **TPS54202DDCR** buck, $0.18 + 15 uH FNR4030S + 2x22 uF + 75k/10k FB | Gives 5.07 V. The parts cost about $0.40 in total. |
| Reverse diode | 1N5817 DO-41 | SS14 SMA (basic part) | |
| Integrator reset | CD4066BE DIP | 74HC4066D SOIC-14 | Same pinout and logic, with lower on-resistance at 5 V. |
| Clamp diodes | 4x 1N4148 per axis | 2x **BAV99** per axis (basic part) | One BAV99 is exactly one series pair, so this halves the placements. The pin 3 midpoint is unused. |
| Level shifters | 2N7000 TO-92 | BSS138 SOT-23 | It has a lower Vgs(th), so it has more margin from the 3.4 V gate bias. Pinout is G-S-D. |
| Reference | TL431ACLPG TO-92 | TL431BIDBZR SOT-23 (TI, 0.5 %) | TI DBZ pinout (1 K, 2 REF, 3 A). Generic SOT-23 TL431s often use the TL432 pinout, so don't substitute one. |
| Logic | 74HC138 DIP | 74HC138D SOIC-16 | |
| Passives | Axial resistors, film caps, disc caps | 0603 1 % resistors; X7R/C0G MLCCs | See section 3 for the integrator caps. |
| Test points | 1-pin headers | 1.5 mm SMD pads | Free. |

These stay through-hole: J1, J2, J4–J7, the six loop-select jumpers, and the mounting holes.
They are hand-soldered and take mechanical load.

## 2. Digital pots (replaces RV101 + 11 gain trimpots)

- **Chips:** U203 (roll), U303 (pitch) and U103 (yaw + VSET). All three are MCP4451-104 (100k, 257 taps, volatile).
  They sit on the shim's existing **5 V I2C bus** (`SDA_5V`/`SCL_5V`, pulled up by R1/R3). SDA_5V and SCL_5V are now global labels.
  The addresses are 0x2C, 0x2D and 0x2E, so they don't clash with the MPU-6050 at 0x68.
  The 74HC138's spare outputs stay free.
- **Wiring:** each channel is used as a rheostat, with A tied to W. Then R_BW is about code/256 x 100k + 75 ohm.
  KiCad's multi-unit `MCP4431-xxxx-ST` symbol is used; it has the same pins as the MCP4451.
  Each channel is drawn inline where its trimpot used to be.
- **Channel map and starting codes.** Every channel powers up at code 128.

  | Channel | Roll / Pitch | Yaw |
  |---|---|---|
  | ch0 | P: 128 = 50k (Kp 1.16) | P: 128 |
  | ch1 | I: 171 = 67k (Ki 10/s) | I: 128 |
  | ch2 | D: 128 = 50k (Kd 9.7 ms) | D: 128 |
  | ch3 | Angle P: 64 = 25k (Go 1.75) | VSET trim |

  The angle stage used a 50k trimpot. It now uses a 100k channel with the same fixed resistors and code 64.
  The range is wider (0.5 to 5.5) and the step is 390 ohm.
- **VSET:** the 10k divider pot became VREF_BUF → 47.5k → VSET → (U103 ch3 + 165k) → GND, plus 100 nF on VSET.
  Code 128 gives about 2.044 V. The range is 1.94–2.12 V, which covers TL431, resistor and ±20 % pot tolerance.
  The step is **0.67 mV**, so the < 1 mV trim spec still holds.
- **Firmware (shim U1):** the pots are volatile. Keep the codes in the ATtiny's EEPROM and write them at boot, before ARM is allowed.
  The wiper registers are 0x00, 0x01, 0x06 and 0x07, and the TCON default is fine.
  Because I2C now sets the gains, you can live-tune them from the RadioMaster's sliders over CRSF.
- **Alternatives:**
  - **MCP4461-104** (non-volatile EEPROM, same pins). Gains survive without firmware, but it is about +$0.50 per chip and has thin JLC stock (the 100k part was not stocked).
  - **TPL0102-100** (TI, dual, 256 taps, NV, $1.83, 1.7k in stock). Six chips cost about $11 instead of $12.70. It needs a custom symbol and 6 addresses.
  - **MCP4131/MCP4231** (SPI, 7-bit). They are cheaper per chip, but need 12 chip selects or SPI daisy-chaining.
  - **X9C104** up/down pots. They are cheap but have only 100 taps and need one CS each.

  MCP4451 won on chip count, board area, symbol availability and bus reuse.
  Watch the stock: JLC had 43 of the 104 parts. DigiKey and Mouser carry plenty.

## 3. Things to know about the SMT passives

- **Integrator and bench-plant caps (5x 1 uF):** 1 uF C0G in 1206 is not stocked at JLC, and SMD film caps aren't either.
  I used **50 V X7R 1206** (basic part). At ≤ 2.5 V of bias the capacitance loss is a few percent, and the Ki pot trims it out anyway.
  X7R does have more dielectric absorption and a wider tolerance than film.
  If integrator drift or windup recovery looks off on the bench, hand-fit a 1 uF PPS film cap (e.g. Panasonic ECHU 1210) in the same footprint area.
- **D-path input cap (100 nF):** X7R 0805, 50 V. The 10 nF D roll-off caps stay C0G.
- **DAC RC filters (1k / 1 uF):** X5R 0603. The 160 Hz corner can move about 10 %, which doesn't matter here.

## 4. Alternatives considered for the major parts

| Part | Picked | Also looked at | Verdict |
|---|---|---|---|
| Op-amp | LMV324B-SR $0.14 | MCP6004T-I/SL $0.49 (drop-in); TLV9004IPWR $0.24 (TSSOP, 0.4 mV Vos, best specs); TI LMV324IDR $0.31 (output-only rail-to-rail, nA bias) | Use TLV9004 if offset matters more than the last $0.70. MCP6004 is the safe fallback. |
| DAC (10 channels) | 5x MCP4822-E/SN, $1.15/ch | DAC60508 octal $10.48 (WQFN); DAC7568 / DAC7578 octal $12–13; AD5328 $23+; MCP4812 10-bit $4.39 | MCP4822 stays the cheapest per channel. The only big saving left is architectural: send THROTTLE to U3 digitally, and use DAC_SPARE as VSET (that would drop the TL431, the buffer and the VSET pot). Then U8 could become an MCP4801. |
| 5 V supply | TPS54202 $0.18 | AP63205WU-7 $0.42 (fixed 5 V, 1.1 MHz, no FB resistors); TPS54302 $0.26 (forced-PWM, 3 A); Pololu module $6 | The TPS54302 is a pin-compatible swap if Eco-mode ripple shows up on the analog rails. |
| MCU (2x) | ATtiny1614-SSNR $1.99 | One ATtiny3216 / AVR32DA28 to merge the shim and the mixer | That saves about $2 and a UPDI header, but it is a firmware merge. |
| IMU | GY-521 module on J2 | MPU-6050 is EOL; an on-board ICM-42688-P or BMI270 needs a 3.3 V LDO | Kept the module. Revisit if the board goes fully on-board. |
| Analog switch | 74HC4066D Nexperia $0.51 | JSMSEMI 74HC4066D $0.16 (C49364386) | It is a fine swap if you want to save $0.35. |

## 5. Cost summary (one board, LCSC qty-1)

- **SMT BOM** (everything with an LCSC field, 172 placements): **about $31.70**. Of that, $12.69 is digital pots and $11.48 is DACs.
- **The same functions in the THT design:** op-amps $9.23 + DACs $27.44 + trimpots $10.06 + Pololu about $6 + CD4066 $0.93 + ATtinys $3.99 is **about $58** for the major parts alone.
- **JLC assembly** adds a $3 one-time fee per extended part per order: 15 extended parts is about $45/order, spread over the boards in that order.
  To cut this, swap the 47.5k/165k VSET resistors for basic values and use the basic 22 uF.
- **The board itself:** an SMT layout should fit well under the current 250 x 100 mm. PCB area is now the next-biggest cost lever.

## 7. PCB layout

- **Board:** 100 x 100 mm, 2 layers. 100 x 100 mm is JLCPCB's cheapest 2-layer size; PID_Board is 250 x 100 mm.
  There are 224 footprints, all SMD parts on the top side.
- **Placement:**
  - The centre keeps the 20x20 M3 ESC stack holes, the GY-521 header at the CG, the ESC cable slot (with a copper keepout) and the ESC harness header J5.
  - Roll and pitch PID are on the left. The reference, the yaw PID and the control/power parts (U3, buck, ESC dividers) run down the middle. The DAC shim is on the right.
  - Connectors and loop-select jumpers sit on the board edges and have silkscreen pin labels.
- **Rules (JLCPCB standard, no extra cost):**

  | Rule | Value |
  |---|---|
  | Signal track | 0.2 mm |
  | Clearance | 0.15 mm (6 mil) |
  | Power class (+5V, VBAT, VIN, SW) | 0.3–0.4 mm |
  | Vias | 0.6 / 0.3 mm |
  | Copper to board edge | 0.3 mm |

- **Ground:** GND is solid pours on both layers, about 250 stitching vias on a 4 mm grid, plus vias that tie every pour fragment and IC ground pin in.
  GND was not autorouted.
- **Totals:** 2,326 track segments, 454 vias (most of them stitching), about 5.7 m of track.
- **How it was routed:**
  1. FreeRouting 2.1 routed the signals.
  2. Its session writer drops some wires and vias, which shows up as single-point paths in the `.ses` file.
     The missing connections were closed with a small two-layer grid router, `scripts/gaprouter.py`, which can rip up and re-route neighbouring tracks.
  3. Ground was finished by stitching.
- **Worth a manual pass in KiCad's interactive router before ordering.** The grid router's tracks are staircased 45° paths. Several are long:
  - **Op-amp inverting nodes:** U202B− (summer, about 54 mm, 3 vias) and the yaw integrator −input (D402-A, about 18 mm). These are high-impedance nodes, so long traces pick up noise.
  - **Other long nets:** PITCH_ANGLE, VGND_A, ROLL_ANGLE_CMD, Net-(U202C−). Each is over 40 mm.

  Shortening the op-amp inverting nodes is the highest-value cleanup.
- **Silkscreen:** a few references with no room are hidden from the silkscreen. They still appear on the fab layer, as do the SDA/XDA pin labels on J2 and the PITCH label on J1.
- **Not done yet:** Gerbers, drill files, and the JLCPCB BOM and CPL files.

## 8. Regenerating

The schematic is written by `scripts/gen_smt.py` (`python -I gen_smt.py <PID_Board_SMT dir> <expected_nets.json>`).
It reads two flattened symbols (ATtiny1614, MCP4822) from `../PID_Board/PID_Board.kicad_sch`.
Editing by hand in KiCad is fine too. Regenerating overwrites those edits.
