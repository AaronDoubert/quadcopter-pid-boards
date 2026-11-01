# PID_Board rev B: simulation specification (digital twin)

Source of truth: `PID_Board/PID_Board.kicad_pro` rev B (rev B.1, 2026-10-26), read through the
KiCad MCP netlist export (228 components, 159 nets) and the schematic text notes, on 2026-10-31.
Parameters: [`pid_board_params.m`](pid_board_params.m) (one struct, `pcb`).

Provenance tags used below:
- **[sch]**: from the schematic netlist or a schematic text note.
- **[doc]**: from a document or firmware sketch:
  - Wiring guide: `C:\Users\doubi\Downloads\Analog PID Wiring Guide — Reference through Rate Angle Loops.md` (2026-10-19)
  - Shim sketch: `C:\Users\doubi\Downloads\mpu6050_5ch_mcp4822_138\mpu6050_5ch_mcp4822_138.ino`, a breadboard predecessor written for a different decoder wiring, **not** the PCB firmware
  - `sim/roll_params.m` (which cites the briefing)
  - Datasheets
- **[assumed]**: no source. Each one is listed under open questions.

Not found anywhere on disk: the "Analog PID Simulation Briefing" (only cited in `sim/roll_params.m`), and
**any firmware for the PCB** (neither the shim U1 with CRSF/ARM/I_HOLD, nor the control_io U3 mixer).
Other related files: `Downloads\mpu6050_mcp4822_feed.ino` (single-DAC predecessor, 200 Hz) and
`Downloads\mpu6050_diag_readout\mpu6050_diag_readout.ino` (IMU readout only).

---

## 1. Architecture

```
            +---------------- shim U1 (ATtiny1614, root sheet) ----------------+
 MPU-6050 --I2C--> gyro p,q,r + accel -> attitude estimate -> DAC codes       |
 ELRS ----CRSF---> sticks -> ROLL/PITCH_ANGLE_CMD, YAW_RATE_CMD, THROTTLE     |
                   ARM (PA2), I_HOLD (PA7)                                    |
            +--SPI--> 74HC138 -> 5x MCP4822 -> 10x (1k / 1uF) ------------------+
                                     |
   ROLL_ANGLE, ROLL_ANGLE_CMD -> [U202B angle diff] -> [U202C -Go] -> ROLL_RATE_SP -JP201-+
   ROLL_RATE  -------------------------------------------------------------------JP202-+-> roll rate cell U201A-D + U202A -> U_ROLL
   PITCH_*   (same, U301/U302)                                                         -> U_PITCH
   YAW_RATE_CMD -JP401-, YAW_RATE -JP402-> yaw rate cell U401A-D + U402A               -> U_YAW
                                     |
   U_ROLL, U_PITCH, U_YAW, THROTTLE, VGND_A -> U3 ADC -> quad-X mixer -> 4x ESC pulse (J5)
```

The five loops are: roll angle (outer) -> roll rate (inner), pitch angle -> pitch rate, and yaw rate.
All analog signals are deviations from VG = 2.048 V. THROTTLE never enters the analog core. It goes
from the DAC straight to U3. The board does not measure VBAT. [sch]

## 2. Supply, op-amps, reference / virtual ground

| Item | Value | Tag |
|---|---|---|
| +5V | U10 fixed-5 V mini buck (5.5-30 V in) fed from VBAT via D1 1N5817; C25 47 uF/25 V, C26 10 uF | sch |
| Op-amps | MCP6004 (drawn with the LM2902 symbol, same pinout), +5V/GND single supply | sch |
| Output swing | ~25 mV from each rail -> **-2.02 V ... +2.93 V around VG (asymmetric)** | doc: datasheet |
| GBW / SR / Vos | 1 MHz / 0.6 V/us / +-4.5 mV max | doc: datasheet |

**Reference (sheet PID_Reference)** [sch]
- R101 1k from +5V feeds U102 TL431ACLPG (REF tied to K), so the node sits at 2.495 V (+-1 %). I_KA ≈ 2.5 mA.
- U101A follower -> VREF_BUF (TP101). It drives RV101 10k (VREF_BUF to GND); the wiper is VSET (TP102).
  The RV101 label says "wiper-GND 8.1k", sized from the measured VREF_BUF of 2.53 V [doc]. VSET must be trimmed
  so that DAC code 2048 minus VGND_A is under 1 mV (schematic note).
- U101B/C/D: three followers of VSET, each with a 100 Ω series resistor (R102/R103/R104) and a 470 nF film cap
  to GND (C101/C102/C103). They give VGND_A, VGND_B and VGND_C, with a 3.4 kHz corner.
- Rail users:
  - **VGND_A**: + inputs of U201B/C/D, U202A/C, U301B/C/D, U302A/C, and U3 PA7 (the ADC reads it).
  - **VGND_B**: diff-amp reference legs R204, R218, R304, R318 and R404, plus every yaw stage (U401 + inputs, U402A).
  - **VGND_C**: bench plants U202D/U302D and the U402B-D spare followers.
- Fidelity option: VGND_B carries the current of the five 20-21 kΩ reference legs. Its value is
  VSET + 100 Ω x Σ(V_leg - VG)/20k (worst case 0.5 mA, or 50 mV). The whole yaw cell is referenced to it.
  For a first model, treat all three rails as ideal VG (`pcb.ref.ideal = 1`).

## 3. Shim (root sheet) and DAC path

**Pins, U1 ATtiny1614** [sch]

| Pin | Function |
|---|---|
| PA1 / PA3 | SPI MOSI / SCK |
| PA4 / PA5 / PA6 | 74HC138 A0 / A1 / A2 |
| PA2 | ARM (to U3 PB3, R24 10k pulldown) |
| PA7 | I_HOLD (to U9 control pins 13/5/6, R25 10k pulldown) |
| PB0 / PB1 | SCL / SDA. 2N7000 level shifters Q1/Q2, gate bias R2/R4 ≈ 3.4 V, 4.7k pull-ups R1/R3 |
| PB3 | CRSF RX (USART0), through Q3. R23 2.2k pull-up |
| PB2 | Not connected (USART0 TX) |

- **GY-521** (J2): only VCC/GND/SCL/SDA are wired. AD0 floats (module pulldown, so 0x68). INT is not connected,
  so the firmware must poll. [sch]
- **ELRS** (J6): 5V, GND, receiver TX. The receiver RX (pin 4) is not connected, so there is no telemetry. [sch]
- **74HC138 U2**: E3 (pin 6) is tied to +5V and E1/E2 to GND, so **the decoder is always enabled**. Some DAC
  CS line is always low unless the address is parked on Y5-Y7, which are unconnected. [sch]

**MCP4822 U4-U8** [sch]
- LDAC is tied to GND, so each output updates on its own CS rising edge.
- 12 bits at gain 2x: 1 mV/LSB, 0-4.095 V absolute, -2.048 ... +2.047 V around VG. [doc: sketch/guide]

| DAC (decoder output) | VA | VB |
|---|---|---|
| U4 (Y0) | ROLL_ANGLE (R5/C15) | ROLL_RATE (R6/C16) |
| U5 (Y1) | PITCH_ANGLE (R7/C17) | PITCH_RATE (R8/C18) |
| U6 (Y2) | YAW_RATE (R9/C19) | YAW_RATE_CMD (R10/C20) |
| U7 (Y3) | ROLL_ANGLE_CMD (R11/C21) | PITCH_ANGLE_CMD (R12/C22) |
| U8 (Y4) | THROTTLE (R13/C23, also U3 PA6 and J1-4) | DAC_SPARE (R14/C24, J1-5 only) |

**DAC RC filters**: 1k in series, 1 uF (ceramic disc) to **GND**. The capacitor node also feeds a 10k diff-amp
input, so the exact filter depends on the load [sch, derived]:
- Into an inverting leg (gyro rate, angle command): Vd -> 0.909·Vd/(1 + s·0.909 ms), so 175 Hz.
- Into a non-inverting leg (angle meas, yaw cmd; 10k+10k to VG): 0.909·Vd/(1 + s·0.952 ms), so 167 Hz.
- THROTTLE (ADC load only): 1/(1 + s·1.0 ms), so 159 Hz.

`roll_params.m` models this as a 159 Hz RC followed by a 0.909 gain. The DC gains are the same; the poles differ by 5-10 %.

**Scales** [doc: guide, firmware not written]
- Kω = 6 mV per deg/s (±250 deg/s = ±1.5 V; DAC clips at ±341 deg/s).
- Kθ = 25 mV per deg (±60° = ±1.5 V; DAC clips at ±81.9°).
- Setpoints use the same scales.
- THROTTLE scale [assumed]: 0-4.095 V absolute for 0-100 %.

**Firmware and timing**

| Item | Value | Tag |
|---|---|---|
| Frame | 1 kHz: read 14 bytes from the MPU, estimate attitude, write 10 DAC channels (model as ZOH Ts = 1 ms) | assumed: briefing via roll_params |
| | The breadboard sketch runs at **200 Hz** (SAMPLE_MS 5) | doc |
| I2C / SPI | 400 kHz / 8 MHz requested (5 MHz real at 20 MHz). 10 writes take ≈ 50-100 us | doc / assumed |
| MPU-6050 | FS ±250 deg/s, ±2 g; startup gyro bias calibration (5 s still) | doc: sketch |
| | DLPF 98 Hz (DLPF_CFG = 2) modelled as a 2.8 ms delay | assumed: briefing |
| | The sketch never writes CONFIG, so DLPF_CFG = 0 (256 Hz, 0.98 ms delay) | doc |
| Angle estimate | Complementary filter, α ≈ 0.98 per frame. Neither the sketch nor any doc implements one (the sketch sends raw accel) | assumed |
| CRSF | 420 kbaud on PB3 | sch note |
| | Channels 172..992..1811 = 988..1500..2012 us, about 250 Hz packet rate | assumed |
| Sticks | Roll/pitch to angle command, ±60° full stick (±1.5 V); yaw to ±250 deg/s (±1.5 V); throttle to THROTTLE | assumed |
| DAC chip select | PCB firmware must write PA4-6 atomically (VPORTA) and park on Y5-Y7 between transfers. The sketch's DEC_EN gating on PA7 does not exist on the PCB (PA7 is I_HOLD) | sch |
| ARM / I_HOLD | Disarmed: all ESC outputs at 1000 us, I_HOLD high | sch note |
| | ARM from a CRSF switch channel | assumed |
| | I_HOLD high also while throttle is low | assumed |
| | CRSF failsafe disarms after about 250 ms | assumed |
| Boot | R24 holds ARM low (disarmed) and R25 holds I_HOLD **low, so the integrators are free** until U1 runs | sch |

## 4. Rate cell (roll shown; pitch = refdes +100)

Sign rule: every P, I and D path has two inversions (stage + summer), so U rises when SP > FB. [sch note, guide]

| Stage | Op-amp | Parts | Transfer (all voltages re VG) |
|---|---|---|---|
| 1 Rate diff amp | U201A (+ ref VGND_B) | R201 10k (FB_SEL to -), R202 10k (fb), R203 10k (SP_SEL to +), R204 10k (+ to VGND_B) | E_RR = g_sp·SP − g_fb·FB. **g_sp = 0.955** (SP from the U202C output, 0 Ω). **g_fb = 0.909** (gyro through the 1k RC). TP201 |
| 2 P | U201B | R205 47k in; feedback R206 4.7k + RV201 (100k) | P_OUT = −Kp·E, **Kp = (4.7k + RV_P)/47k**, range 0.10-2.23, 1.16 at 50k. TP202 |
| 3 Leaky I | U201C | R207 33k + RV202 (100k) in; C201 1 uF film ∥ R208 1 MΩ ∥ clamp; U9A across C201 | I_OUT/E = −Ki/(s + ωL). **Ki = 1/((33k + RV_I)·1 uF)**, range 7.5-30 /s, 10 /s at 67k. ωL = 1/(1 MΩ·1 uF) = 1 rad/s (0.16 Hz); DC gain = 1M/(33k + RV_I). Clamp: D201+D202 and D203+D204, two 1N4148 in series each way across C201, so \|I_OUT\| ≤ 2·Vf ≈ 0.6-1.0 V (model 0.75 V). TP203 |
| 4 Band-limited D | U201D | R209 27k + C202 100 nF film in series; feedback R210 47k + RV203 (100k) ∥ C203 10 nF C0G | D_OUT/E = −s·Rd·Cd / ((1 + s·Rs·Cd)(1 + s·Rd·Cf)), **Kd = (47k + RV_D)·100 nF**, range 4.7-14.7 ms, 9.7 ms at 50k. f1 = 1/(2π·27k·100n) = 59 Hz (fixed). f2 = 1/(2π·Rd·10n) = 108-339 Hz (164 Hz nominal). HF gain = Rd/Rs ≈ 3.6. Acts on the error, so a setpoint step kicks it. TP204 |
| 5 Summer | U202A | R211 (P), R212 (I), R213 (D), R214 fb, all 10k | U_ROLL = −(P + I + D). Pull R211/R212/R213 to disable a term. TP205, J1-1, U3 PA1 |
| 6 Angle diff amp (swapped) | U202B (+ ref VGND_B) | R215 10k (ANGLE_CMD to -), R216 10k fb, R217 10k (ANGLE to +), R218 10k to VGND_B | E_A = 0.909·(θ_meas − θ_cmd). Both legs see the 1k DAC source, so both gains are 0.909. TP206 |
| 7 Angle P | U202C | R219 20k in; feedback R220 10k + RV204 (50k) | ROLL_RATE_SP = −Go·E_A, **Go = (10k + RV_A)/20k**, range 0.5-3.0, 1.75 at 25k. TP207 |
| 8 Bench plant | U202D (+ VGND_C) | R221 22k from U_ROLL; C204 1 uF film | PLANT_OUT = −U/(s·22 ms). No leak and no reset, so it rails in flight (harmless; only loads U_ROLL with 22k). TP208 |

The whole cell with clamps, rails and the CD4066 ignored:

U − VG = [Kp + Ki/(s + ωL) + Kd·s/((1 + s/ω1)(1 + s/ω2))]·(g_sp·SP − g_fb·FB)

Cascade: SP_rate = Go·0.909·(θcmd − θ). The outer gain is Ka = Go·0.909·Kθ/Kω·(g_sp/g_fb) ≈ 7.0 deg/s per deg at Go = 1.75 (outer bandwidth ≈ 1.1 Hz).

**Pitch** (sheet PID_Pitch): identical values; refdes U301/U302, R301-R321, C301-C304, D301-D304, RV301-RV304,
JP301/JP302, TP301-TP310. Sources: PITCH_RATE, PITCH_ANGLE, PITCH_ANGLE_CMD. Output U_PITCH goes to TP305, J1-2 and U3 PA2. [sch]

**Yaw** (sheet PID_Yaw; rate loop only, every stage referenced to VGND_B) [sch]:
- U401A diff amp R401-R404 10k. The SP is YAW_RATE_CMD **straight from a DAC** (1k source), so **g_sp = g_fb = 0.909** (balanced, unlike roll/pitch).
- U401B P: R405 47k, R406 4.7k + RV401. Same Kp.
- U401C I: **R407 15k** + RV402 (100k, "set mid" = 50k), C401, R408 1M, D401-D404. **Ki = 1/((15k + RV_I)·1 uF)**, range 8.7-66.7 /s, 15.4 /s at mid.
- U401D D: R409 27k, C402 100 nF, R410 47k + RV403, C403 10 nF. Same Kd. **R413 (D into summer) is DNP**, so D_OUT (TP404) exists but U_YAW = −(P + I).
- U402A summer: R411, R412, R414. U402B/C/D are spare followers on VGND_C.
- No angle loop and no bench plant. TPs: TP401 E_YR, TP402-TP405, TP406 EXT_SP, TP407 EXT_FB.

## 5. Jumpers, test points, integrator reset

- **JPx01 LOOP_SP**: 1-2 selects the rate setpoint (ROLL/PITCH_RATE_SP from the angle loop; YAW_RATE_CMD for yaw).
  2-3 selects EXT_SP (TPx09 / TP406). [sch]
- **JPx02 LOOP_FB**: 1-2 selects the gyro DAC (ROLL/PITCH/YAW_RATE). 2-3 selects EXT_FB (TPx10 / TP407). [sch]
- **Bench mode (roll/pitch)**, from the schematic note [sch]:
  - Wire PLANT_OUT to EXT_SP (LOOP_SP 2-3) and the AD3 W1 generator to EXT_FB (LOOP_FB 2-3).
  - The plant inverts, so the swapped legs restore negative feedback: E = 0.955·PLANT_OUT − 1.0·W1 (no 1k RC on W1).
  - In the steady state, PLANT_OUT ≈ −W1·1.05 relative to VG.
  - The connection is a flying lead. There is no on-board PLANT_OUT to EXT_SP trace.
- **Integrator reset U9 CD4066** [sch]:
  - Switches: A (pins 1/2) across roll C201, B (3/4) across pitch C301, C (8/9) across yaw C401. Controls 13/5/6 = I_HOLD. Switch D is spare (control = GND).
  - I_HOLD high discharges CI through Ron (≈ 470 Ω typ, ~1 kΩ max at 5 V), so τ ≈ 0.5-1 ms and I_OUT goes to VG.
- **J1 Axis_Mon (scope header)**: 1 U_ROLL, 2 U_PITCH, 3 U_YAW, 4 THROTTLE, 5 DAC_SPARE, 6 VGND_A, 7 GND. [sch]
- **Other test points**: TP101 VREF_BUF, TP102 VSET, TP103-105 VGND_A/B/C.

## 6. control_io (U3 ATtiny1614)

- **ADC inputs**: PA1 U_ROLL, PA2 U_PITCH, PA3 U_YAW, PA6 THROTTLE, PA7 VGND_A. The op-amp outputs connect
  **directly**, with no divider or filter. The firmware forms (U − VGND_A). [sch]
- **ADC**: 10 bits, VDD reference, LSB 4.88 mV [assumed]. A 4.3 V internal reference would clip U above +2.25 V.
  The result is ratiometric to the 5 V buck, so a VDD error becomes a gain error.
- **Mixer**: quad-X, Betaflight motor order [assumed]: M1 rear-right (PB0), M2 front-right (PB1), M3 rear-left (PB2),
  M4 front-left (PA4).
  - M_i = thr_us + K·[roll, pitch, yaw]·mix_i, with mix rows RR [−1 +1 −1], FR [−1 −1 +1], RL [+1 +1 +1], FL [+1 −1 −1].
  - K = 200 us/V is a placeholder [assumed].
  - Sign conventions must match the gyro axis mapping in the shim [open].
- **ESC protocol**: the schematic note says "PWM / OneShot / DShot". Firmware note: idle/arm pulse 800-1050 us,
  run 1050-2000 us, 1000 us while disarmed. Model it as standard PWM, 1000-2000 us. [sch]
  - Frame rate assumed 400 Hz (Ts 2.5 ms), with the mixer computed once per frame [assumed].
  - Pin constraint [sch]: PB0-PB2 are TCA0 WO0-2, but PA4 is only WO4 in 8-bit split mode. All four outputs
    need split mode (about 10 us resolution) or software-timed pulses.
- **ESC drive**: 1k series and 2.2k to GND, so the high level at the ESC is 3.44 V. J5: 1 BAT−, 2 BAT+, 3-6 M1-M4, 7-8 not connected (ESC current sense unused). [sch]
- **ARM** input on PB3. While it is low, all outputs are 1000 us. [sch note]

## 7. Simulink interface

| Direction | Signal | Units / notes |
|---|---|---|
| In | true body rates p, q, r | rad/s; the shim model adds bias, noise, DLPF delay, ±250 deg/s saturation, and Kω |
| In | true roll, pitch angles (or the accel vector to run the complementary filter) | rad |
| In | RC sticks roll, pitch, yaw, throttle | CRSF 172-1811 or normalized ±1 / 0-1 |
| In | ARM switch | bool. Drives ARM and I_HOLD through the shim logic |
| In | VBAT | V. Plant/ESC only; the board does not sense it (VDD fixed at 5 V) |
| Param | RV_* per axis, en_*, JP_SP/JP_FB | from `pcb` |
| Out | M1-M4 ESC pulse width | us, 1000-2000, discrete at pcb.ctrl.Ts |

Signals worth logging (all relative to VG):
- Rate cells: E_RR/E_PR/E_YR, P_OUT, I_OUT, D_OUT, U_ROLL/U_PITCH/U_YAW
- Angle loops: E_A, ROLL/PITCH_RATE_SP
- Bench: PLANT_OUT
- DAC outputs after the RC (ROLL_RATE etc.), THROTTLE
- Logic: I_HOLD, ARM
- Saturation flags for the rails and the I clamp

Suggested split:
- Shim: discrete, Ts_shim.
- DAC: quantizer, clip and loaded RC.
- Analog: continuous, 3 cells plus 2 angle stages, with rail saturation after every op-amp.
- control_io: discrete ADC quantizer, mixer and ESC ZOH.

## 8. Inconsistencies (schematic vs `sim/roll_params.m` / `build_roll_model.m` / docs)

1. **Mixer ADC LSB**: `mix_adc_lsb` assumes a 1k/2.2k divider in front of the ADC (7.1 mV at U). The schematic
   puts 1k/2.2k on the **ESC outputs**; the ADC reads U directly, so the LSB is 4.88 mV with a VDD reference.
2. **Bench mode**: the model feeds back +∫U/τ through g_fb = 0.909. On the hardware, PLANT_OUT enters the **+ leg**
   (g = 0.955), the generator enters EXT_FB with gain 1.0 and no RC, and the generator setpoint appears inverted.
   The DC behaviour is equivalent, but the gains and sign differ.
3. **DAC RC**: the model uses 159 Hz followed by a gain. The loaded poles are 175 Hz (inverting legs) and 167 Hz (+ legs). Minor.
4. **Gyro scale**: the guide and model use 6 mV per deg/s. The only shim code (breadboard sketch) maps ±250 deg/s
   to ±2.048 V, which is **8.19 mV per deg/s**. It also streams accel X/Y instead of angles, and uses a 6-channel
   map that is not the PCB map.
5. **Timing**: the model uses Ts_fw = 1 ms and a 98 Hz DLPF (2.8 ms). The sketch uses 200 Hz and the default DLPF
   (CFG 0). Neither the briefing nor PCB firmware was found.
6. **Decoder enable**: the sketch gates CS with DEC_EN on PA7. On the PCB, PA7 is I_HOLD and E3 is hard-wired high.
   The sketch's DAC write routine would glitch other DACs on the PCB.
7. **I_HOLD at boot**: CHANGELOG rev A says the pulldowns "define" the lines at boot. R25 pulls I_HOLD **low**, which
   means the integrators are *not* held during boot. The model's `t_arm` holds the integrator (ideal reset). The
   hardware reset goes through Ron ≈ 0.5-1 kΩ.
8. **Yaw differs from roll**: R407 is 15k, the yaw D term is not summed (R413 DNP), and g_sp = 0.909 (not 0.955).
   The yaw Ki default ("set mid") is 15.4 /s.
9. **VSET**: the label says wiper-GND 8.1k (sized for the measured 2.53 V). With a nominal 2.495 V TL431 that gives
   2.021 V, 27 mV (4.5 deg/s) off DAC midscale. It must be trimmed; the model assumes a perfect 2.048 V.
10. **t_esc = 0** in the model. The schematic note implies standard 1000-2000 us PWM, i.e. a 2.5-20 ms frame plus ZOH.
11. **Diff-amp imbalance** (roll/pitch): g_sp ≠ g_fb gives a 5 % rate-scale error and finite CMRR
    (E = 0.046·V_cm). The model already has this.
12. **Rails**: the model uses −2.0 / +2.9 V and the datasheet gives −2.02 / +2.93 V. That is consistent; note the
    asymmetry, since negative U saturates first.
13. **Clamp**: the guide says 0.6-0.9 V and the model 0.75 V. Two 1N4148 at 10-50 uA give about 0.9-1.0 V. Bench measurement recommended.

## 9. Open questions (cannot be determined from the repo)

- All PCB firmware (shim and control_io):
  - Loop rates, DLPF, gyro FS
  - Kω / Kθ, attitude filter, stick-to-command scaling and max angle/rate
  - THROTTLE voltage scale
  - ARM switch channel and failsafe
  - When I_HOLD is asserted (disarmed only, or also at low throttle)
  - ADC reference
  - Mixer gains (us/V) and saturation/airmode handling
  - ESC protocol and frame rate
  - How M4 on PA4 is timed
- IMU axis/sign mapping: GY-521 orientation on J2 versus the frame axes, and the motor numbering/rotation on the Aero Selfie 4in1.
- Whether VGND_B loading and op-amp offsets (±4.5 mV per stage) matter enough to model.
- Measured trimpot settings after tuning (the schematic values are the design nominal).
- The location of the "Analog PID Simulation Briefing" (2026-10-20).
