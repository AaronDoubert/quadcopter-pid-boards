# quad_params.m: sources, fits and sanity checks

Status key: **M** = measured by a third party on the same or a very similar part, **D** = datasheet or manufacturer value, **C** = computed here from other rows, **E** = engineering estimate (needs measuring).

Nobody has published a thrust-stand test of a 2207 ~1800KV motor with a 5x4.3x3 prop on 4S (searched MiniQuadTestBench, manufacturer tables, Tyto/RCbenchmark). Most 2207 motors in the 1700-1950KV range are 6S motors. So the model is split into a **prop model** (thrust and torque vs RPM, taken from real 4S data with the exact HQ prop) and a **motor/ESC/battery electrical model** (iFlight XING-E 2207 1800KV specs). The electrical model then predicts RPM, and so thrust, for any bus voltage.

## Source list

| Tag | Source |
|---|---|
| S1 | PID_Board/MOUNTING.md and README.md in this repo (frame sketch, board size, ESC/IMU placement) |
| S2 | MiniQuadTestBench, Hypetrain Cricket 2306.6 2550KV full results, HQ v1s 5x4.3x3PC on a ~16 V supply: https://www.miniquadtestbench.com/assets/components/motordata/motorinfo.php?uid=282 |
| S3 | Skystars 2207 1950KV spec and thrust table (6S, GF51433 and T5143S), via racer.lt: https://www.racer.lt/item/skystars-2207-1950kv-4-6s-brushless-motor-for-fpv-racing-drone-parts-34410.html |
| S4 | iFlight XING-E 2207 1800KV spec (KV, 81.5 mOhm, I0 < 1.2 A @10 V, 33.8 g, 12N14P), via racer.lt: https://racer.lt/item/4-pcs-iflight-xing-e-2207-2207-1800kv-3-6s-brushless-motor-for-rc-drone-fpv-racing-24444.html |
| S5 | MiniQuadTestBench Sunnysky E-R2207 1800KV (tested 1870 KV, 35.2 g; 6S only): https://www.miniquadtestbench.com/sunnysky-e-r2207-1800kv.html |
| S6 | Aero Selfie 30A 4in1 ESC listing (only listing found): "PWM, DSHOT, ONESHOT", 6-18 V, 8-bit MCU, 10 A continuous / 30 A 30 s peak per channel: https://www.tilga.com.kw/prod/8541651 |
| S7 | Oscar Liang, ESC protocols (pulse widths / frame times; "roughly 20ms to 50ms" for a 5" motor to change RPM): https://oscarliang.com/esc-firmware-protocols/ |
| S8 | InvenSense PS-MPU-6000A-00 rev 3.1 product specification (SparkFun mirror): https://cdn.sparkfun.com/datasheets/Components/General%20IC/PS-MPU-6000A.pdf |
| S9 | InvenSense RM-MPU-6000A-00 rev 4.0 register map, DLPF_CFG table (register 26): https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf |
| S10 | Faessler, Franchi, Scaramuzza, "Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag...", IEEE RA-L 3(2) 2018, arXiv 1712.02402 (610 g FPV racing quad; dx = 0.544/0.491, dy = 0.386/0.236 s^-1): https://arxiv.org/abs/1712.02402 |
| S11 | Eschmann et al., "Data-Driven System Identification of Quadrotors Subject to Motor Delays", arXiv 2404.07837, Table II (Izz/Ixx ratios of many quads, mean 1.83): https://arxiv.org/abs/2404.07837 |
| S12 | CNHL Black Series 1500 mAh 4S 100C: 183 g incl. lead, 34x35x73.5 mm (retailer listing): https://www.racedayquads.com/products/cnhl-black-series-14-8v-4s-1500mah-100c-lipo-battery-xt60 |
| S13 | drone-zone.de review, MRP 4S 1500 mAh: ~3.2-3.7 mOhm per cell (charger IR at 40 C): https://drone-zone.de/battery-review-mrp-4s-1500-mah-65-130-c (found via search; page returned 404 when fetched later) |
| S14 | uavmodel IR guide: healthy 4S 1300 = 2-5 mOhm/cell, 1500 = 2-4 mOhm/cell: https://blog.uavmodel.com/?p=3506 |
| S15 | Generic LCO/LiPo OCV-SOC table (applied-ee): https://applied-ee.github.io/embedded/docs/power-battery/li-ion-integration/fuel-gauging-and-soc/ |
| S16 | HQProp 5x4.3x3 V1S weight 3.75-3.81 g (retailer listings, e.g. https://www.getfpv.com/hqprop-dp-5x4-3x3v1s-light-orange-propeller-3-blade-2cw-2ccw-bag.html) |
| S17 | sim/roll_params.m in this repo (DLPF 2.8 ms, 1 ms firmware loop) |

## Parameter table

| Field | Value | Unit | Source | Status |
|---|---|---|---|---|
| mass.total | 0.650 | kg | user (AUW with 1500 pack) | E, weigh it |
| mass.total_1300 | 0.622 | kg | 0.650 - 0.183 + ~0.155 | E |
| geom.arm_lat / arm_lon | 0.087 / 0.0745 | m | S1 "motors assumed at (+-87, +-74.5) mm" | E (from sketch) |
| geom.arm, diag | 0.1145 / 0.229 | m | C | C |
| geom.z_prop | -0.030 | m | motor 20 mm tall on 5 mm arm | E |
| geom.cg_frame | [2.3, 0, -1.8] mm | m | mass breakdown below | C |
| geom.r_imu | 15 mm above frame centre | m | S1 (GY-521 over frame centre) | E |
| inertia.J | diag(1.62, 1.87, 3.14)e-3, Ixz 1.8e-5 | kg m^2 | mass breakdown below | C (from E masses) |
| inertia.J_rotor | 5.3e-6 | kg m^2 | bell 1.8e-6 + prop 3.6e-6, see below | E |
| prop.kT | 1.32e-6 | N/(rad/s)^2 | S2 fit | C from M |
| prop.kQ | 1.40e-8 | N m/(rad/s)^2 | S2 fit (+ S3 check) | C from M, +-15 % |
| prop.kT_GF51433 / kQ_GF51433 | 1.45e-6 / 1.60e-8 | | S3 fit 18-26k rpm, kQ scaled | C from M (6S data) |
| prop.mass | 3.8e-3 | kg | S16 | D |
| motor.KV | 1800 | rpm/V | S4 (S5 tested a Sunnysky 1800 at 1870) | D |
| motor.Kt | 5.31e-3 | N m/A | 60/(2 pi KV) | C |
| motor.R_wind | 0.0815 | ohm | S4 | D |
| motor.R (effective) | 0.14 | ohm | R_wind + 0.06 from S2 fit (see below) | E |
| motor.I0 | 1.2 | A | S4 (< 1.2 A at 10 V) | D |
| motor.mass | 33.8e-3 | kg | S4 | D |
| motor.tau_hover | 0.023 | s | J_rotor/(Kt^2/R + 2 kQ w) | C |
| motor.tau | 0.025 | s | C, in the 20-50 ms range of S7 | E |
| motor.I_max_cont_ESC / peak | 10 / 30 | A | S6 | D (listing) |
| batt.cells, capacity | 4, 1.5 Ah (1.3 alt) | | user | given |
| batt.mass | 0.183 (0.155 for 1300) | kg | S12 (1300: no source found) | D / E |
| batt.soc_tab / ocv_tab | 11-point curve 3.00-4.20 V | V/cell | S15 | D (generic) |
| batt.R_cell | 0.006 (0.007 for 1300) | ohm | S13/S14 AC-IR ~3.3 mOhm x ~1.8 for DC sag | E |
| batt.R_wire | 0.004 | ohm | XT60 + 10 cm 14 AWG + pads | E |
| batt.R_total | 0.028 | ohm | C | C |
| batt.I_avionics | 0.3 | A | board + RX | E |
| esc.protocol, pwm range | PWM, 1000-2000 us | us | S6 (PWM/Oneshot/DShot accepted), S7 | D / E (which one the ATtiny sends) |
| esc.update_hz | 490 | Hz | typical; set to the shim's real rate | E |
| esc.frame_by_protocol_us | PWM 2000, OS125 250, OS42 84, MS 25, DShot150/300/600 106.7/53.3/26.7 | us | S7 | D |
| esc.latency | 3.2 | ms | half frame + pulse + 0.2 ms processing | E |
| esc.idle_duty | 0.05 | - | BLHeli_S-like idle | E |
| esc.V_min / V_max | 6 / 18 | V | S6 | D |
| esc firmware | BLHeli_S probably (8-bit MCU), not confirmed | | S6 | E, check with esc-configurator |
| drag.d_lin | [0.40 0.40 0] | 1/s | S10 (0.24-0.54 on a 610 g racing quad) | M on a similar quad |
| drag.CdA | [0.010 0.013 0.035] | m^2 | frontal/plan areas x Cd ~1.1-1.2 | E |
| drag.k_rot | [3.4 2.5 3.4]e-3 | N m s/rad | momentum theory, see below | E, +-100 % |
| imu.gyro.noise_density | 0.005 | deg/s/rtHz | S8 | D |
| imu.gyro.rms_noise_100Hz | 0.05 | deg/s rms | S8 (DLPF_CFG=2) | D |
| imu.gyro.bias_init_max / temp | +-20 / +-20 | deg/s | S8 | D |
| imu.gyro.bias_resid | 0.5 | deg/s | after a boot-time average | E |
| imu.gyro.scale_tol / cross_axis | 3 % / 2 % | | S8 | D |
| imu.gyro.g_sens | 0.1 | deg/s/g | S8 | D |
| imu.gyro.lsb_per_dps | 131 / 65.5 / 32.8 / 16.4 | | S8 | D |
| imu.accel.noise_density | 400 | ug/rtHz | S8 | D |
| imu.accel.bias_init | +-50 mg (x,y), +-80 mg (z) | | S8 | D |
| imu.accel.bias_temp | +-35 mg (x,y), +-60 mg (z), 0-70 C | | S8 | D |
| imu.accel.scale_tol / cross | 3 % / 2 % | | S8 | D |
| imu.dlpf table | gyro BW 256/188/98/42/20/10/5 Hz, delay 0.98/1.9/2.8/4.8/8.3/13.4/18.6 ms; accel 260/184/94/44/21/10/5 Hz, 0/2.0/3.0/4.9/8.5/13.8/19.0 ms | | S9 | D |
| imu.dlpf_cfg_used, Ts | 2, 1 ms | | S17 | given |

## Fits

### kT (thrust vs RPM)
S2 (HQ v1s 5x4.3x3, the actual prop, 4S): T = 45/170/494/893/1309 g at 5869/11004/18326/24588/29990 rpm. Point values of T/w^2 are 1.17, 1.26, 1.32, 1.32 and 1.30 e-6. A least-squares fit on the 25-100 % rows gives **kT = 1.308e-6**, or **1.318e-6** on the 10-26k rpm flight band. Used: **1.32e-6**. The idle point is 11 % low (low Reynolds number). At hover (10.5k rpm) the real thrust is about 4 % below the fit.

S3 (Gemfan 51433, 6S): T/w^2 rises from 1.33e-6 at 18.3k rpm to 1.54e-6 at 30k rpm. LS on 10-26k rpm gives 1.455e-6. The Gemfan makes 10-15 % more thrust than the HQ at the same RPM, so it is kept as `kT_GF51433`.

### kQ (torque vs RPM)
None of the sources measured torque directly, so it is inferred from the electrical data.
* At 100 % throttle the duty is about 1, so phase current = battery current. S2: Q = Kt(I - I0) = 3.745e-3 x (35.69 - 1.5) = 0.128 N m at 3140 rad/s, giving kQ = 1.30e-8 and kQ/kT = 10.0 mm. S3 with GF51433 gives kQ = 1.74e-8 and kQ/kT = 11.2 mm.
* Part throttle, from a power balance P = (E + I R_eff) I with R_eff = 0.105 ohm (next section): kQ = 1.40e-8 at 75 %, 1.55e-8 at 50 % and 2.0e-8 at 25 %. The low-throttle numbers are inflated by ESC and iron losses, which this balance cannot separate out.
* Used: **kQ = 1.40e-8** (kQ/kT = 10.6 mm). Uncertainty is about +-15 %.

### Effective motor resistance
In S2 the 2550KV motor at 100 % reached 29990 rpm at 15.5 V and 35.69 A. Back-EMF is 11.76 V, which leaves 3.74 V across 35.7 A, so R_eff = 0.105 ohm. A 2306 2550KV winding is roughly 40-50 mOhm, so about 0.06 ohm comes from the ESC FETs, wiring and commutation losses. For the 1800KV motor: R = 0.0815 + 0.06 = **0.14 ohm**. The S3 1950KV table implies an even larger drop (0.2 ohm), but its KV is not independently tested, so it was not used.

### Rotor inertia
* Bell: magnets 6 g at r = 11.9 mm, wall 4 g at 13.2 mm, cap 2.5 g as a disk of r = 13.7 mm. Total **1.8e-6** kg m^2.
* Prop: 3.2 g of blade (3.8 g minus hub) as 3 rods from 6 to 63.5 mm, x0.75 for taper. Total **3.6e-6**.
* Rotor total: **5.3e-6** kg m^2.
* Time constant: tau = J_r / (Kt^2/R + 2 kQ w) = 23 ms at hover and 20 ms at full speed. This is consistent with S7 (20-50 ms). ESC latency and current limiting add to it.

### Mass breakdown and inertia (FRD, frame-centre origin)
| Item | Mass (g) | Position (x, y, z) mm | Shape |
|---|---|---|---|
| 4 motors (S4) | 4 x 33.8 | (+-74.5, +-87, -12) | 28x28x20 box |
| 4 props (S16) | 4 x 3.8 | (+-74.5, +-87, -30) | point |
| 4 arms (5" freestyle, est.) | 4 x 14 | rods from r = 30 to 114.5 mm on the diagonals, z = +2 | rod |
| Centre plates, standoffs, hardware (est.) | 64 | (0, 0, -5) | 120x45x25 box |
| Battery 1500 4S (S12), **under** the frame | 183 | (0, 0, +20) | 73.5x35x34 box |
| 4in1 ESC (S6) | 7 | (-57, 0, -8) | 32x32x5 box |
| PID_Board rev B, 90x210 4-layer + THT parts (est.: FR4 ~56 g + Cu ~14 g + parts ~35 g) | 105 | (0, 0, -18) | 210x90x15 plate |
| Board standoffs/screws | 12 | (0, 0, -9) | 130x30x10 box |
| RX, wires, XT60, cap, strap | 25 | (-20, 0, 0) | point |
| Unaccounted remainder (camera/VTX/misc) | 47.6 | (50, 0, -10) | 30 mm cube |
| **Total** | **650** | CG (2.3, 0, -1.8) | |

Result: Ixx = 1.62e-3, Iyy = 1.87e-3, Izz = 3.14e-3 kg m^2. Izz/Ixx = 1.94 and Izz/Iyy = 1.67, which brackets the 1.83 mean of S11. Iyy > Ixx because the 210 mm board runs fore-aft. If the battery is **top-mounted** about 60 mm above the frame, Ixx and Iyy each rise by about 1.2e-3 (roughly +70 %). The 1300 pack changes J by less than 2 %.

### Drag
* Linear rotor drag: S10 identified d = 0.24-0.54 1/s on a 610 g, T/W 4 racing quad with 6" props. Used 0.40 in x and y, which gives 0.26 N/(m/s) for this quad.
* Quadratic: frontal area about 0.008-0.009 m^2 (battery, board stack, motors, arms) with Cd about 1.1. Side area is larger because of the 210 mm board. Plan area is about 0.03 m^2.
* Rotational damping: hover induced velocity v_i = sqrt(T/(2 rho A)) = 7.2 m/s, so dT/dVz = rho A v_i = 0.111 N/(m/s) per rotor. Roll damping is 4 x 0.111 x 0.087^2 = 3.4e-3 and pitch is 2.5e-3 N m s/rad. For yaw, in-plane rotor drag 0.065 N/(m/s) x 0.1145^2 x 4 = 3.4e-3.

### Electrical operating points (Python, battery sag iterated)
Model: Vbus = OCV - 0.028 x (4 I_m + 0.3), steady state (R kQ/Kt) w^2 + w/Kv + R I0 - Vbus = 0.

| Pack OCV/cell | Vbus at full throttle | rpm max | I_motor | I_batt | T_max/motor | T/W |
|---|---|---|---|---|---|---|
| 4.20 (full) | 14.97 V | 22.8k | 16.3 A | 65 A | 7.53 N (768 g) | 4.7 |
| 3.85 (mid) | 13.80 V | 21.2k | 14.2 A | 57 A | 6.51 N (664 g) | 4.1 |
| 3.50 (sagged/low) | 12.62 V | 19.6k | 12.3 A | 50 A | 5.55 N (566 g) | 3.5 |

## Sanity checks
* **Hover:** T = 1.59 N per motor, w = 1099 rad/s (10.5k rpm), I_motor = 4.4 A. Duty is 0.40 on a full pack, 0.42 mid-pack and 0.46 near empty, so PWM is about 1400-1460 us. Total battery current is about 7.4 A and power about 113 W, giving about 10 min hover on 80 % of 1500 mAh. A 6S-class 1800KV motor on 4S hovering near 40 % is plausible. A typical 4S 2400-2600KV build hovers at 25-30 %.
* **T/W:** 4.7 on a full pack and 3.5 when sagged. S10's comparable 610 g quad is quoted at T/W 4.
* **Max angular acceleration**, full pack, two motors at max and two at idle:
  * Roll: about 800 rad/s^2.
  * Pitch: about 600 rad/s^2.
  * Yaw: about 51 rad/s^2.
  * About hover with symmetric differential, roll is about 640 rad/s^2 and yaw about 21 rad/s^2.
  * Yaw authority is about 15x weaker than roll, which is normal for quads.
* **ESC current: important.** The full-throttle phase current (12-16 A per motor) is above the Aero Selfie's 10 A continuous rating, although below its 30 A / 30 s burst. Hover (4.4 A) is fine. Either cap the throttle at about 75-80 % in the mixer, or model `esc.I_limit` = 10 A continuous for long full-throttle segments.
* **Consistency with sim/roll_params.m:** that file uses J = 0.004 kg m^2, L = 0.11 m, m = 0.6 kg and tau_m = 0.04 s. This set gives Ixx = 0.0016, roll arm = 0.087 m (0.1145 diagonal), m = 0.65 kg and tau = 0.025 s. The roll plant is therefore about 2x more responsive than the briefing placeholders.
