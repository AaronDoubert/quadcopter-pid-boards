# Model 2 (fast): Simulink digital twin of PID_Board rev B

A behavioural model of the analog PID flight-controller board. It plugs into the Model 1 plant
(`sim/quad_plant/quad_sim.slx`) as the controller and also runs on its own as a bench model.
Hardware facts come from [`pid_board_spec.md`](pid_board_spec.md) and the parameters from
[`pid_board_params.m`](pid_board_params.m) (struct `pcb`). The analog port contract shared with the
Simscape circuit variant is in [`ANALOG_INTERFACE.md`](ANALOG_INTERFACE.md).

## Files (all in `sim/pid_board/`)

| File | What it is |
|---|---|
| `pid_board_build.m` | Builds all the `.slx` files below from source. Every block parameter is an expression of `pcb` evaluated through subsystem masks. |
| `pid_analog_fast.slx` | Analog section. Referenced model, continuous. 15 inports and 6 outports, all absolute node voltages. |
| `pid_board.slx` | Whole board. Referenced model with the `ctrl_in` (CtrlIn) → `esc_us` [4x1] contract. Holds the shim, the analog Variant Subsystem and control_io. |
| `pid_board_bench.slx`, `pid_board_bench_inputs.m` | Standalone bench harness, with synthetic or recorded IMU and stick data and a LOOP/GYRO jumper bench mode |
| `pid_board_tb_analog.slx` | Analog-only test bench. Input is `pbsim.an_in = [t, 15 voltages]`. |
| `pid_board_cl.slx` | Closed-loop rig: the board plus a rigid-body attitude plant (J and geometry from `quad_params.m`) |
| `pid_board_validate.m` | Validation parts (a)–(f) |
| `gains_from_pots.m`, `pot_from_gains.m` | Convert between pots (Ω or 3296W turns) and gains, in board units or Model 1 ideal-controller units |
| `ANALOG_INTERFACE.md` | Port contract for `pid_analog_fast` / `pid_analog_circuit` |

## How to run

```matlab
addpath('C:\Users\doubi\Quadcopter Flight controller\sim\pid_board', 'C:\Users\doubi\Quadcopter Flight controller\sim\common')
pid_board_params; quad_ctrl_buses;           % pcb, CtrlIn (models also do this on load if missing)
pid_board_build                              % rebuild every model (only if the sources changed)
res = pid_board_validate;                    % all checks, ~1 min
pid_board_bench_inputs('wobble'); out = sim('pid_board_bench');   % bench run, see logsout / SDI
gains_from_pots                              % table of the current pots as gains
pot_from_gains(ideal.roll, 'roll', 'units', 'ideal')               % Model 1 gains -> pots
```

**In Model 1:** `quad_set_controller('install_board')` (done, see below), then
`quad_set_controller('board_fast')` and `run_quad_scenarios`.

Bench scenarios for `pid_board_bench_inputs`:
- `still`, `wobble`, `sticks`, `disarmed`.
- `record` with a struct S (S.t plus any of the CtrlIn fields, N×k).
- `bench_roll`: sets `pcb.roll.JP_SP = JP_FB = 2`, the PLANT_OUT flying lead (`pcb.bench.ext_sp_src(1) = 1`) and AD3 W1 into EXT_FB (`ext_fb_src(1) = 2`, a step or sine set by `pcb.bench.w1_*`). Compare `roll_PLANT` (TP208), `roll_U` (TP205), `roll_E` (TP201) etc. with your scope.

Logged signals:
- Board level: `dac_V` (10 DAC pins), `I_HOLD`, `ARM`, `att_est_deg`, `U_ROLL/U_PITCH/U_YAW/VGND_A`, `PLANT_OUT_*`, `THROTTLE`, `adc_u_V`, `W1`, `esc_us`.
- Test points, in volts re VG: `<axis>_E_A`, `_RATE_SP`, `_E`, `_P`, `_I`, `_D`, `_U`, `_PLANT`.

## Architecture

```
ctrl_in (CtrlIn, 1 kHz) ─► Shim U1 (Ts = pcb.shim.Ts = 1 ms)
   gyro: deg/s, ZOH, DLPF delay round(t_dlpf/Ts) frames, optional noise, ±250 deg/s, 1/131 LSB
   accel: g, same delay, ±2 g, 1/16384 LSB
   complementary filter (pcb.shim.cf_alpha) or euler_true (pcb.shim.angle_src = 0)
   CRSF: sticks quantised to 172..992..1811 at 4 ms; ARM over CRSF; I_HOLD = !ARM | thr < 5 %
   scaling Kw 6 mV/(deg/s), Kθ 25 mV/deg, ±60°, ±250 deg/s → MCP4822 12-bit (1 mV, 0..4.095 V)
 ─► Rate Transition (DAC pins hold) ─► Analog section: Variant Subsystem
      board_variant == 1: pid_analog_fast    board_variant == 2: [pcb.circuit_enable] pid_analog_circuit
      roll/pitch: loaded RC (175/167 Hz) → U202B angle diff → U202C −Go → JP201 → U201A diff
                  → P / leaky I (clamp ±0.75 V, CD4066 Ron reset) / band-limited D → U202A summer
                  rails −2.023/+2.927 V after every op-amp; U202D bench integrator → PLANT_OUT
      yaw: same rate cell, R407 15k, D built but not summed
 ─► control_io U3 (Ts = pcb.ctrl.Ts = 2.5 ms): 10-bit ADC (VDD ref) of U_*, THROTTLE (1k/1uF), VGND_A;
    u = V − VGND_A; quad-X mixer pcb.ctrl.mix (Betaflight order); clip 1050..2000 µs when armed,
    1000 µs when disarmed; 1 µs resolution ─► esc_us [M1 RR, M2 FR, M3 RL, M4 FL]
Bench test points: W1 generator, PLANT_OUT → EXT_SP flying lead (0.5 ms lag) per pcb.bench.*
```

Motor numbering: control_io drives M1 RR (PB0), M2 FR (PB1), M3 RL (PB2) and M4 FL (PA4). This is the Betaflight order used by the contract, so no remapping is needed.

**Sign conventions (shim, `pcb.shim.axis_sign = [1 -1 1]`):** the board's pitch axis is **nose-down positive** to match the Betaflight mixer column. So `PITCH_RATE = −Kw·q` and `PITCH_ANGLE = −Kθ·θ`. A +1 pitch stick (forward) commands θ < 0. Checked in Model 1: a +0.25 pitch stick settles at θ = −15.6° (CF) / −15.0° (euler_true).

### Variant switching

- `pid_board` has a **model argument `board_variant`**: 1 = fast (default), 2 = circuit. Model 1's `quad_set_controller('install_board')` sets it per instance. The bench and closed-loop harnesses pass `board_variant = pcb.variant`.
- Model arguments need Variant activation time **startup**, so both choices compile. The circuit Model block sits behind an inner `update diagram` variant on **`pcb.circuit_enable`** (default 1).
  - While `pid_analog_circuit.slx` is missing, unsaved or not Fixed-step compatible, set `pcb.circuit_enable = 0`. Otherwise **every** `pid_board` sim fails to compile.
  - With `circuit_enable = 0`, a `board_variant = 2` instance has a dead analog section (all outputs 0 V).

### Solver

- Every model is **Fixed-step ode4 with `FixedStep = pcb.sim_dt` (0.5 ms)**. This must equal Model 1's `qsim.dt`: a referenced model with continuous states must use its parent's step, and `auto` fails.
- Discrete rates are 1 ms (shim), 4 ms (CRSF) and 2.5 ms (control_io). All are multiples of 0.5 ms.
- The analog section is not stiff for ode4 at 0.5 ms. Its fastest pole is the CD4066 reset at 2130 rad/s, so h·λ = 1.1, below the stability limit of 2.8.
- `ctrl_in` has sample time 1e-3 (the Model 1 bus rate). A continuous source needs a 1 kHz ZOH in front of it (the rigs have one).

## Assumptions (all are `pcb` fields; confirm them against the real firmware)

| Item | Model value | Field |
|---|---|---|
| Shim frame | 1 kHz, all 10 DACs updated together (skew ignored) | `pcb.shim.Ts` |
| DLPF | 2.8 ms → 3 frames of delay | `pcb.shim.t_dlpf` |
| Gyro scale | **6 mV/(deg/s)** (guide). The breadboard sketch used 8.19. | `pcb.scale.Kw` |
| Attitude filter | complementary filter, **α = 0.99995 per frame (τ ≈ 20 s)**, seeded from the accelerometer at boot. Changed from the spec's ~0.98; see the results below. | `pcb.shim.cf_alpha`, `angle_src` |
| Sticks | ±60°, ±250 deg/s, throttle 0–4.095 V | `pcb.shim.angle_max`, `yaw_rate_max`, `pcb.scale.thr_*` |
| ARM / I_HOLD | ARM from the CRSF switch; I_HOLD high when disarmed or throttle < 5 %. No failsafe input. | `pcb.shim.ihold_low_thr`, `thr_low` |
| ADC | 10 bit, VDD reference, floor | `pcb.ctrl.adc_*` |
| Mixer | throttle 1000 + thr·1000 µs; K_rpy µs/V (200 placeholder; first_flight_gains uses 150/200/300); clip 1050–2000 when armed | `pcb.ctrl.*` |
| ESC | standard PWM; command held per 2.5 ms frame. No extra latency (Model 1 adds ESC latency). | `pcb.ctrl.Ts`, `delay_frames` |
| Rails / reference | ideal VG = 2.048 V (`pcb.ref.ideal = 0` gives the untrimmed 2.021 V VSET); no op-amp offsets or GBW | `pcb.ref.ideal` |

## Validation (`pid_board_validate`, 0 failures)

**(b) Stage gains vs the spec formulas**, linearised from `pid_board_tb_analog`, nominal pots:
- All 42 checks match within 0.01 %. These cover all 3 axes:
  - E/RATE = −0.909, with the RC pole at 175 Hz
  - Kp = 1.164
  - |I/E| = 8.47 / 1.57 / 0.159 at 0.1 / 1 / 10 Hz (yaw Ki = 15.4)
  - |D/E| = 0.600 / 2.39 / 2.64 at 10 / 59 / 100 Hz
  - Go = 1.75
  - E_A/ANGLE_CMD = −0.909 (175 Hz) and E_A/ANGLE = +0.909 (167 Hz)
  - yaw SP leg = 0.909 (167 Hz)
  - yaw U/E with no D
- These are the same values as the legacy guide checks.

**(c) Rails and clamp:**
- P saturates at −2.023 V and I clamps at ±0.750 V.
- U_ROLL = 2.773 V, max 2.927 V (the D kick hits the rail).
- U_PITCH sits on the low rail: −2.023 V, or 25 mV absolute.

**(d) I_HOLD:**
- I falls from the clamp to 10 % in 1.5 ms. The exact value is 1.08 ms; the model resolves it at the 0.5 ms solver step.
- The held level is −8.5 mV, matching Ron/R_in·E.
- I recharges to the clamp after release.

**(e) ARM:**
- Disarmed (wobble, 50 % throttle): every ESC output is exactly 1000 µs and I_HOLD is high.
- Armed: outputs stay within 1050–2000 µs.

**(f) Closed loop with `pid_board_cl`** (rigid body, quad_params J, nominal pots, K_rpy 200):
- From 10°/−8° initial attitude: level within 0.25° by 1.9 s.
- Roll stick 0.25 → 14.8°; pitch −0.25 → +14.5° nose up; yaw 0.2 → 53 deg/s.
- Rate rms at the end: 0.27 deg/s.
- Rate-loop plant gain is 106/79/6.7 per s (the legacy design assumed 45.5 /s).

**(a) Legacy `roll_axis` model** (built into tempdir as `pid_board_legacy_roll`):
- A1, a 10° angle-command step with no airframe: E_RR 0.380/0.380 V, I −0.750/−0.750 V, final U 1.1920/1.1918 V. The largest transient difference is 86 mV, in the D kick (159 Hz RC vs the loaded 175 Hz RC, ode45 vs ode4).
- A2, the bench integrator loop with a 100 mV step: rise 40.7 → 37.3 ms, overshoot 8.1 → 7.7 %, final 0.1057 → 0.1001 V.

**Model 1 (`quad_sim`, first_flight_gains: K_rpy 150/200/300, conservative pots, CF α = 0.99995):**

| scenario | ref | final | OS | rise 10–90 | settle 5 % |
|---|---|---|---|---|---|
| roll_step | 15° | 14.80° | 0 % | 0.33 s | 0.51 s |
| pitch_step | −15° | −15.60° | 4.3 % | 0.28 s | 0.38 s |
| yaw_step | 100 deg/s | 99.4 | 13.7 % | 13 ms | 71 ms |

- Hover: attitude rms 2.6/1.7°, xy drift 5.0 m. This comes from Model 1's gyro bias pattern times the 20 s CF time constant.
- With `angle_src = 0` (euler_true): hover rms 0.05°; roll 14.95° (0 % OS, settles in 0.44 s); pitch −15.02°.
- With the spec's α = 0.98 the quad **diverges** in roll_step (361 % overshoot). In flight the accelerometer measures thrust, not gravity, so a 50 ms CF pulls the angle estimate to zero.

### Differences from the legacy roll model (`sim/build_roll_model.m`, `roll_params.m`)

1. **Bug in the legacy model:** `build_roll_model.m` never wires the Command path inport to its `Ktheta` gain, so `roll_axis.slx` ignores `theta_cmd` (the angle command is always 0). The validation fixes this in its temp copy only. The repository file is unchanged.
2. The DAC RC is the loaded RC: 175 Hz on the inverting legs and 167 Hz on the + legs, with the leg gain kept separate. The legacy model uses 159 Hz followed by 0.909.
3. Rails are −2.023/+2.927 V (legacy: −2.0/+2.9). The clamp is the same, 0.75 V.
4. I_HOLD is a physical CD4066 discharge through Ron = 470 Ω, driven by shim logic. The legacy model uses an ideal reset until `t_arm`.
5. Bench mode follows the real wiring: PLANT_OUT goes into the **+ leg** (EXT_SP), W1 into EXT_FB with no RC.
   - With JP_FB = 2 the 1k is out of the FB leg, so **g_sp = 1.000** (not 0.955) and PLANT_OUT → **+W1**.
   - The spec's "−1.05·W1" (§5) is wrong in both sign and gain. U is inverted relative to the legacy model.
6. The ADC reads U directly: LSB 4.88 mV (legacy: 7.1 mV through a divider that sits on the ESC side). The ESC frame is held at 2.5 ms (legacy: t_esc = 0).
7. All 5 loops are modelled, with the yaw differences (R407 15k, D not summed, g_sp 0.909) and the real shim/ADC quantisation.

## Pots ↔ gains, and getting Model 1's tuned gains onto the board

- `gains_from_pots` and `pot_from_gains` use the schematic formulas.
- The ideal-unit conversion is `Kp_us = K_rpy·Kp·g_fb·Kw·180/π`, with g_fb = 0.909. `sim/quad_plant/ctrl_ideal_to_board.m` leaves out g_fb, so its board Kp/Ki/Kd come out **10 % low**; it should divide by `Kmix·Kw·g_fb`.
- Ka = Go·g_ang·(Kθ/Kω)·(g_sp/g_fb) is the same in both.
- With the tuned `ideal` gains, a mixer scale alone cannot fit roll/pitch:
  - Ki/Kd needs about 270 /s².
  - The board's Ki ≥ 7.5 /s and Kd ≤ 14.7 ms force at least 510 /s².
  - Ka is at most 11.9 /s at any Kmix.
- **Parts changes that fit every tuned gain** (checked with `pot_from_gains`), all pots within travel:
  - roll and pitch: C201/C301 1 µF → 2.2 µF with R208/R308 1 M → 470k (Ki 3.4–13.6 /s, wL stays 0.97 rad/s)
  - roll and pitch: C202/C302 100 nF → 220 nF with R209/R309 27k → 12k (Kd 10.3–32 ms, f1 stays 60 Hz)
  - roll and pitch: R219/R319 20k → 12k (Go 0.83–5.0)
  - yaw: Kmix_yaw = 300 µs/V
  - mixer: K_rpy = [200 200 300]
- Resulting settings:
  - roll: RV 27/78/21/41 kΩ
  - pitch: RV 49/33/38/43 kΩ
  - yaw: RV 74/85 kΩ
- `sim/gains/first_flight_gains.m` stays within the current parts instead (results above).

## Notes for the circuit variant and the integrator

- `pid_analog_circuit.slx` must match ANALOG_INTERFACE.md:
  - ports 1–9 / 1–4 are required, 10–15 / 5–6 optional
  - **Fixed-step**, `FixedStep = 'pcb.sim_dt'`
  - Simscape local solver
- Until then, keep `pcb.circuit_enable = 0` (the validation does this itself).
- `quad_set_controller('install_board')` was run, so `quad_sim.slx` now references `pid_board` with `board_variant` 1/2. `qsim.ctrl` was put back to 1 (ideal).
- `pid_board.slx` takes the CtrlIn bus at 1 ms (Inport sample time 1e-3) and does not use `vbat_V`.
