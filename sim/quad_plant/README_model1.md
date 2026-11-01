# Model 1: 5" quad 6-DOF plant + ideal 5-loop PID

Simulink model of the 5" analog-PID quad (`quad_params.m`) with an ideal discrete
cascaded PID in a controller slot. It is used to learn gains. Model 2, the
PID-board digital twin (`sim/pid_board/pid_board.slx`), plugs into the same slot.

## Files (all in `sim/quad_plant/`)

| File | What it is |
|---|---|
| `quad_sim.slx` | Top model: radio/pilot, controller slot, ESC, motors + battery, 6-DOF airframe, IMU, CtrlIn bus, logging |
| `ctrl_ideal.slx` | Referenced controller model, Inport `ctrl_in` (bus CtrlIn), Outport `esc_us` [4x1] |
| `quad_sim_params.m` | Setup script: runs `quad_params`, `../common/quad_ctrl_buses`, `ctrl_ideal_params`, and defines `qsim` |
| `ctrl_ideal_params.m` | Struct `ideal`: gains, stick scaling, arm/idle, mixer (defaults = tuned values) |
| `quad_sim_derive.m`, `quad_scenario.m` | Derive `qsim` sub-structs, load a scenario into `qsim.cmd` |
| `run_quad_scenarios.m` | Runs the scenarios, prints metrics, saves plots to `results/quad_<scenario>.png` |
| `tune_ideal.m` | Gain tuning on analytic per-axis linear plants (Control System Toolbox) |
| `ctrl_ideal_to_board.m` | Maps ideal gains to board Kp/Ki/Kd/Go and trimpot values; flags values out of range |
| `quad_set_controller.m` | Selects the controller variant, or installs `pid_board.slx` into the slot |
| `quad_build_models.m` | Rebuilds both `.slx` files from source (the code lives in the files below) |
| `ctrl_ideal_core.m`, `quad_fcn_*.m` | MATLAB Function block sources: controller, motor/battery, forces, IMU, pilot |

Base-workspace variables used: `quad`, `CtrlIn`, `ideal`, `qsim`. Nothing else is created.

## How to run

```matlab
run('C:\Users\doubi\Quadcopter Flight controller\sim\quad_plant\quad_sim_params.m')
out = sim('quad_sim');                        % default scenario: 10 s hover
M = run_quad_scenarios();                     % all scenarios, metrics + plots
M = run_quad_scenarios({'roll_step'});        % one scenario
qsim = quad_scenario(qsim, quad, ideal, 'yaw_step'); out = sim('quad_sim');
[g, info] = tune_ideal();                     % re-tune (add 'apply',true to write into `ideal`)
B = ctrl_ideal_to_board(ideal, pcb);          % needs pcb from sim/pid_board/pid_board_params.m
```

Both models have a `PreLoadFcn` that runs `quad_sim_params` if any of the four
variables is missing. Logged signals are in `out.logsout`: Xe, Ve, euler, Vb,
pqr, f_imu, thrust_N, w_radps, motor_rpm, I_motor, I_bus, vbat_V, soc, duty,
esc_us, sticks, arm, gyro, accel, and ctrl_dbg (from inside ctrl_ideal:
[rate_sp; e; u; I; D]). A 10 s flight runs in about 1.2 s of wall time
(ode4, fixed step 0.5 ms). The 240 s battery run takes about 14 s.

## Architecture (quad_sim.slx)

```
Radio (qsim.cmd table + pilot) -> CtrlBus (ZOH 1 kHz, nonvirtual bus CtrlIn) -> Controller [Variant]
  -> ESC (490 Hz frame ZOH -> 2.2 ms latency -> duty) -> Motors_Battery -> Airframe (forces + 6DoF quaternion)
  -> IMU (MPU-6050) -> CtrlBus;  truth pqr/euler, vbat -> CtrlBus;  Xe/Ve -> pilot
```

- **Radio**: `qsim.cmd` is a table with columns `[t roll pitch yaw thr arm alt_en alt_ref]`, read by a From Workspace block.
  - With `alt_en = 1`, a simulated pilot moves the throttle stick to hold `alt_ref`: climb-rate demand, then a PI on the climb-rate error. This is outside the controller, which only sees a throttle stick.
  - Angle mode itself has no altitude loop.
- **ESC**: the PWM frame is a ZOH at `qsim.esc.Ts`, which is 2.0 ms: 490 Hz rounded to the 0.5 ms solver step.
  - It is followed by a transport delay of `quad.esc.latency` minus half a frame, which is 2.2 ms. The total average latency is 3.2 ms.
  - Duty = (us - 1000)/1000, clipped to 0..1. Duty 0 means the FETs are off.
- **Motors_Battery** (`quad_fcn_motor_batt.m`): DC-equivalent motor driven by V_m = duty·V_bus.
  - I = (V_m - w/Kv)/R. The shaft torque is Kt(I - I0·tanh(w/20)), and J_r·dw/dt = Q_m - kQ·w².
  - Each ESC channel is current-limited to 30 A (`quad.esc.I_limit`).
  - The battery is cells·OCV(SOC) - R_total·I_bus. It is solved in closed form, so there is no algebraic loop.
  - I_bus = Σ duty·I_m + avionics. SOC is integrated from it.
  - Damped-light braking (negative current) is allowed.
- **Airframe** (`quad_fcn_forces.m` + Aerospace Blockset `6DoF (Quaternion)`):
  - Thrust kT·w² at each hub, relative to the CG (r_motor). The CG offset is included, so the front motors run about 3 % faster.
  - Yaw torque is -spin·Q_m. This is the stator reaction, so rotor acceleration helps yaw.
  - Rotor gyroscopic moment.
  - Drag: linear plus quadratic, and rotational damping k_rot.
  - Gravity.
  - Ground at z = 0: a spring-damper that only pushes up, plus sliding damping and a levelling torque.
  - The specific force at the IMU location includes the lever-arm terms.
- **IMU** (`quad_fcn_imu.m`, 1 kHz): scale error, residual bias, and white noise (density·√(fs/2), from a seeded Random Number block).
  - Gyro g-sensitivity.
  - DLPF: a 98 Hz first-order filter plus a 1-sample delay, which matches the 2.8 ms of DLPF_CFG 2.
  - ±250 deg/s and ±2 g saturation, and LSB quantisation.
- **CtrlBus**: every element is ZOH-sampled at 1 kHz (`qsim.Ts_bus`), because a nonvirtual bus must have a single rate. `rates_true` and `euler_true` are the true values.

## Controller (ctrl_ideal.slx, `ctrl_ideal_core.m`, 1 kHz)

The structure mirrors the board (`sim/pid_board/pid_board_spec.md`). Units: u in µs.

- Setpoints:
  - Roll/pitch: rate_sp = Ka·(angle_cmd - euler_true).
  - Yaw: rate_sp = yaw stick · 250 deg/s.
  - rate_sp is clipped at ±341 deg/s.
- Rate PID on e = rate_sp - gyro: u = Kp·e + I + D, clipped to ±400 µs.
  - I is leaky: dI/dt = Ki·e - wL·I, with wL = 1 rad/s and |I| ≤ 150 µs. I is held at 0 when disarmed or when the throttle stick is below 0.05 (board I_HOLD).
  - D = Kd·s/((1+s/w1)(1+s/w2)) on the **error**, with w1 = 59 Hz and w2 = 164 Hz, which are the board's R209·C202 and Rd·Cf.
- Sticks: angle ±60°. Pitch stick +1 means nose down.
- Mixer: thr_us = 1050 + thr·950, then esc = thr_us + mix·u, clipped to [1050, 2000]. Disarmed gives 1000 µs.
  - The mixer columns use **this model's** signs: pitch + = nose up, so the front motors go up.
  - The pitch column is the negative of the Betaflight-style column in pid_board_spec §6.

## Tuned gains (`tune_ideal.m`, now the defaults)

Method:
1. Hand-linearise each axis at hover:
   - Motor: Km/(τs+1), with τ = 22.9 ms.
   - Torque arm or yaw (J_r·s + 2kQ·w0) term, then 1/(J·s + k_rot).
   - Delay of 4.7 ms (controller ZOH, half ESC frame, ESC latency, IMU sample) plus the 98 Hz DLPF, 4th-order Padé.
2. Seed with `pidtune`.
3. Grid-search Kp and Kd/Kp (Ki = 6·Kp, board-structured C(s)) for the highest crossover with PM ≥ 45° and GM ≥ 8 dB.
4. Set Ka for an angle-loop PM of 60°, capped at rate crossover / 4.

| axis | Ka 1/s | Kp µs/(rad/s) | Ki µs/rad | Kd µs/(rad/s²) | rate wc rad/s | PM / GM | angle wc rad/s |
|---|---|---|---|---|---|---|---|
| roll | 16.9 | 42.5 | 255 | 0.940 | 67.6 | 54.7° / 8.3 dB | 17.8 |
| pitch | 17.6 | 71.4 | 429 | 1.17 | 70.3 | 45.6° / 8.0 dB | 19.2 |
| yaw | – | 156 | 934 | 0 | 104 | 69.2° / 8.4 dB | – |

## Scenario results (ideal controller, `run_quad_scenarios`)

| Scenario | Result |
|---|---|
| hover 10 s | ESC 1389 µs, duty 0.389, which matches `Vm_hover/vbat` = 0.389 at the simulated 16.56 V (`quad.checks.hover_duty` = 0.424 is at the 15.4 V nominal OCV). rpm 10.49k vs check 10.49k, I_motor 4.39 A vs check 4.39 A. Attitude rms 0.018°, altitude rms 1 mm |
| roll step 15° | overshoot 1.2 %, rise 88 ms, 5 % settle 132 ms, steady-state error -0.1 % |
| pitch step 15° nose down | overshoot 1.3 %, rise 82 ms, settle 132 ms |
| yaw step 100 deg/s | overshoot 1.0 %, rise 10 ms, settle 145 ms, steady-state error -1.1 % (leaky integrator) |
| throttle punch 0.5 s | I_bus peak 119 A (ESC limit 30 A per motor), vbat 16.6 → 13.5 V (3.1 V sag), 22.9k rpm (check 22.8k) |
| takeoff from ground | disarmed output 1000 µs, ground penetration 3.2 mm (static), climbs to 1.50 m |
| battery drain 240 s | SOC 1.0 → 0.67. Hover vbat 16.57 → 15.46 V and hover ESC 1389 → 1417 µs (the pilot adds throttle as the pack sags). At punches at 20/120/220 s: min vbat 13.4/12.8/12.4 V, peak rpm 22.8k/22.2k/21.7k, peak thrust 29.2/27.7/26.5 N |

## Plugging in the board twin (Model 2)

The slot is the Variant Subsystem `quad_sim/Controller`. Its variant control is an expression on `qsim.ctrl`:

| Choice | Condition | Content now |
|---|---|---|
| `ideal` | `qsim.ctrl == 1` (default) | Model block → `ctrl_ideal` |
| `board_fast` | `qsim.ctrl == 2` | placeholder subsystem (outputs 1000 µs) |
| `board_circuit` | `qsim.ctrl == 3` | placeholder subsystem |

Contract for `sim/pid_board/pid_board.slx`:
1. Inport 1 is named `ctrl_in`, with data type `Bus: CtrlIn` (output as nonvirtual bus). Outport 1 is named `esc_us`: double [4x1], µs, in motor order M1 RR, M2 FR, M3 RL, M4 FL.
2. The bus arrives at 1 kHz (`qsim.Ts_bus`). Use Inport sample time -1 or 1e-3.
3. The parent solver is **ode4, fixed step `qsim.dt` = 0.5 ms**. Configure the board model as Fixed-step with step `auto`:
   - fast (discrete) variant: solver `FixedStepDiscrete` or `auto`;
   - circuit variant with continuous states: solver ode4/auto. It must be fixed-step compatible.

   Discrete rates must be multiples of 0.5 ms.
   - The board spec's 400 Hz ESC frame (2.5 ms) is fine.
   - The ATtiny ADC/mixer at other rates must be rounded, or `qsim.dt` reduced.
4. Use only base variables `pcb` (+ `quad`/`CtrlIn` if needed). It must compile with `qsim.ctrl == 1` active even before it exists.
5. Install it with `quad_set_controller('install_board')`.
   - This replaces both placeholders with Model blocks referencing `pid_board` and keeps the variant conditions.
   - If `pid_board` declares a model argument `board_variant`, the fast choice gets `1` and the circuit choice `2` (names and values are options). Otherwise both blocks reference the same model, and the board's own variant control must pick fast/circuit.
   - The install was tested with `ctrl_ideal` as a stand-in, then the model was rebuilt.
6. Select a choice with `quad_set_controller('board_fast')` (it sets `qsim.ctrl`), then run `run_quad_scenarios` as usual.
7. **Sign conventions**: this plant is FRD with θ > 0 = nose up.
   - The board spec's mixer (§6) uses a Betaflight pitch column (nose-down positive).
   - The board model must map `euler_true(2)`, `gyro_radps(2)` and the pitch stick consistently. Check with `pitch_step` that a +0.25 stick gives θ → -15°.
   - The board does not use `vbat_V`. It is there for the ESC/plant only.

## Assumptions and deviations

- **ESC frame**: 490 Hz is rounded to 500 Hz (2.0 ms) so it fits the fixed step. Motor inductance is neglected. The ESC is an ideal buck with damped-light braking and a 30 A limit.
- **Ground**: a single contact point at the CG (spring-damper) plus a levelling torque. It is not a 4-leg model.
- **IMU**: residual biases are a fixed deterministic pattern (0.5 deg/s gyro scale, `qsim.imu.bias_*_pat`), and the sensor axes are aligned with the body. Set `qsim.imu.noise_on`/`bias_on` to 0 for clean runs.
- **Ideal controller angles**: it uses `euler_true`, because it has no attitude estimator.
- **Stimulus**: scripted signal set (`quad_scenario.m`) instead of the Signal Editor. The pilot altitude hold is a test convenience, not part of the controller.
- **Not done**: 3D visualisation (skipped as not cheap enough). `sdo` response optimisation was not needed; the linear tuning verified well in the nonlinear sim.
- **Not used**: nothing from the asbQuadcopter example project is used or required. Only the Aerospace Blockset 6DoF library block is used.

## Open issues

- `ctrl_ideal_to_board(ideal, pcb)` with Kmix = 200 µs/V (placeholder) gives:
  - roll Kp_b 0.62, Ki_b 3.7 /s, Kd_b 13.7 ms, Go 4.3;
  - pitch Kp_b 1.04, Ki_b 6.2, Kd_b 17 ms, Go 4.4;
  - yaw Kp_b 2.27, Ki_b 13.6.

  Several are outside the trimpot ranges:
  - Go is at most 3.0 (Ka ≤ 11.9 /s), whatever Kmix is.
  - Roll/pitch Ki is below the minimum of 7.5 /s.
  - Pitch Kd is above 14.7 ms.
  - Yaw Kp is above 2.23.

  Retune with `tune_ideal('wi', ...)` and a Ka cap, or change the board's fixed resistors or Kmix.
- Gains were tuned at hover with a full pack. Loop gain scales with V_bus, so it is about 7 % lower at 15.5 V.
- Mass, inertia, kQ, R and drag are still estimates (see `quad_params_sources.md`).
