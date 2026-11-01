# pid_analog_circuit: Simscape circuit variant of the PID_Board rev B analog section

Component-level Simscape Electrical model of everything between the MCP4822 DAC pins and the
op-amp outputs read by U3. It is the `pcb.variant == 2` choice of the `Analog section`
Variant Subsystem in `pid_board.slx`; `pid_analog_fast.slx` is the behavioural choice.

Files (all in `sim/pid_board/`):

| File | Purpose |
|---|---|
| `pid_analog_circuit.slx` | The model (referenced model, Simscape network inside) |
| `pid_analog_circuit_params.m` | `pacirc`: part models, solver step, initial conditions (board values come from `pcb`) |
| `pid_analog_circuit_test.m` | Test harness (analytic TFs, limits, reference, circuit vs fast, speed) |

## Ports (identical to ANALOG_INTERFACE.md)

Inports 1-15: `ROLL_ANGLE, ROLL_RATE, PITCH_ANGLE, PITCH_RATE, YAW_RATE, YAW_RATE_CMD,
ROLL_ANGLE_CMD, PITCH_ANGLE_CMD, I_HOLD, EXT_SP_ROLL, EXT_FB_ROLL, EXT_SP_PITCH, EXT_FB_PITCH,
EXT_SP_YAW, EXT_FB_YAW`. Outports 1-6: `U_ROLL, U_PITCH, U_YAW, VGND_A, PLANT_OUT_ROLL,
PLANT_OUT_PITCH`. All absolute volts to GND, scalar double. Inputs 1-8 drive controlled voltage
sources (the DAC pins), 10-15 drive 0-ohm sources on TP209/210/309/310/406/407 (they only matter
when a jumper selects them). Outputs are voltage sensors to GND.

## What is modelled

| Sheet (subsystem) | Contents |
|---|---|
| `DAC_RC` | R5-R12 1k + C15-C22 1 uF to GND; the loading by the 10k diff-amp legs comes out of the circuit itself |
| `PID_Reference` | +5V (U10), R101 1k, TL431 U102 (2.495 V source + 0.2 ohm dynamic impedance), U101A follower (VREF_BUF), RV101 as two resistors (top / wiper-GND = VSET), U101B/C/D followers + R102-R104 100 ohm + C101-C103 470 nF -> VGND_A/B/C |
| `PID_Roll` (masked, `ax = pcb.roll`) | U201A diff amp R201-R204, JP201/JP202, U201B P (R205, R206 + RV201), U201C leaky I (R207 + RV202, C201 1 uF, R208 1M, D201-D204 1N4148 pairs, CD4066 U9A), U201D D (R209 + C202, R210 + RV203 parallel C203), U202A summer R211-R214, U202B angle diff amp R215-R218, U202C angle P R219, R220 + RV204, U202D bench plant R221/C204 |
| `PID_Pitch` (`ax = pcb.pitch`) | Copy of roll, refdes +100, U9B |
| `PID_Yaw` (`ax = pcb.yaw`) | Rate cell only, refdes 4xx, R407 15k, every + input and the diff-amp reference on VGND_B, R413 DNP via `pcb.yaw.en_D = 0`, U9C |
| `EXT_TP` | Sources for the EXT_SP/EXT_FB test points |

Block names are the schematic refdes. Values are expressions on `pcb`/`ax`, so pot, jumper and
enable changes in `pcb` take effect on the next compile:
- Trimpots RVx0y are rheostats (the wiper is tied to one end), so each is one resistor `max(RV, 1 mOhm)`.
- Jumpers are two resistors per jumper (1 mOhm fitted, 10 GOhm open) selected by `JP_SP`/`JP_FB`.
- `en_P/I/D = 0` makes the summer resistor 10 GOhm (DNP).

Test points are logged with signal logging, in volts relative to `pcb.VG`, using the
ANALOG_INTERFACE names: `roll_E` (TP201), `roll_P`, `roll_I`, `roll_D`, `roll_U`, `roll_E_A`
(TP206), `roll_RATE_SP`, `roll_PLANT`; the same with `pitch_`; and `yaw_E/P/I/D/U`. The reference
probes are logged in absolute volts: `ref_VREF_BUF`, `ref_VSET`, `ref_VGND_A/B/C`.

## Part models and deviations from the real parts

- **MCP6004**: ee_lib Band-Limited Op-Amp. A = 90 dB, one pole at GBW/A (GBW 1 MHz), Rin 1e12,
  Rout 50 ohm. Output is hard-limited at 0.025 / 4.975 V absolute (the `pcb` rails), and the
  limiting is asymmetric around VG. Not modelled: slew rate (0.6 V/us swings rail to rail in 8 us,
  which is below any solver step used; enabling it at 0.5 ms makes the op-amp state diverge on
  large steps), Vos (+-4.5 mV), input bias, supply current, and the soft approach to the rails.
- **1N4148**: ee_lib Diode, exponential model, IS 2.52 nA, N 1.752, RS 0.568 ohm, no junction
  capacitance or recovery. The clamp level therefore depends on the current. Measured: 0.73 V at
  about 9 uA, 0.80 V at about 19 uA, 0.65 V at 4 uA. `pcb.V_iclamp` assumes 0.75 V.
- **CD4066**: Simscape Switch. Ron = `pcb.ihold.R_on` (470 ohm, constant, so the real dependence
  on signal level and VDD is ignored), Goff 1e-11 S, threshold VDD/2 on I_HOLD.
- **TL431**: an ideal 2.495 V shunt with 0.2 ohm. RV101 is trimmed by default so that VSET = VG:
  `pacirc.ref.RV_wg` = 8.207k, which gives VGND_A = 2.0479 V. Setting the label value
  `pacirc.ref.RV_wg = pcb.ref.RV_set_wiper_gnd` (8.1k) gives 2.0212 V, as in spec item 9.
- **Not in the model**:
  - The spare followers U402B-D.
  - U1, THROTTLE and DAC_SPARE (outside the analog section).
  - The real rails always load VGND_B; there is no `pcb.ref.ideal` switch. With the default trim,
    VGND_B moves by about 0.2-0.3 mV under signal. This is a real effect, and it shows up as the
    1-3 % yaw stage "errors" in the tests.
- **Initial conditions**: the DAC and rail caps start at VG, and the I/D/plant caps start at 0
  (inputs at rest). If the inputs start away from VG, the DAC RC settles in about 1 ms.

## Solver

The Solver Configuration uses a **local solver**: Backward Euler (L-stable, which damps the
MHz op-amp loop poles), fixed cost, at most `pacirc.nl_iter` = 10 Newton iterations, and step
`pacirc.Ts`. The model is Fixed-step ode4 with step `auto`. It has no continuous Simulink
states; the Simscape network is discrete at `pacirc.Ts`.

- **Default `pacirc.Ts = 0.5 ms`** so it runs inside the Model 1 board (Fixed-step ode4, 0.5 ms).
  A referenced model's rates must be integer multiples of the parent's fixed step, so a
  sub-multiple (for example 20 us) is **not** possible under a 0.5 ms fixed-step parent. Both
  parent types were verified with a scratch parent: Fixed-step ode4 at 0.5 ms, and ode45 with
  MaxStep 1e-4 (the `pid_board_tb_analog`/`_bench`/`_cl` set-up).
- **`pacirc.Ts = 20e-6`** gives full fidelity. It needs a variable-step parent, or a fixed-step
  parent whose step is at most 20 us.
- To integrate the network with the parent's solver instead, run
  `set_param('pid_analog_circuit/Solver_Configuration','UseLocalSolver','off')`. That needs a stiff
  variable-step parent (ode23t/daessc); ode45/ode4 will not handle the op-amp poles.

## Running

```matlab
run('<repo>/sim/pid_board/pid_board_params.m');           % pcb
run('<repo>/sim/pid_board/pid_analog_circuit_params.m');  % pacirc (PostLoadFcn also loads both if missing)
addpath('<repo>/sim/pid_board');
R = pid_analog_circuit_test();                          % all tests at pacirc.Ts
R = pid_analog_circuit_test('steps','sweep','Ts',20e-6); % full-fidelity step
```

Standalone simulation: `sim` with an ExternalInput `[t, 15 columns of absolute volts]`.
Use `SaveFormat` `StructureWithTime`; Dataset yout was very slow here.

## Test results

The results below use `pcb` as found in the session at test time (roll/pitch RV_P = 20k at the
last run, so Kp = 0.526). All values are relative to VG.

**Steps**, measured relative to a rest run (drift removed):
- Small steps: roll FB +50 mV, pitch ANGLE_CMD +40 mV, yaw CMD +50 mV.
- At 20 us, U matches the analytic response within 0.9 % (roll), 1.7 % (pitch) and 1.1-2.2 % (yaw)
  of peak.
- Stage gains:

  | Gain | Measured | Expected |
  |---|---|---|
  | Kp | 1.1637 | 1.1638 |
  | Go | 1.7498 | 1.750 |
  | g_sp (roll/pitch) | 0.963 | 0.955 |
  | g_fb | 0.917 | 0.909 |
  | g_EA | 0.896 | 0.909 |

- I and D stage traces match `-Ki/(s+wL)` and the band-limited D within 0.5-1.5 %.
- The yaw stages are 2-3 % off when measured relative to VG, because VGND_B shifts by about
  0.23 mV for a 50 mV step. Relative to VGND_B they match.
- At 0.5 ms, P and I are unchanged. The D-kick peak is under-resolved (D trace about 8-10 %,
  U peak about 18-20 %), but the final values are identical.

**Multisine** (10 tones, 0.4-300 Hz, 4 mV each), circuit vs analytic:

| Ts | Up to 50 Hz | 100-300 Hz |
|---|---|---|
| 20 us | within 0.1 dB / 0.5 deg (roll, yaw) | pitch +0.5 dB / +2.4 deg (loaded angle legs, not in the spec's simplified TF) |
| 0.5 ms | within 0.65 dB / 1.6 deg | -0.5 to -2.0 dB, +3 to +39 deg (Backward Euler warping of the D term) |

**Limits**:
- U_ROLL reaches the +rail at 4.979 V (needs the D kick). P_OUT bottoms at -2.02 V, so P + I
  alone cannot reach +2.93 V re VG.
- U_PITCH reaches the -rail at 0.026 V (both at 20 us).
- At 0.5 ms the D kick is under-resolved: the peaks are 4.80 V and 0.12 V.

**I_HOLD**:
- Residual I_OUT during hold: 4-9 mV (E x Ron/R_I).
- 90 % discharge: 1.12 ms at 20 us (Ron x C = 0.47 ms), 2.0 ms at 0.5 ms (step-limited).

**Reference**: VGND_A is 2.0479 V trimmed and 2.0212 V with the 8.1k label.

## Circuit vs pid_analog_fast (same inputs, fast at ode4 0.5 ms)

- **Steps**: max |U difference| is 25 mV (roll), 33 mV (pitch) and 12 mV (yaw) out of
  0.14-0.36 V. The differences are concentrated in the D-kick transient. Final values agree, and
  VGND_A agrees.
- **Multisine** (circuit at 0.5 ms vs fast): within 0.1 dB / 1.6 deg up to 20 Hz, -0.65 dB at
  50 Hz, -1.3 to -2.3 dB and +20 to +39 deg at 200-300 Hz.
- This is the same as circuit vs analytic. **The fast model matches the analytic TFs**; the
  difference comes from the circuit's 0.5 ms Backward Euler step.
- At 20 us the circuit reproduces the analytic TFs (and hence the fast model) within about
  0.15 dB / 0.5 deg for roll and yaw. Pitch differs by +0.5 dB / +2.4 deg at 300 Hz: the fast
  model uses the spec's simplified angle-leg gains.

## Speed

- **Compile**: about 21-27 s per (re)compile.
- **Clean execution rates**:

  | Ts | Execution | 10 s run (estimate) |
  |---|---|---|
  | 0.5 ms | about 0.5 s per simulated second (1 s run: 0.5 s) | about 5 s + compile |
  | 20 us | about 4.2 s per simulated second (0.2 s run: 0.84 s) | about 42 s + compile |

- **Shared-session caveat**: in the shared MATLAB session, any run longer than about 1 s of wall
  time picked up a one-off stall of about 110-130 s. This came from other sessions' work running
  inside the simulation's event processing, so the raw wall times recorded were inflated (for
  example, 10 s at 0.5 ms measured 132 s total).
- **Closed loop**: closed-loop timing (pid_board_cl with `pcb.variant = 2`) was not measured. The
  network cost does not depend on loop closure. Under the 0.5 ms Model 1 parent it adds about
  0.5 s per simulated second.
