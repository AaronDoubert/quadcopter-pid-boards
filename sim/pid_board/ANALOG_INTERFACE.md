# Analog section interface (pid_analog_fast.slx / pid_analog_circuit.slx)

Contract shared by the two implementations of the PID_Board rev B **analog section**
(everything between the MCP4822 DAC output pins and the op-amp outputs read by U3).
`pid_board.slx` holds both in a Variant Subsystem (`Analog section`) and wires them
identically, so **port names, order, units and boundaries below must not change**.

- Fast/behavioural variant: `pid_analog_fast.slx` (built by `pid_board_build.m`).
- Circuit variant: `pid_analog_circuit.slx` (Simscape Electrical, separate author).

## Boundary

- **Inputs are the MCP4822 VOUT pins**, i.e. the DAC voltages *before* the 1k series
  resistors R5-R12. The 1k/1uF RC filters R5-R12/C15-C22 and their loading by the
  10k diff-amp legs are **inside** the analog model.
- THROTTLE (R13/C23) and DAC_SPARE (R14/C24) are **not** part of the analog section
  (THROTTLE goes straight to U3; it is modelled in `pid_board.slx` / control_io).
- **Outputs are the op-amp output nodes** (U202A, U302A, U402A, U202D, U302D) and
  the VGND_A rail, exactly what U3's ADC (or a scope probe) sees. No ADC is inside.
- The CD4066 U9 integrator reset is inside (driven by I_HOLD).
- The reference (TL431, U101, VGND_A/B/C rails) is inside. In the fast model
  `pcb.ref.ideal = 1` makes all three rails exactly `pcb.VG` (2.048 V).
- Jumpers JPx01/JPx02 and trimpots RVx01-RVx04 are parameters read from the base
  workspace struct `pcb` (`pcb.roll.RV_P`, `pcb.roll.JP_SP`, ...; see
  `pid_board_params.m`). No jumper/pot is a port.

## Units / signal types

All ports: scalar `double`, **absolute node voltage in volts (0..5 V, referred to GND)**,
continuous time (sample time inherited / continuous). Nominal quiescent level is
VG = 2.048 V on every analog input/output. I_HOLD is a 0 / 5 V logic level (CD4066
control, switch closed when > VDD/2 = 2.5 V).

## Inports (order is fixed)

| # | Name | Net / source | Notes |
|---|---|---|---|
| 1 | `ROLL_ANGLE` | U4 VA -> R5/C15 | + leg of U202B (loaded RC 0.952 ms) |
| 2 | `ROLL_RATE` | U4 VB -> R6/C16 | JP202 1-2 -> - leg of U201A (0.909 ms) |
| 3 | `PITCH_ANGLE` | U5 VA -> R7/C17 | + leg of U302B |
| 4 | `PITCH_RATE` | U5 VB -> R8/C18 | JP302 1-2 -> - leg of U301A |
| 5 | `YAW_RATE` | U6 VA -> R9/C19 | JP402 1-2 -> - leg of U401A |
| 6 | `YAW_RATE_CMD` | U6 VB -> R10/C20 | JP401 1-2 -> + leg of U401A (0.952 ms) |
| 7 | `ROLL_ANGLE_CMD` | U7 VA -> R11/C21 | - leg of U202B |
| 8 | `PITCH_ANGLE_CMD` | U7 VB -> R12/C22 | - leg of U302B |
| 9 | `I_HOLD` | U1 PA7 | 0/5 V; high = CD4066 A/B/C closed across C201/C301/C401 |
| 10 | `EXT_SP_ROLL` | TP209 | used only when `pcb.roll.JP_SP == 2` (0-ohm source assumed) |
| 11 | `EXT_FB_ROLL` | TP210 | used only when `pcb.roll.JP_FB == 2` (no RC, gain 1.0) |
| 12 | `EXT_SP_PITCH` | TP309 | `pcb.pitch.JP_SP == 2` |
| 13 | `EXT_FB_PITCH` | TP310 | `pcb.pitch.JP_FB == 2` |
| 14 | `EXT_SP_YAW` | TP406 | `pcb.yaw.JP_SP == 2` |
| 15 | `EXT_FB_YAW` | TP407 | `pcb.yaw.JP_FB == 2` |

Ports 1-9 are the core interface. Ports 10-15 (bench / external test-point inputs)
are needed for the LOOP_SP/LOOP_FB bench mode; `pid_board.slx` ties them to VG
unless a bench mode is selected (`pcb.bench.*`). A circuit model that does not
implement bench mode may omit 10-15 (the Variant Subsystem tolerates a subset of
ports), but must then keep 1-9 in this order.

## Outports (order is fixed)

| # | Name | Node | Notes |
|---|---|---|---|
| 1 | `U_ROLL` | U202A out (TP205, J1-1, U3 PA1) | |
| 2 | `U_PITCH` | U302A out (TP305, J1-2, U3 PA2) | |
| 3 | `U_YAW` | U402A out (TP405, J1-3, U3 PA3) | |
| 4 | `VGND_A` | VGND_A rail (TP103, J1-6, U3 PA7) | |
| 5 | `PLANT_OUT_ROLL` | U202D out (TP208) | bench integrator, 22 ms, rails in flight |
| 6 | `PLANT_OUT_PITCH` | U302D out (TP308) | |

Outports 5-6 are optional for the circuit variant (same rule as inports 10-15).

## Logged test points (fast model; recommended names for the circuit model)

Signal logging on, names `<axis>_<tp>`, volts **relative to VG**:
`roll_E_A` (TP206), `roll_RATE_SP` (TP207), `roll_E` (TP201 E_RR), `roll_P` (TP202),
`roll_I` (TP203), `roll_D` (TP204), `roll_U` (TP205), `roll_PLANT` (TP208); same with
`pitch_` (TP3xx); `yaw_E` (TP401), `yaw_P`, `yaw_I`, `yaw_D`, `yaw_U` (TP402-405).

## Solver notes (updated: fixed step)

The analog model is a referenced model (Normal mode) inside `pid_board.slx`, which is
itself referenced by Model 1 (`quad_sim.slx`, **Fixed-step ode4, qsim.dt = 0.5 ms**).
Therefore every analog model must be:

- `SolverType` **Fixed-step**, `Solver` ode4 (or auto), `FixedStep` = `'pcb.sim_dt'`
  (0.5 ms; a referenced model with continuous states must use the parent's step exactly,
  `'auto'` resolves to StopTime/50 and fails).
- Stable and accurate with ode4 at 0.5 ms: fastest real pole |lambda| <~ 2.8/h = 5600 rad/s.
  The fast model's fastest pole is the CD4066 reset, 1/(470 ohm * 1 uF) = 2130 rad/s.
  A Simscape network needs a local solver (Solver Configuration: "Use local solver",
  Backward Euler / Trapezoidal, sample time pcb.sim_dt or a divisor) or it will not run.

`pid_board.slx` selects the analog section with its model argument `board_variant`
(1 fast, 2 circuit). Model arguments need Variant activation time **startup**, which compiles
**both** choices. The circuit choice sits behind an inner `update diagram` variant on
`pcb.circuit_enable` (default 1): while `pid_analog_circuit.slx` is missing, unsaved or not
fixed-step compatible, set `pcb.circuit_enable = 0`, otherwise every pid_board simulation
(including the fast one) fails to compile.

The fast model has no algebraic loops; the bench flying lead PLANT_OUT -> EXT_SP closes in
`pid_board.slx` through a 0.5 ms lag (`pcb.bench.lead_tau`).
