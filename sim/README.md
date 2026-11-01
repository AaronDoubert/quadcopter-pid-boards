# Simulation: 5" quad + analog PID board twin

Two linked Simulink models (MATLAB R2024b):

| | Model | What it is for |
|---|---|---|
| **1** | [`quad_plant/quad_sim.slx`](quad_plant/) | The 5" quad itself: 6-DOF airframe, ESC, 2207 1800KV motors on 5x4.3x3 props, 4S battery with sag, MPU-6050 noise and delay. Has a controller slot. |
| **2** | [`pid_board/pid_board.slx`](pid_board/) | Digital twin of PID_Board rev B: shim (IMU, DAC), 5 analog loops, control_io (ADC, mixer, ESC pulses). The analog section comes in two variants: **fast** (behavioural, from component values) and **circuit** (Simscape Electrical, real op-amps, pots and diodes). |

Both honour one interface ([`common/quad_ctrl_buses.m`](common/quad_ctrl_buses.m)): bus `CtrlIn` in (gyro, accel, truth, sticks, arm, vbat) → `esc_us` [4×1] out. So the controller slot in Model 1 can hold:

1. `ideal`: an idealised 5-loop cascaded PID with the board's loop shape ([`quad_plant/ctrl_ideal.slx`](quad_plant/)), used to learn gains
2. `board_fast`: Model 2 with the fast analog section
3. `board_circuit`: Model 2 with the Simscape circuit

Model 2 also runs standalone on a bench harness (`pid_board/pid_board_bench.slx`), so you can compare against scope traces on the real board's test points.

![Model 1 top level](docs/img/m1_quad_sim_top.png)

## Results so far

| | |
|---|---|
| ![roll step](docs/img/m1_roll_step.gif) | ![yaw step](docs/img/m1_yaw_step.gif) |

- **Hover:** 1389 µs (39% duty) on a full pack; rpm and current match the thrust-stand data.
- **Tuned ideal gains:** 15° roll/pitch steps settle in 0.13 s with about 1% overshoot.
- **[First-flight gains](gains/first_flight_gains.md):** conservative trimpot settings and mixer scales. They settle in 0.43 s with no overshoot, and keep at least 35° phase margin at 0.7–1.7× inertia or 2 ms of extra delay. See [`gains/README.md`](gains/README.md).
- **Robustness** (`quad_plant/robustness_sweep.m`): wrong inertia is harmless; CG offset matters. Keep the CG within about 5 mm of the motor centre.
- **ESC warning:** a full-throttle punch draws about 30 A per motor (119 A total). The Aero Selfie 4in1 is rated 10 A continuous per channel.

## Quick start

```matlab
cd sim/quad_plant
quad_sim_params                      % quad, ideal gains, CtrlIn bus, sim options
M = run_quad_scenarios();            % hover, steps, punch, takeoff, battery drain (+ plots)

cd ../gains
FF = first_flight_gains();           % conservative pot settings -> first_flight_gains.md

cd ../docs
make_flight_gif('roll_step', 'img/my_run.gif', 'ctrl', 1)   % 1 ideal, 2 board fast, 3 board circuit
```

## Layout

| Folder | Contents |
|---|---|
| [`quad_plant/`](quad_plant/) | Model 1, parameters (`quad_params.m` + sources), scenarios, tuning, robustness sweep. [README](quad_plant/README_model1.md) |
| [`pid_board/`](pid_board/) | Model 2: spec extracted from the KiCad schematics, `pcb` parameters, fast and circuit analog sections, board model, bench harness, validation, pot↔gain conversion |
| [`gains/`](gains/) | First-flight (conservative) trimpot settings and how they were chosen |
| [`common/`](common/) | Interface contract (`CtrlIn` bus, conventions) |
| [`docs/`](docs/) | Screenshots, GIFs and the GIF generator |
| `*.m` in this folder | The original roll-axis-only model (`build_roll_model.m`, `roll_params.m`, ...), superseded by Model 2 |

## Conventions

- Body axes: x forward, y right, z down. Roll right, nose up and nose right are positive.
- Motors (Betaflight Quad-X, seen from above): M1 rear-right CW, M2 front-right CCW, M3 rear-left CCW, M4 front-left CW.
- Sticks: roll, pitch and yaw in [-1, 1], throttle in [0, 1]. Pitch +1 = stick forward = nose down.

## Known gaps / to confirm on the real hardware

- **Shim and control_io firmware not found.** Loop rates, gyro scale (6 vs 8.19 mV per °/s), ESC protocol and mixer scale are assumptions, and each is a parameter in `pcb`. See `pid_board/pid_board_spec.md` section 8.
- **Physical numbers to measure:** weigh the quad, find the CG, run a bifilar pendulum for inertia, log hover throttle. CAD mass properties can replace the estimates in `quad_params.m`.
- **ESC current rating** vs about 30 A peak per motor.
