# Gains

Two sets of gains live in this repo:

| Set | Where | Use |
|---|---|---|
| **First flight (conservative)** | [`first_flight_gains.md`](first_flight_gains.md), made by [`first_flight_gains.m`](first_flight_gains.m) | Set the trimpots to these before the first hover |
| Tuned (aggressive) | `sim/quad_plant/ctrl_ideal_params.m`, made by `sim/quad_plant/tune_ideal.m` | Upper bound, for reference; several values fall outside the board's pot ranges |

## How the first-flight set was chosen

The search works in **board units**: the trimpots RV_P, RV_I, RV_D and RV_A, plus a per-axis mixer scale `Kmix`. `Kmix` is the µs of ESC pulse per volt of controller output U. It's a constant in the control_io firmware, `pcb.ctrl.K_rpy`.

Each candidate is turned into the controller shape the board actually has:
- a leaky integrator (1 MΩ across 1 µF)
- the D-term roll-off poles, where the second pole moves with RV_D
- the 400 Hz control_io loop delay

It's then checked on the linearised hover plant. "Wary" means every one of these must hold:

- **Margins:** at least 55° phase and 10 dB gain margin on the nominal plant. The aggressive set uses 45° and 8 dB.
- **Robustness:** at least 35° and 6 dB with inertia at 1.7× (battery on top) or 0.7×, and with 2 ms of extra unmodelled delay.
- **Pot travel:** every pot between 10% and 90% of travel, so you can tune both ways.
- **Integrator clamp:** at least 75 µs, enough to trim about a 20 mm CG offset.
- **Angle loop:** at least 70° phase margin, and at least 5× slower than the rate loop.

## Checked with the board twin flying the quad

These are the pots and mixer scales above, in Model 2 (both analog variants) inside Model 1:

| | Roll step | Pitch step | Yaw step | Hover rms | Takeoff tilt |
|---|---|---|---|---|---|
| Board twin, fast | 0% OS, 0.55 s | 4.2% OS, 0.41 s | 14% OS, 0.07 s | 2.6° / 1.7° | 2.1° |
| Board twin, Simscape circuit | 0% OS, 0.56 s | 4.0% OS, 0.41 s | 12% OS, 0.07 s | 2.6° / 1.7° | 2.1° |

Hover rms is roll / pitch. The hover error comes from the shim's assumed angle estimator. With true angles it is 0.05°; see `sim/README.md`.

## Checked in the nonlinear sim (Model 1, ideal controller with these gains)

| Test | Result |
|---|---|
| 15° roll / pitch step | 0.0% / 0.2% overshoot, 0.45 s to settle (5%) |
| 100 °/s yaw step | 2.7% overshoot, 0.07 s |
| Hover | 0.04° rms attitude noise |
| Full-throttle punch | 2.8° max attitude disturbance |
| Takeoff | 0.7° max tilt |
| Inertia 0.7× to 1.7× | No change (0–0.2% overshoot); at 2.5×: 4–4.5% overshoot |
| CG offset 5 / 10 / 20 mm | Takeoff tilt about 2 / 4 / 7.5°, hover tilt about 0.2 / 0.4 / 0.87° |

**Keep the CG within about 5 mm of the motor centre.** The soft gains handle wrong inertia easily. But because the board's integrator leaks, it can't fully hold the constant torque an off-centre CG needs. The quad leans and drifts slightly, and it tilts more at takeoff, before the integrator has caught up.

## First-flight procedure (suggested)

1. With the board powered off, set every pot with a meter to the kΩ in `first_flight_gains.md`. The pot table there lists which RV is which.
2. Load the per-axis mixer scale into the control_io firmware.
3. Props off: arm, tilt the quad by hand, and check that each motor speeds up on the side that drops.
4. Hover tethered or low over grass. If it wobbles fast, lower RV_P and RV_D 10% at a time. If it feels mushy and drifts, raise Go (RV_A).
5. Once it's stable, tune one axis at a time toward the tuned set, watching the test points.

Rerun the design after changing anything physical (CAD masses, a different battery):

```matlab
FF = first_flight_gains();            % prints and rewrites first_flight_gains.md
FF = first_flight_gains('apply', true); robustness_sweep(evalin('base','ideal'))
```
