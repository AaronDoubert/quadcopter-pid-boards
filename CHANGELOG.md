# Changelog

All boards are KiCad 10 projects. Each version is tagged in git. The latest through-hole board is the
one in `PID_Board/`; older through-hole revisions are kept in `PID_Board/old_versions/`.

## Simulation (`sim/`)

### 2026-11-01: quad model + board twin
- **Quad model** `sim/quad_plant/quad_sim.slx`: 6-DOF 5" quad (2207 1800KV, 5x4.3x3, 4S, 650 g) with ESC, motors, battery sag, IMU and ground contact. It has a controller slot that holds the ideal PID or the board twin.
- **Board twin** `sim/pid_board/pid_board.slx`: shim, the 5 analog loops and control_io, built from the rev B schematic values. The analog section has two variants:
  - fast and behavioural (`pid_analog_fast.slx`)
  - a Simscape Electrical circuit (`pid_analog_circuit.slx`)
- **Bench harness** that mirrors the real board's test points, plus a validation suite.
- **Conservative first-flight trimpot settings** and per-axis mixer scales (`sim/gains/`), checked with both variants flying the quad.
- The original roll-axis model moved to `sim/legacy_roll_axis/`.

## PID_Board (through-hole)

### Rev B.1 (2026-10-26)
- U10 buck: the documented module changed from the Pololu D24V5F5 to a cheap fixed-5 V mini buck (Teyleten
  "3A Mini DCDC Buck", 5.5-30 V in), wired into the existing U10 header pins 2/3/4. No copper change.
- KNOWN_ISSUES: the power section is 2S-4S; for 6S, C25 needs a 35 V+ rating.

### Rev B (2026-10-25): compact frame-mount layout. Current version (tag `pid-tht-rev-b`)
Redesigned to mount on the 5" quad frame instead of sitting on the bench.
- **Size:** 250 x 100 mm became **90 x 210 mm**, a long narrow board that runs along the frame body.
- **Mounting:** holes on the frame's **30.5 x 30.5 stack** and **20 x 20 ESC stack** patterns. Each M3
  hole has a 3.4 mm copper-free ring for standoffs and screw heads. The 4in1 ESC sits underneath the rear
  section on the 20x20 holes.
- **Prop clearance:** every tall part (trimpots, DIP sockets, electrolytics, film boxes, TO-92s, headers,
  buck module) is outside the 5" prop disks, so all the trimpots can still be reached and adjusted on the quad.
- **IMU at the CG:** the GY-521 plugs into J2 with its MPU-6050 directly over the frame centre.
- **Floorplan, front to rear:**
  1. DAC shim, with the ELRS and UPDI headers on the front edge
  2. ROLL
  3. GY-521, with the REF panel and scope header beside it
  4. PITCH, laid out as the mirror image of roll
  5. YAW, control I/O and power, over the ESC
- **Tuning panels:** each axis has a labelled panel: a test-point row in signal-flow order, LOOP/EXT
  and GYRO/EXT jumpers, and Go/Kp/Ki/Kd trimpots.
- **4-layer stack-up:** In1 = solid GND plane, In2 = solid +5 V plane, signals on top and bottom.
- **Resistors:** still lying flat, but at 7.62 mm lead pitch instead of 10.16 mm.
- **ELRS header (J6):** moved to the front edge next to the shim MCU, so the CRSF line is short.
- **Routing:** FreeRouting 2.5, plus a grid router for the last connections. DRC: 0 errors,
  0 unconnected, schematic parity clean.
- **New files:** `MOUNTING.md` (stack, standoffs, receiver choice), `mounting_plan.png` (board drawn on the frame with
  the prop disks), a 3D render, 4-layer Gerbers and drill, grouped BOM, CPL, and a STEP model.

### Rev A (2026-10-25): tag `pid-tht-rev-a`
- Added R24 (ARM) and R25 (I_HOLD) 10k pulldowns so both lines are defined while the ATtinys boot.
- Silkscreen cleanup, board title, JLC order-number marker. JLCPCB/PCBWay fab export (2-layer).
- Archived in `PID_Board/old_versions/revA_250x100_2026-10-25/`.

### v1 (2026-10-20): tag `pid-tht-v1`
- First routed through-hole board: 250 x 100 mm, 2-layer. Shim (ATtiny1614, MCP4822 DACs, 74HC138)
  plus analog roll/pitch/yaw PID with trimpots, and control I/O to the 4in1 ESC.
- Archived in `PID_Board/old_versions/v1_250x100_2026-10-20/`.

## PID_Board_SMT
### v1 (2026-10-21)
- SMT, cost-reduced fork: digital pots (3x MCP4451), LMV324 op-amps, TPS54202 buck, 100 x 100 mm 2-layer,
  JLCPCB BOM/CPL with LCSC part numbers.
