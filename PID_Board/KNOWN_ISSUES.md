# PID_Board (THT) known issues / things to check

## Level shifters Q1, Q2, Q3 (2N7000) are marginal
- Q1 = SDA, Q2 = SCL (GY-521 3.3 V <-> ATtiny 5 V), Q3 = CRSF from the ELRS receiver.
- Gate bias (R2 10k / R4 22k from 5 V) is about 3.4 V. The 2N7000 max gate threshold Vgs(th) is 3 V
  (Diotec datasheet), so in the worst case the FET is barely on. Likely to work with 4.7k pull-ups,
  but I2C dropouts or missed CRSF packets are the symptom to watch for.
- Fallback: swap Q1-Q3 for a logic-level TO-92 FET such as ZVN3306A (same footprint, no board change).
  Verify its pin order is S-G-D before fitting (BS170 is reversed). Fit Q1-Q3 in 1x3 sockets so the swap is easy.

## Other items to verify before first power-up
- U10 (5 V buck module) wiring: header pin 1 unconnected, 2 VIN (after D1), 3 GND, 4 VOUT. With the
  Teyleten mini module, that is IN+ to 2, IN- to 3 and OUT+ to 4 (see MOUNTING.md).
- The power section is sized for 2S-4S. For 6S (25.2 V fully charged), C25 (47 uF 25 V after D1) needs a 35 V+ part (a 10 uF/50 V
  from the parts kit works), and the mini buck is rated "recommended within 28 V". That leaves little margin
  for motor-braking spikes.
- J5 ESC harness pin order against the Aero Selfie 4in1 30A.
- UPDI headers J4/J7 carry +5 V: do not connect a UPDI programmer while the battery is plugged in.
- Mechanical fit (see MOUNTING.md): the motor positions, the GY-521 chip offset and the ESC height
  are taken from the frame sketch and module listings, not measured.

## Rev B layout notes
- The tracks were routed by a script-driven grid router (FreeRouting 2.1 could not finish this
  board), then straightened to 45/90 degree segments. DRC is clean, but the routing looks
  machine-made, and a few cross-board signals run 100 mm or more. That is fine here: they are
  low-impedance DAC outputs, VGND rails and slow control lines, all over a solid GND plane. The
  longest high-impedance runs are the integrator reset lines to U9 (CD4066), which serves all three
  axes from the middle of the board.
- Hand soldering a 4-layer board: GND and +5 V pads connect to the inner planes through thermal
  reliefs. They still take a little longer to heat than signal pads, so use a hotter or wider tip on them.
- 4-layer at 90 x 210 mm costs more than the old 2-layer board at JLCPCB/PCBWay. A 2-layer version
  is possible at this size but would need GND pours and many more vias.

## Changes in rev B (frame-mount layout)
- Board reshaped from 250 x 100 mm to 90 x 210 mm to fit the 5" frame. Holes for the 30.5 stack and the
  20x20 ESC stack, each with a 3.4 mm copper-free ring for standoffs. Tall parts stay out of
  the prop disks.
- Floorplan in bands, front to rear: DAC shim, ROLL, GY-521 over the frame CG (+ REF panel, J1),
  PITCH (laid out as the mirror image of ROLL), and YAW + control I/O + power over the ESC. Each axis has a tuning
  panel: labelled test-point row, LOOP/EXT jumper, Go/Kp/Ki/Kd trimpots, GYRO/EXT jumper.
- 4 layers: In1.Cu GND plane, In2.Cu +5 V plane; signals on top and bottom.
- Resistor footprints changed from 10.16 mm to 7.62 mm lead pitch (still lying flat).
- ELRS header J6 moved to the front edge next to the shim MCU (short CRSF line).

## Changes in rev A
- Added R24 (ARM) and R25 (I_HOLD) 10k pulldowns so the motor-arm and integrator-hold lines are defined while the ATtinys boot.
- Silkscreen cleanup, board name text, JLCJLCJLCJLC marker on the back.
