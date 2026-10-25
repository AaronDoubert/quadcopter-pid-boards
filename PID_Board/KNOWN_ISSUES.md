# PID_Board (THT) known issues / things to check

## Level shifters Q1, Q2, Q3 (2N7000) are marginal
- Q1 = SDA, Q2 = SCL (GY-521 3.3 V <-> ATtiny 5 V), Q3 = CRSF from the ELRS receiver.
- Gate bias (R2 10k / R4 22k from 5 V) is about 3.4 V. The 2N7000 max gate threshold Vgs(th) is 3 V
  (Diotec datasheet), so in the worst case the FET is barely on. Likely to work with 4.7k pull-ups,
  but I2C dropouts or missed CRSF packets are the symptom to watch for.
- Fallback: swap Q1-Q3 for a logic-level TO-92 FET such as ZVN3306A (same footprint, no board change).
  Verify its pin order is S-G-D before fitting (BS170 is reversed). Fit Q1-Q3 in 1x3 sockets so the swap is easy.

## Other items to verify before first power-up
- Pololu D24V5F5 (U10) pin order: header is 1 unconnected, 2 VIN (after D1), 3 GND, 4 VOUT.
- J5 ESC harness pin order against the Aero Selfie 4in1 30A.
- UPDI headers J4/J7 carry +5 V: do not connect a UPDI programmer while the battery is plugged in.
- 7 starved-thermal DRC warnings on GND pads (connected, cosmetic).

## Changes in this revision
- Added R24 (ARM) and R25 (I_HOLD) 10k pulldowns so the motor-arm and integrator-hold lines are defined while the ATtinys boot.
- Silkscreen cleanup, board name text, JLCJLCJLCJLC marker on the back.
