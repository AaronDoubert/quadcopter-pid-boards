# PID_Board (THT) rev B: mounting on the 5" frame

Board: 90 x 210 mm, 4-layer (In1 = GND plane, In2 = +5 V plane). All coordinates are from the frame's
centre mark: the small hole midway between the motors in the frame sketch. +y points toward the 20x20 holes, called "rear" here.
In KiCad the frame centre is at (100, 100); the board edge runs from x 55 to 145 and y -5 to 205.
The board overhangs the frame plate by about 30 mm at each end.

## Holes (all 3.2 mm, M3)
| Pair | Frame position | Spacing | Use |
|---|---|---|---|
| H1/H2 | y = -24.9 | 30.5 | front of the 30.5 stack pattern: standoffs straight to the frame plate |
| H3/H4 | y = +5.6 | 30.5 | rear of the 30.5 pattern |
| H5/H6 | y = +47.1 | 20 | 20x20 ESC stack, front row |
| H7/H8 | y = +67.1 | 20 | 20x20 ESC stack, rear row |

Each hole has a 3.4 mm radius copper-free ring on the outer layers, so the standoffs, nuts and screw heads
never sit on a trace. The 25 mm pair (y = -46.8) and the 28 mm pair (y = +18.1) on the frame are not
used. The board does not need them.

## Stack (bottom to top)
1. Frame plate.
2. 4in1 ESC on the 20x20 holes (soft-mount grommets).
3. PID_Board on 8 M3 standoffs. All 8 are the same length, set by the ESC: ESC height + about 4 mm for
   the THT leads under the board. The ESC sits under the rear section (y +42 to +72) and is outlined
   on the back silkscreen.
4. On top of the board, plugged into headers: the GY-521 (J2) and the 5 V buck module (U10). The
   ELRS receiver connects by wires to J6 at the front edge.

## What sits where
* **Tall parts** (trimpots, DIP sockets, electrolytics, film boxes, TO-92s, headers, the buck module) stay
  out of the 5" prop disks (63.5 mm radius + 3 mm margin). Only low parts (flat resistors, DO-35
  diodes, disc caps, thin film caps) are near the board corners, which sit under the prop disks.
* **Bands, front to rear:**
  1. DAC shim: the ATtiny, 74HC138, 5x MCP4822, level shifters, J4 UPDI and J6 ELRS.
  2. ROLL PID with its tuning panel.
  3. GY-521 over the frame centre, with the REF panel (VSET trimpot, VGND test points) on its right and
     J1, the scope header, on the left edge.
  4. PITCH PID, laid out as the mirror image of roll.
  5. Over the ESC: yaw PID, control-I/O ATtiny, U10 buck and the YAW panel. J5 (ESC harness) and
     J7 (UPDI2) are on the rear edge.
* **Tuning panels.** Each panel is a row of test points in signal-flow order, then
  SP jumper, Go/Kp/Ki/Kd trimpots and FB jumper. The labels are on the silkscreen.
* **GY-521** plugs into J2 (8-pin female header, pins toward the rear) and lies flat over the
  board. Its MPU-6050 sits over the frame centre mark (assumed 7 mm in front of the header row; check
  this against your module). Only low parts are under it. A pad of foam tape between the module and
  the board helps with vibration.
* **Power.** The ESC harness goes to J5 on the rear edge (GND, VBAT, M1-M4). VBAT then goes through
  D1 (1N5817) to U10, a small fixed-5 V buck module next to the four ESC holes. Its 5 V output feeds
  the +5 V plane.
  * **Module used:** Teyleten Robot "3A Mini DCDC Buck" 5 V fixed, 5.5-30 V in, 10-pack, Amazon
    B0DKTMGBHL. Its pads are four castellated corners: IN+ and IN- at one end, OUT+ and OUT- at the other
    (IN- and OUT- are both ground).
  * **Wiring (no board change):** U10 is a 1x4 header footprint. Solder three short wires into it:
    module IN+ to pin 2 (VIN), IN- to pin 3 (GND), OUT+ to pin 4 (+5 V). Pin 1 stays empty. Stick the
    module flat to the board beside the header with foam tape.
  * Check 5.0 V on pin 4 with a meter before plugging in the ICs, the GY-521 or the receiver.
  * Any 3-pin "7805 replacement" (IN-GND-OUT) or a Pololu D24V5F5 also fits straight into pins 2-4.
* **ELRS receiver.** A RadioMaster RP1 V2 or RP2, or any ELRS 2.4 GHz nano receiver with CRSF
  output, works with the T8L. Solder it to J6 (5V, GND, CRSF) with short wires and stick it under the
  front overhang. Run the antenna up and back along a standoff, away from the ESC and the carbon. Only the
  receiver's TX pin is used (CRSF into the shim ATtiny through Q3); its RX pin stays unconnected.

## Check before ordering
* Wheelbase: motors assumed at (+-87, +-74.5) mm, from the sketch dimensions.
* GY-521 chip offset from the header row (7 mm assumed).
* ESC height, to pick the standoff length.
* Which end is the front: the FRONT arrow on the silkscreen assumes the 30.5 pattern faces forward.
