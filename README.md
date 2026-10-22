# Quadcopter Flight Controller – PID Boards

Analog PID controller boards (roll / pitch / yaw + reference) for a quadcopter, designed in KiCad 10.

| Board | Description | Top render | Bottom render | BOM |
|---|---|---|---|---|
| `PID_Board/` | Through-hole version | `pcb_top_render.png` | `pcb_bottom_render.png` | `fab/PID_Board-bom.csv` |
| `PID_Board_SMT/` | SMT version with digital pots (JLCPCB-ready) | `pcb_top_render.png` | `pcb_bottom_render.png` | `fab/PID_Board_SMT-bom.csv`, `fab/PID_Board_SMT_BOM_JLCPCB.csv` |

Each board folder contains the KiCad project, Gerbers (`fab/`), and renders. `PID_Board_SMT/` also has the schematic PDF, notes, and build scripts. `sim/` holds the MATLAB roll-axis model and sweeps.

## Renders
### PID_Board (through-hole)
![top](PID_Board/pcb_top_render.png) ![bottom](PID_Board/pcb_bottom_render.png)
### PID_Board_SMT
![top](PID_Board_SMT/pcb_top_render.png) ![bottom](PID_Board_SMT/pcb_bottom_render.png)
