"""Make sure the GND pours exist on both layers (create only if missing), then refill them.

Usage: KiCad python.exe pour.py <board.kicad_pcb> <size_mm>
(Removing and re-creating zones in one session crashes KiCad 10's python, so existing pours are kept.)
"""
import sys
import pcbnew

BOARD, S = sys.argv[1], float(sys.argv[2])
X0, Y0 = 20.0, 20.0
X1, Y1 = X0 + S, Y0 + S
mm = pcbnew.FromMM

b = pcbnew.LoadBoard(BOARD)
gnd = b.FindNet("GND")
have = {z.GetLayer() for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == gnd.GetNetCode()}
for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
    if layer in have:
        continue
    z = pcbnew.ZONE(b)
    z.SetLayer(layer)
    z.SetNet(gnd)
    ol = pcbnew.SHAPE_POLY_SET()
    ol.NewOutline()
    for x, y in ((X0 + 0.3, Y0 + 0.3), (X1 - 0.3, Y0 + 0.3), (X1 - 0.3, Y1 - 0.3), (X0 + 0.3, Y1 - 0.3)):
        ol.Append(mm(x), mm(y))
    z.SetOutline(ol)
    z.SetZoneName(f"GND_{'top' if layer == pcbnew.F_Cu else 'bottom'}")
    b.Add(z)
for z in b.Zones():
    if z.GetIsRuleArea():
        continue
    z.SetLocalClearance(mm(0.15))
    z.SetMinThickness(mm(0.15))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.2))
    z.SetThermalReliefSpokeWidth(mm(0.25))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(BOARD)
print("pours filled:", sorted(z.GetZoneName() for z in b.Zones() if not z.GetIsRuleArea()))
