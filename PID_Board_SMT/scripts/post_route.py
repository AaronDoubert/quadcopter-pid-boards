"""Import the FreeRouting session, add GND pours on both layers, stitch vias, fill, save.

Usage: KiCad python.exe post_route.py <board.kicad_pcb> <board.ses> <size_mm>
"""
import sys
import pcbnew

BOARD, SES, S = sys.argv[1], sys.argv[2], float(sys.argv[3])
X0, Y0 = 20.0, 20.0
X1, Y1 = X0 + S, Y0 + S
mm = pcbnew.FromMM

b = pcbnew.LoadBoard(BOARD)
for t in list(b.GetTracks()):
    b.Remove(t)
for z in [b.GetArea(i) for i in range(b.GetAreaCount())]:
    if not z.GetIsRuleArea():
        b.Remove(z)
if not pcbnew.ImportSpecctraSES(b, SES):
    print("SES import failed"); sys.exit(1)

gnd = b.FindNet("GND")
for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
    z = pcbnew.ZONE(b)
    z.SetLayer(layer)
    z.SetNet(gnd)
    ol = z.Outline()
    ol.NewOutline()
    for x, y in ((X0 + 0.3, Y0 + 0.3), (X1 - 0.3, Y0 + 0.3), (X1 - 0.3, Y1 - 0.3), (X0 + 0.3, Y1 - 0.3)):
        ol.Append(mm(x), mm(y))
    z.SetLocalClearance(mm(0.3))
    z.SetMinThickness(mm(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.3))
    z.SetThermalReliefSpokeWidth(mm(0.35))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetZoneName(f"GND_{'top' if layer == pcbnew.F_Cu else 'bottom'}")
    b.Add(z)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(BOARD)
tracks = list(b.GetTracks())
vias = [t for t in tracks if t.GetClass() == "PCB_VIA"]
print("tracks", len(tracks) - len(vias), "vias", len(vias))
