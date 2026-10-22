"""Tie every GND pour fragment on either layer to the other layer with at least one via.

Usage: KiCad python.exe stitch_islands.py <board.kicad_pcb>
For each filled-pour outline with no GND via or through-hole GND pad inside it, try via spots on a
0.3 mm grid inside the outline that sit in copper on BOTH pours and clear other nets; first fit wins.
"""
import sys
import pcbnew

B = sys.argv[1]
mm, to = pcbnew.FromMM, pcbnew.ToMM
b = pcbnew.LoadBoard(B)
gnd = b.FindNet("GND")
zones = {z.GetLayer(): z for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == gnd.GetNetCode()}
others = [t for t in b.GetTracks() if t.GetNetCode() != gnd.GetNetCode()]
others += [p for fp in b.GetFootprints() for p in fp.Pads() if p.GetNetCode() != gnd.GetNetCode()]
holes = [p for fp in b.GetFootprints() for p in fp.Pads() if p.HasHole()]
gvias = [t for t in b.GetTracks() if t.GetClass() == "PCB_VIA" and t.GetNetCode() == gnd.GetNetCode()]
gth = [p for p in holes if p.GetNetCode() == gnd.GetNetCode()]


def ok(v):
    for o in others:
        for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
            if o.IsOnLayer(lay) and o.GetEffectiveShape(lay).Collide(v.GetEffectiveShape(lay), mm(0.15)):
                return False
    for h in holes:
        if (h.GetPosition() - v.GetPosition()).EuclideanNorm() < max(h.GetDrillSize().x, h.GetDrillSize().y) / 2 + mm(0.65):
            return False
    for g in gvias:
        if (g.GetPosition() - v.GetPosition()).EuclideanNorm() < mm(0.9):
            return False
    return True


added = 0
for lay, z in zones.items():
    other = zones[pcbnew.B_Cu if lay == pcbnew.F_Cu else pcbnew.F_Cu]
    fill = z.GetFilledPolysList(lay)
    for k in range(fill.OutlineCount()):
        outline = fill.Outline(k)
        anchored = any(outline.PointInside(g.GetPosition()) for g in gvias + gth)
        if anchored:
            continue
        bb = outline.BBox()
        placed = False
        y = bb.GetTop() + mm(0.3)
        while y < bb.GetBottom() and not placed:
            x = bb.GetLeft() + mm(0.3)
            while x < bb.GetRight():
                p = pcbnew.VECTOR2I(int(x), int(y))
                if z.HitTestFilledArea(lay, p, -mm(0.35)) and other.HitTestFilledArea(other.GetLayer(), p, -mm(0.35)):
                    v = pcbnew.PCB_VIA(b)
                    v.SetPosition(p); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                    v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3)); v.SetNet(gnd)
                    if ok(v):
                        b.Add(v); gvias.append(v); added += 1; placed = True
                        break
                x += mm(0.3)
            y += mm(0.3)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(B)
print("island vias added", added)
