"""Connect GND pads that the pours leave stranded, then stitch the two GND pours together.

Usage: KiCad python.exe stitch_gnd.py <board.kicad_pcb> <drc.json>

For every GND pad DRC lists as unconnected, try a via at 8 directions x 3 distances around the pad,
joined to it by a 0.3 mm F.Cu track; keep the first candidate that clears every other-net item by
0.2 mm. Then drop a grid of stitching vias (4 mm pitch) wherever both pours have copper.
"""
import json, math, sys
import pcbnew

B, DRC = sys.argv[1], sys.argv[2]
mm, to = pcbnew.FromMM, pcbnew.ToMM
b = pcbnew.LoadBoard(B)
gnd = b.FindNet("GND")
CLR = mm(0.2)
eb = b.GetBoardEdgesBoundingBox()
EX0, EY0, EX1, EY1 = eb.GetLeft(), eb.GetTop(), eb.GetRight(), eb.GetBottom()


def others():
    items = []
    for t in b.GetTracks():
        if t.GetNetCode() != gnd.GetNetCode():
            items.append(t)
    for fp in b.GetFootprints():
        for p in fp.Pads():
            if p.GetNetCode() != gnd.GetNetCode():
                items.append(p)
    return items


OTHERS = others()
HOLES = [p for fp in b.GetFootprints() for p in fp.Pads() if p.HasHole()]
KEEP = [z for z in b.Zones() if z.GetIsRuleArea()]


def clear(item, layers):
    if any(z.Outline().Collide(item.GetPosition(), mm(1.0)) for z in KEEP):
        return False
    bb = item.GetBoundingBox()
    if bb.GetLeft() < EX0 + mm(0.4) or bb.GetTop() < EY0 + mm(0.4) or bb.GetRight() > EX1 - mm(0.4) or bb.GetBottom() > EY1 - mm(0.4):
        return False
    for o in OTHERS:
        for lay in layers:
            if not o.IsOnLayer(lay):
                continue
            if o.GetEffectiveShape(lay).Collide(item.GetEffectiveShape(lay), CLR):
                return False
    if item.GetClass() == "PCB_VIA":
        for h in HOLES:
            r = max(h.GetDrillSize().x, h.GetDrillSize().y) / 2
            if (h.GetPosition() - item.GetPosition()).EuclideanNorm() < r + mm(0.3 + 0.35):
                return False
        for v in b.GetTracks():
            if v.GetClass() == "PCB_VIA" and (v.GetPosition() - item.GetPosition()).EuclideanNorm() < mm(0.9):
                return False
    return True


def new_via(x, y):
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(x, y)); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3)); v.SetNet(gnd)
    return v


d = json.load(open(DRC))
targets = set()
for x in d.get("unconnected_items", []):
    for it in x["items"]:
        if "[GND]" in it["description"] and it["description"].startswith("Pad"):
            targets.add((round(it["pos"]["x"], 3), round(it["pos"]["y"], 3)))
pads = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        if p.GetNetCode() == gnd.GetNetCode():
            pads[(round(to(p.GetPosition().x), 3), round(to(p.GetPosition().y), 3))] = p

fixed, failed = 0, []
for key in sorted(targets):
    pad = pads.get(key)
    if pad is None:
        failed.append(key); continue
    if pad.HasHole():
        continue            # through-hole pads reach both pours already
    c = pad.GetPosition()
    r0 = max(to(pad.GetBoundingBox().GetWidth()), to(pad.GetBoundingBox().GetHeight())) / 2
    done = False
    for dist in (r0 + 0.55, r0 + 0.9, r0 + 1.4):
        for k in range(8):
            a = math.pi / 4 * k
            vx, vy = c.x + mm(dist * math.cos(a)), c.y + mm(dist * math.sin(a))
            via = new_via(vx, vy)
            tr = pcbnew.PCB_TRACK(b)
            tr.SetStart(c); tr.SetEnd(pcbnew.VECTOR2I(vx, vy)); tr.SetWidth(mm(0.3)); tr.SetLayer(pcbnew.F_Cu); tr.SetNet(gnd)
            if clear(via, (pcbnew.F_Cu, pcbnew.B_Cu)) and clear(tr, (pcbnew.F_Cu,)):
                b.Add(via); b.Add(tr); OTHERS  # GND items are not obstacles for later GND vias
                fixed += 1; done = True
                break
        if done:
            break
    if not done:
        failed.append((key, pad.GetParentFootprint().GetReference(), pad.GetNumber()))

# stitching grid between the two pours
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
zones = {z.GetLayer(): z for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == gnd.GetNetCode()}
stitched = 0
step = mm(4.0)
y = EY0 + mm(2.0)
while y < EY1 - mm(1.5):
    x = EX0 + mm(2.0)
    while x < EX1 - mm(1.5):
        p = pcbnew.VECTOR2I(int(x), int(y))
        if all(z.HitTestFilledArea(lay, p, mm(0.45)) for lay, z in zones.items()):
            v = new_via(int(x), int(y))
            if clear(v, (pcbnew.F_Cu, pcbnew.B_Cu)):
                b.Add(v); stitched += 1
        x += step
    y += step
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(B)
print("GND pads fixed", fixed, "| stitching vias", stitched, "| could not fix", failed)
