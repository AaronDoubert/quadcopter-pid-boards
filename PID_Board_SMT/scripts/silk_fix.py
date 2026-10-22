"""Resolve the silkscreen DRC warnings left after labelling (KiCad python.exe silk_fix.py <board> <drc.json>)."""
import json, re, sys
import pcbnew
B, DRC = sys.argv[1], sys.argv[2]
mm, to = pcbnew.FromMM, pcbnew.ToMM
b = pcbnew.LoadBoard(B)
d = json.load(open(DRC))
flag = set()
for v in d["violations"]:
    if v["type"].startswith("silk"):
        for it in v["items"]:
            m = re.match(r"PCB text '([^']*)'", it["description"])
            if m:
                flag.add((m.group(1), round(it["pos"]["x"], 2), round(it["pos"]["y"], 2)))
texts = [t for t in b.GetDrawings() if t.GetClass() == "PCB_TEXT" and t.GetLayer() == pcbnew.F_SilkS]
pads = []
for fp in b.GetFootprints():
    for p in fp.Pads():
        bb = p.GetBoundingBox(); pads.append((to(bb.GetLeft()) - 0.2, to(bb.GetTop()) - 0.2, to(bb.GetRight()) + 0.2, to(bb.GetBottom()) + 0.2))
    for g in fp.GraphicalItems():
        if g.GetLayer() == pcbnew.F_SilkS and g.GetClass() != "PCB_FIELD":
            bb = g.GetBoundingBox(); pads.append((to(bb.GetLeft()), to(bb.GetTop()), to(bb.GetRight()), to(bb.GetBottom())))
eb = b.GetBoardEdgesBoundingBox()
E = (to(eb.GetLeft()) + 0.5, to(eb.GetTop()) + 0.5, to(eb.GetRight()) - 0.5, to(eb.GetBottom()) - 0.5)
def box(t):
    bb = t.GetBoundingBox(); return (to(bb.GetLeft()), to(bb.GetTop()), to(bb.GetRight()), to(bb.GetBottom()))
def clear(bx, me):
    if bx[0] < E[0] or bx[1] < E[1] or bx[2] > E[2] or bx[3] > E[3]:
        return False
    others = pads + [box(o) for o in texts if o is not me]
    return not any(not (bx[2] <= a or bx[0] >= c or bx[3] <= e or bx[1] >= f) for a, e, c, f in others)
removed = moved = 0
titled = re.compile(r"^(J\d+|JP\d+) ")
for t in list(texts):
    key = (t.GetText(), round(to(t.GetPosition().x), 2), round(to(t.GetPosition().y), 2))
    if key not in flag:
        continue
    if titled.match(t.GetText()):
        ref = t.GetText().split()[0]
        fp = b.FindFootprintByReference(ref)
        if fp:
            fp.Reference().SetVisible(True)
        b.Remove(t); texts.remove(t); removed += 1
        continue
    if t.GetText() == "ESC cable":
        p = t.GetPosition(); t.SetPosition(pcbnew.VECTOR2I(p.x, p.y - mm(6.0))); moved += 1
        continue
    p0 = t.GetPosition()
    for dx, dy in [(0, -0.6), (0, 0.6), (-0.6, 0), (0.6, 0), (0, -1.2), (0, 1.2), (-1.2, 0), (1.2, 0), (0.8, 0.8), (-0.8, -0.8), (0.8, -0.8), (-0.8, 0.8)]:
        t.SetPosition(pcbnew.VECTOR2I(p0.x + mm(dx), p0.y + mm(dy)))
        if clear(box(t), t):
            moved += 1
            break
    else:
        t.SetPosition(p0)
b.Save(B)
print("titles removed", removed, "texts moved", moved)
