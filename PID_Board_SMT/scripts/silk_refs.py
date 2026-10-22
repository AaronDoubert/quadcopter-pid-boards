"""Move footprint reference texts so they do not overlap each other, silkscreen lines or pads.

Usage: KiCad python.exe silk_refs.py <board.kicad_pcb>
Each visible reference tries: above / below / left / right of its courtyard (then the same at
a larger gap); the first spot clear of placed texts, pads and other courtyards' silk wins.
"""
import sys
import pcbnew

B = sys.argv[1]
mm, to = pcbnew.FromMM, pcbnew.ToMM
b = pcbnew.LoadBoard(B)
eb = b.GetBoardEdgesBoundingBox()
EX0, EY0, EX1, EY1 = to(eb.GetLeft()), to(eb.GetTop()), to(eb.GetRight()), to(eb.GetBottom())

obstacles = []          # pads and silk graphics of every footprint
for fp in b.GetFootprints():
    for pad in fp.Pads():
        bb = pad.GetBoundingBox()
        obstacles.append((to(bb.GetLeft()) - 0.15, to(bb.GetTop()) - 0.15, to(bb.GetRight()) + 0.15, to(bb.GetBottom()) + 0.15))
    for g in fp.GraphicalItems():
        if g.GetLayer() == pcbnew.F_SilkS and g.GetClass() != "PCB_FIELD":
            bb = g.GetBoundingBox()
            obstacles.append((to(bb.GetLeft()), to(bb.GetTop()), to(bb.GetRight()), to(bb.GetBottom())))
for d in b.GetDrawings():
    if d.GetLayer() == pcbnew.F_SilkS:
        bb = d.GetBoundingBox()
        obstacles.append((to(bb.GetLeft()), to(bb.GetTop()), to(bb.GetRight()), to(bb.GetBottom())))
for z in b.Zones():
    pass

placed = []


def hit(box, boxes):
    x1, y1, x2, y2 = box
    return any(not (x2 <= a or x1 >= c or y2 <= bb or y1 >= d) for a, bb, c, d in boxes)


moved = hidden = 0
fps = sorted(b.GetFootprints(), key=lambda f: -len(f.Pads()))
for fp in fps:
    t = fp.Reference()
    if not t.IsVisible():
        continue
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    cb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
    l, tp, r, bt = to(cb.GetLeft()), to(cb.GetTop()), to(cb.GetRight()), to(cb.GetBottom())
    t.SetTextAngleDegrees(0)
    tb = t.GetBoundingBox()
    w, h = to(tb.GetWidth()), to(tb.GetHeight())
    cxm, cym = (l + r) / 2, (tp + bt) / 2
    cands = []
    for gap in (0.15, 0.6, 1.2):
        cands += [(cxm, tp - gap - h / 2), (cxm, bt + gap + h / 2), (l - gap - w / 2, cym), (r + gap + w / 2, cym)]
    cur = to(t.GetPosition().x), to(t.GetPosition().y)
    cands.insert(0, cur)
    for x, y in cands:
        box = (x - w / 2, y - h / 2, x + w / 2, y + h / 2)
        if box[0] < EX0 + 0.3 or box[1] < EY0 + 0.3 or box[2] > EX1 - 0.3 or box[3] > EY1 - 0.3:
            continue
        if not hit(box, placed) and not hit(box, obstacles):
            if (x, y) != cur:
                t.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y))); moved += 1
            placed.append(box)
            break
    else:
        t.SetVisible(False); hidden += 1     # no clear spot: keep it on the fab layer only (assembly drawing)
b.Save(B)
print("moved", moved, "hidden (no room)", hidden)
