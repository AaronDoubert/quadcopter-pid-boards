import sys, pcbnew
B = sys.argv[1]; mm, to = pcbnew.FromMM, pcbnew.ToMM
b = pcbnew.LoadBoard(B)
eb = b.GetBoardEdgesBoundingBox(); L, T, R, Bo = eb.GetLeft() + mm(0.4), eb.GetTop() + mm(0.4), eb.GetRight() - mm(0.4), eb.GetBottom() - mm(0.4)
items = [t for t in b.GetDrawings() if t.GetClass() == "PCB_TEXT" and t.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS)]
items += [fp.Reference() for fp in b.GetFootprints() if fp.Reference().IsVisible()]
n = 0
for t in items:
    bb = t.GetBoundingBox(); dx = dy = 0
    if bb.GetLeft() < L: dx = L - bb.GetLeft()
    if bb.GetRight() > R: dx = R - bb.GetRight()
    if bb.GetTop() < T: dy = T - bb.GetTop()
    if bb.GetBottom() > Bo: dy = Bo - bb.GetBottom()
    if dx or dy:
        p = t.GetPosition(); t.SetPosition(pcbnew.VECTOR2I(p.x + dx, p.y + dy)); n += 1
        print("moved", t.GetText() if hasattr(t, "GetText") else "", round(to(dx), 2), round(to(dy), 2))
b.Save(B); print("texts pulled inside", n)
