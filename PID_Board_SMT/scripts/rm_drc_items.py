"""Delete vias/tracks that DRC flags as dangling or as too close to a hole (positions from drc.json)."""
import json, sys
import pcbnew
B, DRC = sys.argv[1], sys.argv[2]
mm, to = pcbnew.FromMM, pcbnew.ToMM
b = pcbnew.LoadBoard(B)
d = json.load(open(DRC))
want = []
for v in d["violations"]:
    if v["type"] in ("hole_clearance", "holes_co_located", "track_dangling", "via_dangling"):
        it = v["items"][0]
        if it["description"].startswith(("Via", "Track")):
            want.append((it["description"].split(" on ")[0], it["pos"]["x"], it["pos"]["y"]))
gone = 0
for t in list(b.GetTracks()):
    kind = "Via" if t.GetClass() == "PCB_VIA" else "Track"
    for desc, x, y in want:
        if not desc.startswith(kind) or f"[{t.GetNetname()}]" not in desc:
            continue
        p = t.GetPosition() if kind == "Via" else None
        hit = (kind == "Via" and abs(to(p.x) - x) < 0.01 and abs(to(p.y) - y) < 0.01) or \
              (kind == "Track" and t.HitTest(pcbnew.VECTOR2I(mm(x), mm(y)), mm(0.01)) and to(t.GetLength()) < 3)
        if hit:
            b.Remove(t); gone += 1
            break
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(B)
print("removed", gone, "of", len(want))
