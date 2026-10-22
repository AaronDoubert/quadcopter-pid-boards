"""Add gaprouter.py routes to the board (KiCad python.exe apply_routes.py <board> <routes.json>)."""
import json, sys
import pcbnew
B, R = sys.argv[1], sys.argv[2]
mm = pcbnew.FromMM
b = pcbnew.LoadBoard(B)
d = json.load(open(R))
nt = nv = 0
rip = {u for r in d["routes"] for u in r.get("ripup", [])}
gone = 0
for t in list(b.GetTracks()):
    if t.m_Uuid.AsString() in rip:
        b.Remove(t); gone += 1
if rip:
    print("ripped up", gone, "track segments")
for r in d["routes"]:
    net = b.FindNet(r["net"])
    for s in r["segs"]:
        if s[0] == "track":
            lay = pcbnew.F_Cu if s[1] == "F" else pcbnew.B_Cu
            for (ax, ay), (bx, by) in zip(s[2][:-1], s[2][1:]):
                t = pcbnew.PCB_TRACK(b)
                t.SetStart(pcbnew.VECTOR2I(mm(ax), mm(ay))); t.SetEnd(pcbnew.VECTOR2I(mm(bx), mm(by)))
                t.SetWidth(mm(r["width"])); t.SetLayer(lay); t.SetNet(net)
                b.Add(t); nt += 1
        else:
            v = pcbnew.PCB_VIA(b)
            v.SetPosition(pcbnew.VECTOR2I(mm(s[1]), mm(s[2]))); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3)); v.SetNet(net)
            b.Add(v); nv += 1
b.Save(B)
print("added tracks", nt, "vias", nv, "| failed", d["failed"])
