"""Dump copper geometry + the unconnected signal pairs for gaprouter.py (run with KiCad's python.exe).

Usage: python.exe export_geom.py <board.kicad_pcb> <drc.json> <out.json>
Each copper item: net, layers, raw outline polygons (for source/target marking) per layer.
Pairs: for every non-GND unconnected item pair in DRC, the item indices of the two copper islands.
"""
import json, sys
import pcbnew

B, DRC, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
to, mm = pcbnew.ToMM, pcbnew.FromMM
b = pcbnew.LoadBoard(B)
LAYERS = {pcbnew.F_Cu: "F", pcbnew.B_Cu: "B"}

items = []          # (pcb item, dict)
for t in b.GetTracks():
    items.append(t)
for fp in b.GetFootprints():
    for p in fp.Pads():
        items.append(p)


def polys(it, layer, inflate):
    ps = pcbnew.SHAPE_POLY_SET()
    it.TransformShapeToPolygon(ps, layer, mm(inflate), mm(0.01), pcbnew.ERROR_OUTSIDE)
    out = []
    for i in range(ps.OutlineCount()):
        ol = ps.Outline(i)
        out.append([(round(to(ol.CPoint(j).x), 4), round(to(ol.CPoint(j).y), 4)) for j in range(ol.PointCount())])
    return out


data = []
for it in items:
    lays = [n for l, n in LAYERS.items() if it.IsOnLayer(l)]
    if not lays:
        continue
    d = {"net": it.GetNetname(), "kind": it.GetClass(), "layers": lays, "raw": {}}
    if it.GetClass() == "PCB_TRACK":
        d["uuid"] = it.m_Uuid.AsString()
    for l, n in LAYERS.items():
        if n in lays:
            d["raw"][n] = polys(it, l, 0.0)
    if it.GetClass() == "PAD" and it.HasHole():
        d["hole"] = [to(it.GetPosition().x), to(it.GetPosition().y), to(max(it.GetDrillSize().x, it.GetDrillSize().y)) / 2]
    data.append(d)

# copper islands per net (union-find over touching same-net items)
parent = list(range(len(items)))


def find(i):
    while parent[i] != i:
        parent[i] = parent[parent[i]]
        i = parent[i]
    return i


bynet = {}
idx = [i for i, it in enumerate(items) if any(it.IsOnLayer(l) for l in LAYERS)]
for i in idx:
    bynet.setdefault(items[i].GetNetCode(), []).append(i)
for net, ids in bynet.items():
    if net == 0:
        continue
    for a in range(len(ids)):
        for c in range(a + 1, len(ids)):
            i, j = ids[a], ids[c]
            for l in LAYERS:
                if items[i].IsOnLayer(l) and items[j].IsOnLayer(l):
                    if items[i].GetEffectiveShape(l).Collide(items[j].GetEffectiveShape(l), 0):
                        parent[find(i)] = find(j)
                        break
pos_index = {i: n for n, i in enumerate(idx)}

drc = json.load(open(DRC))
pairs = []
seen = set()
for x in drc.get("unconnected_items", []):
    its = x["items"]
    net = its[0]["description"].split("[")[1].split("]")[0]
    if net == "GND" or len(its) != 2:
        continue
    code = b.FindNet(net).GetNetCode()
    roots = []
    for it in its:
        p = pcbnew.VECTOR2I(mm(it["pos"]["x"]), mm(it["pos"]["y"]))
        best = None
        for i in bynet[code]:
            if any(items[i].IsOnLayer(l) and items[i].GetEffectiveShape(l).Collide(p, mm(0.01)) for l in LAYERS):
                best = i
                break
        roots.append(find(best) if best is not None else None)
    if None in roots or roots[0] == roots[1] or (net, frozenset(roots)) in seen:
        continue
    seen.add((net, frozenset(roots)))
    A = [pos_index[i] for i in bynet[code] if find(i) == roots[0]]
    Bc = [pos_index[i] for i in bynet[code] if find(i) == roots[1]]
    pairs.append({"net": net, "a": A, "b": Bc})

# GND pads stranded from the pour: target = every other GND item plus the pour fill
gcode = b.FindNet("GND").GetNetCode()
zone_ids = []
for z in b.Zones():
    if z.GetIsRuleArea() or z.GetNetCode() != gcode:
        continue
    for l, n in LAYERS.items():
        if not z.IsOnLayer(l):
            continue
        fp_ = z.GetFilledPolysList(l)
        polys_, holes_ = [], []
        for i in range(fp_.OutlineCount()):
            ol = fp_.Outline(i)
            polys_.append([(round(to(ol.CPoint(j).x), 4), round(to(ol.CPoint(j).y), 4)) for j in range(ol.PointCount())])
            for h in range(fp_.HoleCount(i)):
                hl = fp_.Hole(i, h)
                holes_.append([(round(to(hl.CPoint(j).x), 4), round(to(hl.CPoint(j).y), 4)) for j in range(hl.PointCount())])
        data.append({"net": "GND", "kind": "ZONE", "layers": [n], "raw": {n: polys_}, "holes": {n: holes_}, "fill": True})
        zone_ids.append(len(data) - 1)
stranded = set()
for x in drc.get("unconnected_items", []):
    for it in x["items"]:
        if "[GND]" in it["description"] and it["description"].startswith("Pad"):
            stranded.add((round(it["pos"]["x"], 3), round(it["pos"]["y"], 3)))
for i in bynet.get(gcode, []):
    it = items[i]
    if it.GetClass() != "PAD":
        continue
    k = (round(to(it.GetPosition().x), 3), round(to(it.GetPosition().y), 3))
    if k not in stranded:
        continue
    A = [pos_index[j] for j in bynet[gcode] if find(j) == find(i)]
    Bc = [pos_index[j] for j in bynet[gcode] if find(j) != find(i)] + zone_ids
    pairs.append({"net": "GND", "a": A, "b": Bc, "pad": k})

# pour fragments with no via / GND through-hole inside them (and not the main pour): connect each one
if "--fragments" in sys.argv:
    anchors = [t.GetPosition() for t in b.GetTracks() if t.GetClass() == "PCB_VIA" and t.GetNetCode() == gcode]
    anchors += [p.GetPosition() for fp in b.GetFootprints() for p in fp.Pads() if p.GetNetCode() == gcode and p.HasHole()]
    for z in b.Zones():
        if z.GetIsRuleArea() or z.GetNetCode() != gcode:
            continue
        l = z.GetLayer(); n = LAYERS[l]
        fill = z.GetFilledPolysList(l)
        areas = [fill.Outline(i).Area() for i in range(fill.OutlineCount())]
        main = areas.index(max(areas)) if areas else -1
        for i in range(fill.OutlineCount()):
            ol = fill.Outline(i)
            if i == main or any(ol.PointInside(a_) for a_ in anchors):
                continue
            poly = [(round(to(ol.CPoint(j).x), 4), round(to(ol.CPoint(j).y), 4)) for j in range(ol.PointCount())]
            data.append({"net": "GND", "kind": "FRAG", "layers": [n], "raw": {n: [poly]}, "fill": True})
            fid = len(data) - 1
            touching = {find(j) for j in bynet[gcode] if items[j].IsOnLayer(l)
                        and any(ol.PointInside(items[j].GetPosition()) or ol.Collide(items[j].GetPosition(), mm(0.3)) for _ in [0])}
            others_ = [pos_index[j] for j in bynet[gcode] if find(j) not in touching]
            others_ += [k for k in zone_ids if data[k]["layers"] != [n]]   # the other layer's pour only
            pairs.append({"net": "GND", "a": [fid], "b": others_, "frag": True})

# GND clusters (items + pour islands) cut off from the main ground: route each one to the rest
if "--loose" in sys.argv:
    gitems = [i for i in idx if items[i].GetNetCode() == gcode]
    isl = []
    for z in b.Zones():
        if z.GetIsRuleArea() or z.GetNetCode() != gcode:
            continue
        l = z.GetLayer(); f = z.GetFilledPolysList(l)
        for k in range(f.OutlineCount()):
            isl.append((l, f.Outline(k)))
    N = len(gitems) + len(isl)
    par2 = list(range(N))
    def f2(i):
        while par2[i] != i:
            par2[i] = par2[par2[i]]; i = par2[i]
        return i
    for a_ in range(len(gitems)):
        for c_ in range(a_ + 1, len(gitems)):
            if find(gitems[a_]) == find(gitems[c_]):
                par2[f2(a_)] = f2(c_)
    for k, (l, ol) in enumerate(isl):
        for a_, i in enumerate(gitems):
            it = items[i]
            if not it.IsOnLayer(l):
                continue
            pts = [it.GetStart(), it.GetEnd()] if it.GetClass() == "PCB_TRACK" else [it.GetPosition()]
            if any(ol.PointInside(p) or ol.Collide(p, mm(0.05)) for p in pts):
                par2[f2(len(gitems) + k)] = f2(a_)
    comp = {}
    for n_ in range(N):
        comp.setdefault(f2(n_), []).append(n_)
    main = max(comp, key=lambda r: len(comp[r]))
    isl_data = []
    for k, (l, ol) in enumerate(isl):
        poly = [(round(to(ol.CPoint(j).x), 4), round(to(ol.CPoint(j).y), 4)) for j in range(ol.PointCount())]
        data.append({"net": "GND", "kind": "ISLAND", "layers": [LAYERS[l]], "raw": {LAYERS[l]: [poly]}, "fill": True})
        isl_data.append(len(data) - 1)
    for r, mem in comp.items():
        if r == main:
            continue
        if any(m < len(gitems) and items[gitems[m]].GetClass() == "PAD" and items[gitems[m]].HasHole() for m in mem):
            continue                    # through-hole pads join both pours via thermals (not visible to this check)
        if not any(m >= len(gitems) for m in mem) and not any(m < len(gitems) and items[gitems[m]].GetClass() == "PAD" for m in mem):
            continue
        A = [pos_index[gitems[m]] if m < len(gitems) else isl_data[m - len(gitems)] for m in mem]
        Bc = [pos_index[gitems[m]] if m < len(gitems) else isl_data[m - len(gitems)] for m in comp[main]]
        pairs.append({"net": "GND", "a": A, "b": Bc, "loose": True})

eb = b.GetBoardEdgesBoundingBox()
keep = []
for z in b.Zones():
    if z.GetIsRuleArea():
        ol = z.Outline().Outline(0)
        keep.append([(to(ol.CPoint(j).x), to(ol.CPoint(j).y)) for j in range(ol.PointCount())])
json.dump({"edge": [to(eb.GetLeft()), to(eb.GetTop()), to(eb.GetRight()), to(eb.GetBottom())],
           "items": data, "pairs": pairs, "keepouts": keep}, open(OUT, "w"))
print("items", len(data), "pairs", len(pairs), [p["net"] for p in pairs])
