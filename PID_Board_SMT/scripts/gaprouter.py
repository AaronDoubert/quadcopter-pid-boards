"""Tiny two-layer grid router for the few connections FreeRouting leaves open.

Usage: venv python gaprouter.py <geom.json> <routes.json>
0.1 mm grid, 8-direction moves on F/B plus vias. Other nets' copper is inflated by
clearance + half the new track width (tracks) or + via radius (vias); board edge and rule areas
are blocked. Routes join two copper islands of one net; each new route becomes an obstacle for
the next.
"""
import heapq, json, math, sys
import numpy as np
from matplotlib.path import Path

G = json.load(open(sys.argv[1]))
RES = 0.1
CLR = 0.15
TRACK_W = {"default": 0.2, "+5V": 0.3, "GND": 0.2}
VIA_R = 0.3
X0, Y0, X1, Y1 = G["edge"]
NX, NY = int(round((X1 - X0) / RES)) + 1, int(round((Y1 - Y0) / RES)) + 1
xs = X0 + np.arange(NX) * RES
ys = Y0 + np.arange(NY) * RES
LAY = {"F": 0, "B": 1}


def raster(poly, inflate):
    """Boolean mask (NY, NX) of cells within `inflate` of polygon `poly` (list of (x, y))."""
    p = np.asarray(poly)
    x0, y0 = p.min(0) - inflate - RES
    x1, y1 = p.max(0) + inflate + RES
    i0, i1 = max(int((x0 - X0) / RES), 0), min(int((x1 - X0) / RES) + 1, NX)
    j0, j1 = max(int((y0 - Y0) / RES), 0), min(int((y1 - Y0) / RES) + 1, NY)
    if i0 >= i1 or j0 >= j1:
        return None
    gx, gy = np.meshgrid(xs[i0:i1], ys[j0:j1])
    pts = np.column_stack([gx.ravel(), gy.ravel()])
    inside = Path(p).contains_points(pts, radius=0.0).reshape(gx.shape)
    if inflate > 0:
        # distance from each cell to the polygon edges
        d = np.full(gx.shape, np.inf)
        q = np.vstack([p, p[:1]])
        for (ax, ay), (bx, by) in zip(q[:-1], q[1:]):
            vx, vy = bx - ax, by - ay
            L = vx * vx + vy * vy or 1e-12
            t = np.clip(((gx - ax) * vx + (gy - ay) * vy) / L, 0, 1)
            d = np.minimum(d, np.hypot(gx - (ax + t * vx), gy - (ay + t * vy)))
        inside |= d <= inflate
    m = np.zeros((NY, NX), bool)
    m[j0:j1, i0:i1] = inside
    return m


def seg_poly(a, b, w):
    """Rectangle (as polygon) around segment a-b of width w (ends squared off; inflate rounds them)."""
    (ax, ay), (bx, by) = a, b
    L = math.hypot(bx - ax, by - ay) or 1e-9
    nx, ny = -(by - ay) / L * w / 2, (bx - ax) / L * w / 2
    return [(ax + nx, ay + ny), (bx + nx, by + ny), (bx - nx, by - ny), (ax - nx, ay - ny)]


# static masks per item and layer, built lazily
items = G["items"]
added = []      # (net, layer, polygon) of routes placed by this script


RIPUP = "--ripup" in sys.argv
SOFT = []       # (item index, layer, track mask, via mask) of rip-up-able tracks


def blocked(net, w):
    tr = np.zeros((2, NY, NX), bool)
    via = np.zeros((NY, NX), bool)
    infl_t = CLR + w / 2 + 0.03
    infl_v = CLR + VIA_R + 0.03
    SOFT.clear()
    for k, it in enumerate(items):
        if (it["net"] == net and it["net"] != "") or it.get("fill"):
            continue
        if RIPUP and it["kind"] == "PCB_TRACK":
            for ln, polys in it["raw"].items():
                for poly in polys:
                    mt, mv = raster(poly, infl_t), raster(poly, infl_v)
                    if mt is not None:
                        SOFT.append((k, LAY[ln], mt, mv))
            continue
        for ln, polys in it["raw"].items():
            for poly in polys:
                m = raster(poly, infl_t)
                if m is not None:
                    tr[LAY[ln]] |= m
                m = raster(poly, infl_v)
                if m is not None:
                    via |= m
        if "hole" in it:
            hx, hy, hr = it["hole"]
            m = raster([(hx - hr, hy - hr), (hx + hr, hy - hr), (hx + hr, hy + hr), (hx - hr, hy + hr)], 0.25 + VIA_R)
            via |= m
    for anet, ln, poly in added:
        if anet == net:
            continue
        m = raster(poly, infl_t)
        tr[LAY[ln]] |= m
        via |= raster(poly, infl_v)
    edge_t = int(math.ceil((0.3 + w / 2) / RES)) + 1
    edge_v = int(math.ceil((0.3 + VIA_R) / RES)) + 1
    for k in (0, 1):
        tr[k, :edge_t, :] = tr[k, -edge_t:, :] = True
        tr[k, :, :edge_t] = tr[k, :, -edge_t:] = True
    via[:edge_v, :] = via[-edge_v:, :] = True
    via[:, :edge_v] = via[:, -edge_v:] = True
    for ko in G["keepouts"]:
        m = raster(ko, w / 2 + 0.05)
        tr[0] |= m; tr[1] |= m; via |= raster(ko, VIA_R + 0.05)
    via |= tr[0] | tr[1]
    return tr, via


def island_mask(ids):
    m = np.zeros((2, NY, NX), bool)
    for i in ids:
        lm = np.zeros((2, NY, NX), bool)
        for ln, polys in items[i]["raw"].items():
            for poly in polys:
                r = raster(poly, -0.0)
                if r is not None:
                    lm[LAY[ln]] |= r
        for ln, hs in items[i].get("holes", {}).items():
            for poly in hs:
                r = raster(poly, -0.0)
                if r is not None:
                    lm[LAY[ln]] &= ~r
        m |= lm
    return m


MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
VIA_COST = 25.0
RIP_COST = 60.0


def astar(src, dst, tr, via, soft=None, softv=None):
    tgt = np.argwhere(dst)
    tcy, tcx = tgt[:, 1].mean(), tgt[:, 2].mean()
    hw = 0.95 if len(tgt) < 0.02 * dst[0].size else 0.0   # big targets (pours): plain Dijkstra finds the nearest copper
    INF = 1e18
    dist = np.full((2, NY, NX), INF)
    prev = {}
    heap = []
    for l, j, i in np.argwhere(src):
        dist[l, j, i] = 0
        heapq.heappush(heap, (math.hypot(j - tcy, i - tcx) * hw, 0.0, l, j, i))
    while heap:
        f, g, l, j, i = heapq.heappop(heap)
        if g > dist[l, j, i]:
            continue
        if dst[l, j, i]:
            path = [(l, j, i)]
            while (l, j, i) in prev:
                l, j, i = prev[(l, j, i)]
                path.append((l, j, i))
            return path[::-1]
        nbrs = []
        for di, dj, c in MOVES:
            jj, ii = j + dj, i + di
            if 0 <= jj < NY and 0 <= ii < NX and (not tr[l, jj, ii] or src[l, jj, ii]):
                nbrs.append((l, jj, ii, c + (RIP_COST if soft is not None and soft[l, jj, ii] and not src[l, jj, ii] else 0)))
        if not via[j, i] and not src[l, j, i]:
            nbrs.append((1 - l, j, i, VIA_COST + (RIP_COST if softv is not None and softv[j, i] else 0)))
        for nl, jj, ii, c in nbrs:
            ng = g + c
            if ng < dist[nl, jj, ii]:
                dist[nl, jj, ii] = ng
                prev[(nl, jj, ii)] = (l, j, i)
                heapq.heappush(heap, (ng + math.hypot(jj - tcy, ii - tcx) * hw, ng, nl, jj, ii))
    return None


def simplify(path):
    """Grid path -> list of ('track', layer, [pts]) and ('via', x, y)."""
    out, cur, layer = [], [], None          # cur holds integer grid points (i, j)
    for l, j, i in path:
        if layer is None:
            layer = l
        if l != layer:
            if len(cur) > 1:
                out.append(("track", "FB"[layer], cur))
            out.append(("via", cur[-1][0], cur[-1][1]))
            cur = [cur[-1]]
            layer = l
        if len(cur) >= 2:
            (ai, aj), (bi, bj) = cur[-2], cur[-1]
            if (bi - ai) * (j - bj) - (bj - aj) * (i - bi) == 0 and (bi - ai) * (i - bi) + (bj - aj) * (j - bj) >= 0:
                cur[-1] = (i, j)
                continue
        if not cur or cur[-1] != (i, j):
            cur.append((i, j))
    if len(cur) > 1:
        out.append(("track", "FB"[layer], cur))
    mmpt = lambda p: (round(X0 + p[0] * RES, 4), round(Y0 + p[1] * RES, 4))
    res = []
    for s in out:
        if s[0] == "track":
            res.append(("track", s[1], [mmpt(p) for p in s[2]]))
        else:
            x, y = mmpt((s[1], s[2]))
            res.append(("via", x, y))
    return res


results, failed = [], []
for pr in G["pairs"]:
    net = pr["net"]
    w = TRACK_W.get(net, TRACK_W["default"])
    tr, via = blocked(net, w)
    src, dst = island_mask(pr["a"]), island_mask(pr["b"])
    dst &= ~src
    soft = softv = None
    if RIPUP:
        soft = np.zeros((2, NY, NX), bool); softv = np.zeros((NY, NX), bool)
        for _, ln, mt, mv in SOFT:
            soft[ln] |= mt; softv |= mv
    path = astar(src, dst, tr, via, soft, softv)
    rip = set()
    if path is not None and RIPUP:
        for l, j, i in path:
            for k, ln, mt, mv in SOFT:
                if (ln == l and mt[j, i]):
                    rip.add(items[k]["uuid"])
        for (l1, j1, i1), (l2, j2, i2) in zip(path[:-1], path[1:]):
            if l1 != l2:
                for k, ln, mt, mv in SOFT:
                    if mv[j1, i1]:
                        rip.add(items[k]["uuid"])
    if path is None:
        failed.append(net); print("FAILED", net); continue
    segs = simplify(path)
    for s in segs:
        if s[0] == "track":
            for a, b in zip(s[2][:-1], s[2][1:]):
                added.append((net, s[1], seg_poly(a, b, w)))
        else:
            r = VIA_R
            added.append((net, "F", [(s[1] - r, s[2] - r), (s[1] + r, s[2] - r), (s[1] + r, s[2] + r), (s[1] - r, s[2] + r)]))
            added.append((net, "B", [(s[1] - r, s[2] - r), (s[1] + r, s[2] - r), (s[1] + r, s[2] + r), (s[1] - r, s[2] + r)]))
    L = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for s in segs if s[0] == "track" for a, b in zip(s[2][:-1], s[2][1:]))
    print(f"routed {net}: {L:.1f} mm, {sum(1 for s in segs if s[0] == 'via')} vias")
    results.append({"net": net, "width": w, "segs": segs, "ripup": sorted(rip)})
    if rip:
        print("   rips up", len(rip), "track segments")
json.dump({"routes": results, "failed": failed}, open(sys.argv[2], "w"))
