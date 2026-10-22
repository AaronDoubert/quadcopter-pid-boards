"""Place every footprint of PID_Board_SMT (run with KiCad's python.exe).

Usage: python.exe place_smt.py <board.kicad_pcb> <size_mm>

Square board (100 mm = JLCPCB's cheapest 2-layer size). The centre keeps the 20x20 ESC stack
holes, the GY-521 header at the CG and the ESC cable slot + harness header. Roll / pitch PID on
the left, reference / yaw / control I/O down the middle, DAC shim on the right.
Connectors and jumpers sit on board edges; everything else is placed greedily: next part =
most connections to placed parts, on the free grid spot (0/90/180/270) that minimises the
distance from its signal pads to already-placed pads on the same net. A part that does not fit
its region looks in a progressively larger window around it.
"""
import math, sys
import pcbnew

BOARD = sys.argv[1]
S = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
X0, Y0 = 20.0, 20.0
X1, Y1 = X0 + S, Y0 + S
CX, CY = X0 + S / 2, Y0 + S / 2
CELL = 0.635
CLEAR = 0.35
POWER = {"GND", "+5V"}
E = 1.5
LW = S * 0.335
REGIONS = {
    "roll":  (X0 + E, Y0 + E, X0 + LW, CY),
    "pitch": (X0 + E, CY, X0 + LW, Y1 - E),
    "ref":   (X0 + LW, Y0 + E, X1 - LW, CY - 15.0),
    "yaw":   (X0 + LW, CY - 15.0, X1 - LW, CY + 17.0),
    "ctrl":  (X0 + LW, CY + 17.0, X1 - LW, Y1 - E),
    "shim":  (X1 - LW, Y0 + E, X1 - E, Y1 - E),
}
HOLE_IN = 3.5
CORNER_HOLES = [(X0 + HOLE_IN, Y0 + HOLE_IN), (X1 - HOLE_IN, Y0 + HOLE_IN), (X0 + HOLE_IN, Y1 - HOLE_IN), (X1 - HOLE_IN, Y1 - HOLE_IN)]
STACK_HOLES = [(CX - 10, CY - 10), (CX + 10, CY - 10), (CX - 10, CY + 10), (CX + 10, CY + 10)]
IMU_ROW_Y = CY - 3.96
GY521 = (CX - 10.6, IMU_ROW_Y - 1.3, CX + 10.6, IMU_ROW_Y + 15.1)
SLOT = (CX - 6.67, CY + 19.54, CX + 6.67, CY + 22.71)
SHIM_EXTRA = {"J6", "J1"}
REF_EXTRA = {"Q1", "Q2", "Q3", "R1", "R2", "R3", "R4", "R23"}   # level shifters: centre column has room


def group(ref, sheet):
    if ref in SHIM_EXTRA:
        return "shim"
    if ref in REF_EXTRA:
        return "ref"
    return {"PID_Reference": "ref", "PID_Roll": "roll", "PID_Pitch": "pitch", "PID_Yaw": "yaw",
            "Control_IO": "ctrl"}.get(sheet, "ctrl" if ref.startswith("H") else "shim")


def fixed():
    f = {}
    for i, (hx, hy) in enumerate(CORNER_HOLES + STACK_HOLES):
        f[f"H{i + 1}"] = (hx, hy, 0)
    f["J2"] = (CX - 8.89, IMU_ROW_Y, 90)
    f["J5"] = (CX - 8.89, CY + 26.67, 90)
    f["J4"] = (X1 - 2.54, Y0 + 10.16, 0)
    f["J6"] = (X1 - 2.54, Y0 + 19.05, 0)
    f["J1"] = (X1 - 2.54, Y1 - 27.94, 0)
    f["J7"] = (CX + 3.81, Y1 - 2.54, 90)
    f["JP201"] = (X0 + 2.54, Y0 + 12.7, 0)
    f["JP202"] = (X0 + 2.54, Y0 + 21.59, 0)
    f["JP301"] = (X0 + 2.54, Y1 - 26.67, 0)
    f["JP302"] = (X0 + 2.54, Y1 - 17.78, 0)
    f["JP401"] = (CX - 16.51, CY - 13.97, 0)
    f["JP402"] = (CX + 16.51, CY - 13.97, 0)
    return f


def mm(v):
    return pcbnew.ToMM(v)


class Placer:
    def __init__(self, b):
        self.b = b
        self.fps = {fp.GetReference(): fp for fp in b.GetFootprints()}
        self.grp = {r: group(r, fp.GetSheetname().strip("/")) for r, fp in self.fps.items()}
        self.geo = {}
        for ref, fp in self.fps.items():
            fp.SetPosition(pcbnew.VECTOR2I(0, 0))
            g = {}
            for rot in (0, 90, 180, 270):
                fp.SetOrientationDegrees(rot)
                cy = fp.GetCourtyard(pcbnew.F_CrtYd)
                bb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
                pads = [(mm(p.GetPosition().x), mm(p.GetPosition().y), p.GetNetname()) for p in fp.Pads()]
                g[rot] = ((mm(bb.GetLeft()), mm(bb.GetTop()), mm(bb.GetRight()), mm(bb.GetBottom())), pads)
            self.geo[ref] = g
        self.nx, self.ny = int(S / CELL) + 2, int(S / CELL) + 2
        self.occ = bytearray(self.nx * self.ny)
        self.placed, self.netpads = {}, {}

    def cells(self, x1, y1, x2, y2):
        i1, j1 = int((x1 - X0) / CELL), int((y1 - Y0) / CELL)
        i2, j2 = int(math.ceil((x2 - X0) / CELL)), int(math.ceil((y2 - Y0) / CELL))
        return max(i1, 0), max(j1, 0), min(i2, self.nx - 1), min(j2, self.ny - 1)

    def free(self, x1, y1, x2, y2):
        if x1 < X0 + 0.8 or y1 < Y0 + 0.8 or x2 > X1 - 0.8 or y2 > Y1 - 0.8:
            return False
        i1, j1, i2, j2 = self.cells(x1, y1, x2, y2)
        for j in range(j1, j2 + 1):
            row = j * self.nx
            if any(self.occ[row + i1: row + i2 + 1]):
                return False
        return True

    def mark(self, x1, y1, x2, y2, pad=CLEAR):
        i1, j1, i2, j2 = self.cells(x1 - pad, y1 - pad, x2 + pad, y2 + pad)
        for j in range(j1, j2 + 1):
            row = j * self.nx
            for i in range(i1, i2 + 1):
                self.occ[row + i] = 1

    def put(self, ref, x, y, rot):
        (l, t, r, btm), pads = self.geo[ref][rot]
        self.mark(x + l, y + t, x + r, y + btm)
        self.placed[ref] = (x, y, rot)
        for px, py, net in pads:
            if net and net not in POWER:
                self.netpads.setdefault(net, []).append((x + px, y + py, self.grp[ref]))

    def try_place(self, ref, grow):
        g = self.grp[ref]
        rx0, ry0, rx1, ry1 = REGIONS[g]
        rx0, ry0, rx1, ry1 = max(rx0 - grow, X0), max(ry0 - grow, Y0), min(rx1 + grow, X1), min(ry1 + grow, Y1)
        pts = []
        for n in self.nets_of[ref]:
            for px, py, pg in self.netpads.get(n, []):
                pts.append((px, py, 1.0 if pg == g else 0.3))
        if pts:
            wsum = sum(w for _, _, w in pts)
            tx = sum(px * w for px, _, w in pts) / wsum
            ty = sum(py * w for _, py, w in pts) / wsum
        else:
            tx, ty = (rx0 + rx1) / 2, (ry0 + ry1) / 2
        tx = min(max(tx, rx0), rx1); ty = min(max(ty, ry0), ry1)
        best = None
        rots = (0,) if ref.startswith(("TP", "J")) else (0, 90, 180, 270)
        for rot in rots:
            (l, t, r, btm), pads = self.geo[ref][rot]
            cand = []
            gx0 = math.ceil((rx0 - l) / CELL); gx1 = math.floor((rx1 - r) / CELL)
            gy0 = math.ceil((ry0 - t) / CELL); gy1 = math.floor((ry1 - btm) / CELL)
            for gi in range(gx0, gx1 + 1):
                for gj in range(gy0, gy1 + 1):
                    x, y = gi * CELL, gj * CELL
                    cx, cy = x + (l + r) / 2, y + (t + btm) / 2
                    cand.append(((cx - tx) ** 2 + (cy - ty) ** 2, x, y))
            cand.sort()
            tried = 0
            for _, x, y in cand:
                if not self.free(x + l, y + t, x + r, y + btm):
                    continue
                cost = 0.0
                for px, py, net in pads:
                    if not net or net in POWER or net not in self.netpads:
                        continue
                    ax, ay = x + px, y + py
                    cost += min(math.hypot(ax - qx, ay - qy) * (1.0 if qg == g else 0.4) for qx, qy, qg in self.netpads[net])
                if not any(net in self.netpads for _, _, net in pads if net):
                    cost = math.hypot(x - tx, y - ty)
                if best is None or cost < best[0]:
                    best = (cost, x, y, rot)
                tried += 1
                if tried >= 80:
                    break
        return best

    def run(self):
        for ref, (x, y, rot) in fixed().items():
            if ref in self.fps:
                self.put(ref, x, y, rot)
        for hx, hy in CORNER_HOLES + STACK_HOLES:
            self.mark(hx - 3.6, hy - 3.6, hx + 3.6, hy + 3.6, 0)
        self.mark(*SLOT, 1.0)
        self.nets_of = {r: {n for _, _, n in self.geo[r][0][1] if n and n not in POWER} for r in self.fps}
        todo = set(self.fps) - set(self.placed)
        grown = []
        while todo:
            ref = max(sorted(todo), key=lambda r: (sum(1 for n in self.nets_of[r] if n in self.netpads), len(self.geo[r][0][1])))
            best = None
            for grow in (0, 4, 8, 14, 25, 100):
                best = self.try_place(ref, grow)
                if best:
                    break
            if best is None:
                print("NO ROOM for", ref); sys.exit(1)
            if grow:
                grown.append((ref, grow))
            self.put(ref, best[1], best[2], best[3])
            todo.discard(ref)
        if grown:
            print("placed outside their region:", grown)

    def finish(self):
        b = self.b
        for ref, (x, y, rot) in self.placed.items():
            fp = self.fps[ref]
            fp.SetOrientationDegrees(rot)
            fp.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
            fp.Reference().SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(0.8), pcbnew.FromMM(0.8)))
            fp.Reference().SetTextThickness(pcbnew.FromMM(0.15))
            if ref.startswith("H"):
                fp.Reference().SetVisible(False)
        for d in list(b.GetDrawings()):
            b.Remove(d)

        def shape(kind, x1, y1, x2, y2, layer, w):
            s = pcbnew.PCB_SHAPE(b)
            s.SetShape(kind)
            s.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(x1), pcbnew.FromMM(y1)))
            s.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(x2), pcbnew.FromMM(y2)))
            s.SetLayer(layer); s.SetWidth(pcbnew.FromMM(w))
            b.Add(s)

        def text(t, x, y, layer, size=1.0, mirror=False):
            tx = pcbnew.PCB_TEXT(b)
            tx.SetText(t)
            tx.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
            tx.SetLayer(layer)
            tx.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
            tx.SetTextThickness(pcbnew.FromMM(size * 0.15))
            if mirror:
                tx.SetMirrored(True)
            b.Add(tx)

        shape(pcbnew.SHAPE_T_RECT, X0, Y0, X1, Y1, pcbnew.Edge_Cuts, 0.1)
        shape(pcbnew.SHAPE_T_RECT, *SLOT, pcbnew.Edge_Cuts, 0.1)
        shape(pcbnew.SHAPE_T_RECT, *GY521, pcbnew.F_Fab, 0.1)
        ra = pcbnew.ZONE(b)
        ra.SetIsRuleArea(True)
        ra.SetDoNotAllowTracks(True); ra.SetDoNotAllowVias(True); ra.SetDoNotAllowPads(False)
        ra.SetDoNotAllowFootprints(False); ra.SetDoNotAllowZoneFills(True)
        ls = pcbnew.LSET(); ls.AddLayer(pcbnew.F_Cu); ls.AddLayer(pcbnew.B_Cu)
        ra.SetLayerSet(ls)
        ol = ra.Outline(); ol.NewOutline()
        for x, y in ((SLOT[0] - 1, SLOT[1] - 1), (SLOT[2] + 1, SLOT[1] - 1), (SLOT[2] + 1, SLOT[3] + 1), (SLOT[0] - 1, SLOT[3] + 1)):
            ol.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
        ra.SetZoneName("ESC_slot_keepout")
        b.Add(ra)
        shape(pcbnew.SHAPE_T_SEGMENT, CX - 2, CY, CX + 2, CY, pcbnew.F_Fab, 0.1)
        shape(pcbnew.SHAPE_T_SEGMENT, CX, CY - 2, CX, CY + 2, pcbnew.F_Fab, 0.1)
        text("GY-521 above, CG +", CX, IMU_ROW_Y + 13.5, pcbnew.F_Fab, 0.8)
        shape(pcbnew.SHAPE_T_RECT, CX - 14.0, CY - 15.0, CX + 14.0, CY + 15.0, pcbnew.B_SilkS, 0.15)
        text("4in1 ESC below (20x20 M3)", CX, CY + 16.5, pcbnew.B_SilkS, 0.9, mirror=True)
        text("ESC cable", CX, SLOT[3] + 1.4, pcbnew.F_SilkS, 0.8)
        text("PID_Board_SMT  FRONT ->", CX, Y0 + 1.6, pcbnew.F_SilkS, 0.9)
        b.Save(BOARD)
        used = sum((self.geo[r][p[2]][0][2] - self.geo[r][p[2]][0][0]) * (self.geo[r][p[2]][0][3] - self.geo[r][p[2]][0][1])
                   for r, p in self.placed.items())
        print("placed", len(self.placed), "footprints; courtyard %.0f mm2 on %.0f mm2" % (used, S * S))


p = Placer(pcbnew.LoadBoard(BOARD))
p.run()
p.finish()
