"""Add silkscreen pin-function labels and a title next to every header on PID_Board_SMT.

For each pin the label goes on whichever side of the row (left/right for
vertical rows, above/below for horizontal rows) does not hit another
footprint's courtyard, a hole keepout or the board edge.
"""
import pcbnew

B = r"C:\Users\doubi\Quadcopter Flight controller\PID_Board_SMT\PID_Board_SMT.kicad_pcb"
X0, Y0, X1, Y1 = 20.0, 20.0, 120.0, 120.0
SIZE = 0.8
TAG = "pinlabel:"   # marker so re-runs replace earlier labels

HEADERS = {
    "J1": ("AXIS MON", ["ROLL", "PITCH", "YAW", "THR", "SPARE", "VGND", "GND"]),
    "J2": ("GY-521", ["VCC", "GND", "SCL", "SDA", "XDA", "XCL", "AD0", "INT"]),
    "J4": ("UPDI MCU", ["UPDI", "GND", "5V"]),
    "J5": ("ESC 4in1", ["B-", "B+", "M1", "M2", "M3", "M4", "NC", "CUR"]),
    "J6": ("ELRS RX", ["5V", "GND", "TX", "RX"]),
    "J7": ("UPDI ESC", ["UPDI", "GND", "5V"]),
}
for n in (2, 3, 4):
    HEADERS[f"JP{n}01"] = ("LOOP SP", ["SP", "SEL", "EXT"])
    HEADERS[f"JP{n}02"] = ("LOOP FB", ["GYRO", "SEL", "EXT"])

b = pcbnew.LoadBoard(B)
mm, to = pcbnew.FromMM, pcbnew.ToMM

for d in list(b.GetDrawings()):
    if d.GetClass() == "PCB_TEXT" and d.GetText().startswith(TAG):
        b.Remove(d)
# earlier runs stored the tag in the text's name only; find them by a hidden marker in the board's user-text list
existing = [d for d in b.GetDrawings() if d.GetClass() == "PCB_TEXT" and d.GetLayer() == pcbnew.F_SilkS
            and getattr(d, "GetText")() in {lbl for _, pins in HEADERS.values() for lbl in pins} | {t for t, _ in HEADERS.values()}]
for d in existing:
    b.Remove(d)

boxes = []
for fp in b.GetFootprints():
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    bb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
    boxes.append((fp.GetReference(), to(bb.GetLeft()), to(bb.GetTop()), to(bb.GetRight()), to(bb.GetBottom())))
placed_text = []


def clear(ref, x1, y1, x2, y2):
    if x1 < X0 + 0.6 or y1 < Y0 + 0.6 or x2 > X1 - 0.6 or y2 > Y1 - 0.6:
        return False
    for r, a, c, e, f in boxes:
        if r != ref and not (x2 < a or x1 > e or y2 < c or y1 > f):
            return False
    for a, c, e, f in placed_text:
        if not (x2 < a or x1 > e or y2 < c or y1 > f):
            return False
    return True


def add_text(t, x, y, angle=0):
    tx = pcbnew.PCB_TEXT(b)
    tx.SetText(t)
    tx.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    tx.SetLayer(pcbnew.F_SilkS)
    tx.SetTextSize(pcbnew.VECTOR2I(mm(SIZE), mm(SIZE)))
    tx.SetTextThickness(mm(0.12))
    if angle:
        tx.SetTextAngleDegrees(angle)
    b.Add(tx)


def width(t):
    return len(t) * SIZE * 0.85 + 0.2


missing = []
for ref, (title, pins) in HEADERS.items():
    fp = b.FindFootprintByReference(ref)
    if fp is None:
        missing.append(ref); continue
    pads = sorted(((int(p.GetNumber()), to(p.GetPosition().x), to(p.GetPosition().y)) for p in fp.Pads()))
    xs = [p[1] for p in pads]; ys = [p[2] for p in pads]
    vertical = (max(ys) - min(ys)) > (max(xs) - min(xs))
    # pick the side that is clear for the most labels
    best_side, best_ok = None, -1
    for side in (-1, 1):
        ok = 0
        for (num, px, py), lbl in zip(pads, pins):
            w = width(lbl)
            if vertical:
                cx = px + side * (1.6 + w / 2)
                ok += clear(ref, cx - w / 2, py - SIZE / 2, cx + w / 2, py + SIZE / 2)
            else:
                cy = py + side * (1.6 + w / 2)
                ok += clear(ref, px - SIZE / 2, cy - w / 2, px + SIZE / 2, cy + w / 2)
        if ok > best_ok:
            best_side, best_ok = side, ok
    for (num, px, py), lbl in zip(pads, pins):
        w = width(lbl)
        for side in (best_side, -best_side):          # per-pin fallback to the other side
            if vertical:
                cx = px + side * (1.6 + w / 2); box = (cx - w / 2, py - SIZE / 2, cx + w / 2, py + SIZE / 2)
            else:
                cy = py + side * (1.6 + w / 2); box = (px - SIZE / 2, cy - w / 2, px + SIZE / 2, cy + w / 2)
            if clear(ref, *box):
                break
        else:
            side = best_side
            if vertical:
                cx = px + side * (1.6 + w / 2); box = (cx - w / 2, py - SIZE / 2, cx + w / 2, py + SIZE / 2)
            else:
                cy = py + side * (1.6 + w / 2); box = (px - SIZE / 2, cy - w / 2, px + SIZE / 2, cy + w / 2)
        if vertical:
            add_text(lbl, (box[0] + box[2]) / 2, py)
        else:
            add_text(lbl, px, (box[1] + box[3]) / 2, 90)
        placed_text.append(box)
    # title (reference + function) at the first clear spot around the row; the footprint's own ref text is hidden
    title = f"{ref} {title}"
    w = width(title)
    fp.Reference().SetVisible(False)
    if vertical:
        cands = [(xs[0], min(ys) - 2.6), (xs[0], max(ys) + 2.6), (xs[0], min(ys) - 3.8), (xs[0], max(ys) + 3.8)]
    else:
        cands = [(min(xs) - 1.6 - w / 2, ys[0]), (max(xs) + 1.6 + w / 2, ys[0]),
                 ((min(xs) + max(xs)) / 2, ys[0] - 2.6), ((min(xs) + max(xs)) / 2, ys[0] + 2.6)]
    for cx, cy in cands + [cands[0]]:
        if clear(ref, cx - w / 2, cy - SIZE / 2, cx + w / 2, cy + SIZE / 2):
            break
    add_text(title, cx, cy)
    placed_text.append((cx - w / 2, cy - SIZE / 2, cx + w / 2, cy + SIZE / 2))
    print(f"{ref:6s} {'vertical' if vertical else 'horizontal'} side {best_side:+d} clear {best_ok}/{len(pins)}")

b.Save(B)
print("missing:", missing or "none")
