"""Prepare a KiCad-exported Specctra DSN for FreeRouting.

* power nets move out of kicad_default into wider classes
* every clearance becomes 150 um (6 mil, JLCPCB standard)
* thin keepout strips along the board edge keep copper off it (FreeRouting has no edge rule;
  shrinking the boundary itself made FreeRouting 2.1 write incomplete session files)
"""
import re, sys
p = sys.argv[1]
s = open(p, encoding="utf-8").read()
CLASSES = {"Power": (["+5V", "/Control_IO/VBAT", '"Net-(D1-K)"', "/Control_IO/SW_NODE"], 400), "Gnd": (["GND"], 300)}
if "(class Power" in s:          # KiCad already exported the project's Power netclass
    CLASSES = {}
i = s.index("(class kicad_default")
j = s.index("(circuit", i)
head = s[i:j]
for nets, _ in CLASSES.values():
    for n in nets:
        head, k = re.subn(r'(?<=\s)' + re.escape(n) + r'(?=\s)', ' ', head, count=1)
        assert k == 1, n
s = s[:i] + head + s[j:]
end = s.index("(rule", s.index("(circuit", i))
end = s.index(")\n    )", end) + len(")\n    )")
extra = ""
for name, (nets, w) in CLASSES.items():
    extra += (f"\n    (class {name} {' '.join(nets)}\n      (circuit\n        (use_via \"Via[0-1]_600:300_um\")\n      )\n"
              f"      (rule\n        (width {w})\n        (clearance 200)\n      )\n    )")
s = s[:end] + extra + s[end:]
s = s.replace("(clearance 200)", "(clearance 150)")
s = s.replace("(width 400)", "(width 300)")
# GND is not autorouted: the pours on both layers carry it (stitch_gnd.py adds vias for stranded pads)
gi = s.index("\n    (net GND\n")
ge = s.index("\n    )", gi) + len("\n    )")
s = s[:gi] + s[ge:]
s = re.sub(r"(\(class [^\n(]*?)\sGND(?=[\s)])", r"\1", s)

m = re.search(r"\(boundary\s*\(path pcb 0\s+([-\d\s]+)\)", s)
nums = [int(v) for v in m.group(1).split()]
xs, ys = nums[0::2], nums[1::2]
x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
W = 200
strips = [(x0, y0, x1, y0 + W), (x0, y1 - W, x1, y1), (x0, y0, x0 + W, y1), (x1 - W, y0, x1, y1)]
ko = ""
for k, (a, b, c, d) in enumerate(strips):
    ko += f'\n    (keepout "edge{k}" (polygon signal 0  {a} {b}  {c} {b}  {c} {d}  {a} {d}  {a} {b}))'
close = s.index("\n    )", m.end()) + len("\n    )")
s = s[:close] + ko + s[close:]
open(p, "w", encoding="utf-8").write(s)
print("classes added; clearance 150 um; edge keepouts", W, "um")
