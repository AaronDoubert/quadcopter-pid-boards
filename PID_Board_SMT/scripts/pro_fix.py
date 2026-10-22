"""pcbnew.SaveBoard rewrites the .kicad_pro without our netclasses; put them (and board rules) back."""
import json, sys
p = sys.argv[1]
d = json.load(open(p, encoding="utf-8"))
ns = d["net_settings"]
base = next(c for c in ns["classes"] if c["name"] == "Default")
base["clearance"] = 0.15
pw = dict(base); pw.update(name="Power", track_width=0.4, via_diameter=0.6, via_drill=0.3, priority=0)
ns["classes"] = [base, pw]
ns["netclass_patterns"] = [{"netclass": "Power", "pattern": n} for n in ("+5V", "GND", "/Control_IO/VBAT", "/Control_IO/SW_NODE", "Net-(D1-K)")]
r = d["board"]["design_settings"]["rules"]
r["min_copper_edge_clearance"] = 0.3
r["min_resolved_spokes"] = 1
r["min_clearance"] = 0.15
json.dump(d, open(p, "w", encoding="utf-8"), indent=2)
print("project rules restored")
