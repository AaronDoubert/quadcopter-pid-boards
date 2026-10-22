"""Build PID_Board_SMT.kicad_pcb from the kicad-cli XML netlist (no GUI 'Update PCB' needed).

Usage: KiCad python.exe create_pcb.py <netlist.xml> <out.kicad_pcb>
Footprints come from the stock libraries, nets from the netlist, symbol links (KIID paths)
from the netlist sheet paths so DRC schematic parity works.
"""
import sys, os
import xml.etree.ElementTree as ET
import pcbnew

FPDIR = r"C:\Program Files\KiCad\10.0\share\kicad\footprints"
xml, out = sys.argv[1], sys.argv[2]
r = ET.parse(xml).getroot()

# per-unit fields: footprint fields must match the unit its KIID path points at (DRC parity)
import re, glob
unit_fields = {}
for f in glob.glob(os.path.join(os.path.dirname(out), "*.kicad_sch")):
    t = open(f, encoding="utf-8").read()
    for m in re.finditer(r'\n\t\(symbol\n\t\t\(lib_id "[^"]+"\)(.*?)\n\t\)\n', t, re.S):
        blk = m.group(1)
        ref = re.search(r'\(reference "([^"]+)"\)', blk).group(1)
        unit = int(re.search(r'\(unit (\d+)\)', blk).group(1))
        if ref not in unit_fields or unit < unit_fields[ref][0]:   # parity compares against the lowest unit
            unit_fields[ref] = (unit, dict(re.findall(r'\(property "(Function|Guide_Ref)" "([^"]*)"', blk)))

b = pcbnew.CreateEmptyBoard()
nets = {}
for n in r.iter("net"):
    name = n.get("name")
    ni = pcbnew.NETINFO_ITEM(b, name)
    b.Add(ni)
    nets[name] = ni
pin_net = {}
for n in r.iter("net"):
    for node in n.iter("node"):
        pin_net[(node.get("ref"), node.get("pin"))] = nets[n.get("name")]

for c in r.iter("comp"):
    ref, val, fpid = c.get("ref"), c.findtext("value"), c.findtext("footprint")
    lib, name = fpid.split(":")
    fp = pcbnew.FootprintLoad(os.path.join(FPDIR, lib + ".pretty"), name)
    assert fp, fpid
    fp.SetReference(ref)
    fp.SetValue(val)
    fp.SetFPIDAsString(fpid)
    props = {p.get("name"): p.get("value") for p in c.iter("property")}
    sp = c.find("sheetpath").get("tstamps")
    first = c.findtext("tstamps").split()[0]
    fields = {"LCSC": props.get("LCSC")}
    fields.update(unit_fields.get(ref, (0, {}))[1])
    for k, v in fields.items():
        if v:
            fp.SetField(k, v)
            fp.GetField(k).SetVisible(False)
    fp.SetPath(pcbnew.KIID_PATH(sp + first))
    fp.SetSheetname(props.get("Sheetname", ""))
    fp.SetSheetfile(props.get("Sheetfile", ""))
    fp.SetExcludedFromBOM(False)   # match the symbols (all in_bom yes)
    if "dnp" in props or val.endswith("DNP"):
        fp.SetDNP(True)
    for pad in fp.Pads():
        ni = pin_net.get((ref, pad.GetNumber()))
        if ni:
            pad.SetNet(ni)
    b.Add(fp)

b.SetCopperLayerCount(2)
pcbnew.SaveBoard(out, b)
print("footprints", len(b.GetFootprints()), "nets", len(nets))
