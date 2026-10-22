"/c/Program Files/KiCad/10.0/bin/kicad-cli.exe" pcb drc --format json --severity-all --schematic-parity -o drc.json "C:/Users/doubi/Quadcopter Flight controller/PID_Board_SMT/PID_Board_SMT.kicad_pcb" >/dev/null 2>&1
python -I - drc.json <<'PY'
import json,sys,collections
d=json.load(open(sys.argv[1]))
v=d.get('violations',[]); u=d.get('unconnected_items',[]); p=d.get('schematic_parity',[])
print('violations',len(v),collections.Counter((x['type'],x['severity']) for x in v))
print('unconnected',len(u),'| parity',len(p),collections.Counter(x['type'] for x in p))
for x in (v+u+p)[:40]: print('  ',x['type'],'|',' ; '.join(f"{i['description']} @({i['pos']['x']*1:.2f},{i['pos']['y']*1:.2f})" for i in x['items'])[:230])
PY
