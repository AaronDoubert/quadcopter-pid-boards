set -eo pipefail
SPW="C:\Users\doubi\AppData\Local\Temp\claude\C--Users-doubi--claude-spares-spare-XByeoP\0869fac4-e185-4ab7-8431-c3438d3c504e\scratchpad"
PCBW="C:\Users\doubi\Quadcopter Flight controller\PID_Board_SMT\PID_Board_SMT.kicad_pcb"
KPY="/c/Program Files/KiCad/10.0/bin/python.exe"
rm -f route/board.dsn route/board.ses
"$KPY" dsn.py "$PCBW" "$SPW\route\board.dsn" 2>&1 | grep -v swig
python -I dsn_classes.py route/board.dsn
( cd route && timeout 1200 "/c/Program Files/Eclipse Adoptium/jdk-21.0.7.6-hotspot/bin/java.exe" -jar /c/Users/doubi/.local/share/freerouting-2.1.0/freerouting-2.1.0.jar -de board.dsn -do board.ses -mp ${1:-40} -mt 1 --gui.enabled=false > fr.log 2>&1 || true )
grep -oE "pass #[0-9]+ .*" route/fr.log | tail -3
ls -la route/board.ses
