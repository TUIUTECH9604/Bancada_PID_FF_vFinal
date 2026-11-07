#!/bin/bash
# uso: holdtest.sh arquivo  (perfil até 60°, segura 12 s, volta a 0)
cd "$(dirname "$0")"
./nt.sh enabled || { echo "ROBO NAO HABILITADO (habilite no Driver Station)"; exit 1; }
./nt.sh set /Tuning/Dial/TestSetpointDeg 60 >/dev/null
./nt.sh rec 16 $1 Robot/m_robotContainer/dial/getDeg Robot/m_robotContainer/dial/getGoalDeg Robot/m_robotContainer/dial/getCommandedVolts Robot/m_robotContainer/dial/getIntegralVolts >/dev/null & R=$!
./nt.sh set /Tuning/Dial/RemoteCmd 7 >/dev/null
python -c "import time;time.sleep(14)"
./nt.sh set /Tuning/Dial/TestSetpointDeg 0 >/dev/null
python -c "import time;time.sleep(1)"
./nt.sh set /Tuning/Dial/RemoteCmd 0 >/dev/null
wait $R
python - $1 <<'PY'
import csv,collections,sys
d=collections.defaultdict(list)
for r in csv.DictReader(open(sys.argv[1])):
    try:d[r['topic'].split('/')[-1]].append((float(r['t']),float(r['value'])))
    except:pass
t0=[t for t,v in d['getGoalDeg'] if v==60.0][0]
for a in range(2,14,2):
    seg=[x-60 for t,x in d['getDeg'] if t0+a<=t<t0+a+2]
    iv=[x for t,x in d['getIntegralVolts'] if t0+a<=t<t0+a+2]
    if seg: print('t %2d-%2d s: erro medio %5.2f  min %5.2f max %5.2f pp %4.2f | I %.3f..%.3f V'%(a,a+2,sum(seg)/len(seg),min(seg),max(seg),max(seg)-min(seg),min(iv) if iv else 0,max(iv) if iv else 0))
PY
