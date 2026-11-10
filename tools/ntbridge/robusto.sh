#!/bin/bash
# uso: ./robusto.sh arquivo.csv
# O TESTE QUE VALIDA GANHOS: passos de 15 graus E movimentos grandes (90/180, como o controle),
# com PARADA AUTOMATICA: 4+ inversoes de sentido (>0,7 grau) em 1 s, duas vezes seguidas =
# oscilando -> desliga o comando na hora. Robo HABILITADO. Imprime "ABORTADO" se ficou instavel.
cd "$(dirname "$0")"
D="$(pwd)"
./nt.sh enabled || { echo "ROBO NAO HABILITADO"; exit 1; }
ANG="0 90 180 0 15 30 45 60 75 90 270 90 180 165 150 135 180 0 90 0"
N=$(echo $ANG | wc -w); H=2.6
DUR=$(python -c "print(int($N*($H+0.9)+8))")
rm -f "$D/abortou.flag"
./nt.sh set /Tuning/Dial/TestSetpointDeg 0 >/dev/null
NT_STOPFILE="$D/abortou.flag" ./nt.sh rec $DUR "$1" Robot/m_robotContainer/dial/getGoalDeg Robot/m_robotContainer/dial/getQuadDeg Robot/m_robotContainer/dial/getDeg Robot/m_robotContainer/dial/getCommandedVolts >/dev/null & R=$!
( ruim=0; while kill -0 $R 2>/dev/null; do
    ./nt.sh rec 1 "$D/vigia.csv" Robot/m_robotContainer/dial/getQuadDeg >/dev/null 2>&1
    inv=$(python osc.py "$D/vigia.csv")
    if [ "${inv:-0}" -ge 4 ]; then ruim=$((ruim+1)); else ruim=0; fi
    if [ $ruim -ge 2 ]; then
      ./nt.sh set /Tuning/Dial/RemoteCmd 0 >/dev/null; touch "$D/abortou.flag"
      echo "ABORTADO: oscilando ($inv inversoes em 1 s)"; break
    fi
  done ) & W=$!
./nt.sh set /Tuning/Dial/RemoteCmd 7 >/dev/null
python -c "import time;time.sleep(2.5)"
for a in $ANG; do
  [ -f "$D/abortou.flag" ] && break
  ./nt.sh set /Tuning/Dial/TestSetpointDeg $a >/dev/null
  python -c "import time;time.sleep($H-0.15)"
done
./nt.sh set /Tuning/Dial/RemoteCmd 0 >/dev/null
wait $R 2>/dev/null
kill $W 2>/dev/null
if [ -f "$D/abortou.flag" ]; then echo "RESULTADO: ABORTADO (instavel)"; rm -f "$D/abortou.flag"; else python mapa_ana.py -r "$1"; fi
rm -f "$D/vigia.csv"
exit 0
