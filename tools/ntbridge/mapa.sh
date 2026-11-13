#!/bin/bash
# uso: [MEC=<mecanismo>] ./mapa.sh arquivo.csv [segundos_por_alvo]
# Mapa: passos curtos por todo o curso e de volta (MAPA do .conf; sem MAPA, os ALVOS), segurando
# cada alvo. Robô HABILITADO. Análise: python mapa_ana.py arquivo.csv (-r arquivo.csv: só o resumo).
cd "$(dirname "$0")" || exit 1
source ./mec.sh
./nt.sh enabled || { echo "ROBO NAO HABILITADO"; exit 1; }
H=${2:-$ESPERA}
SEQ="${MAPA:-$ALVOS}"
N=$(echo $SEQ | wc -w)
DUR=$(python -c "print(int($N*($H+3.5)+8))")   # ~3 s por comando pelo Wi-Fi
./nt.sh set $TUN/TestSetpointDeg $(echo $SEQ | awk '{print $1}') >/dev/null
./nt.sh rec $DUR "$1" $T/getGoalDeg $T/getQuadDeg $T/getDeg $T/getCommandedVolts $T/getPidVolts \
  $T/getIntegralVolts $T/getProfileVelDegps >/dev/null & R=$!
./nt.sh set $TUN/RemoteCmd 7 >/dev/null
python -c "import time;time.sleep(2.5)"
for a in $SEQ; do ./nt.sh set $TUN/TestSetpointDeg $a >/dev/null; python -c "import time;time.sleep($H-0.15)"; done
./nt.sh set $TUN/RemoteCmd 0 >/dev/null
kill $R 2>/dev/null; wait $R 2>/dev/null
