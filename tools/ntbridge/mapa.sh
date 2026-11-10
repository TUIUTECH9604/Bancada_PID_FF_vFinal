#!/bin/bash
# uso: ./mapa.sh arquivo.csv [segundos_por_angulo]
# Passos de 15 graus (com perfil) de 0 a 180 e de volta, segurando cada angulo. Robo HABILITADO.
# Analise: python mapa_ana.py arquivo.csv   (ou -r arquivo.csv ... para so o resumo)
cd "$(dirname "$0")"
./nt.sh enabled || { echo "ROBO NAO HABILITADO"; exit 1; }
H=${2:-2.6}
ANG="0 15 30 45 60 75 90 105 120 135 150 165 180 165 150 135 120 105 90 75 60 45 30 15 0"
N=$(echo $ANG | wc -w)
DUR=$(python -c "print(int($N*($H+0.9)+8))")
./nt.sh set /Tuning/Dial/TestSetpointDeg 0 >/dev/null
./nt.sh rec $DUR "$1" Robot/m_robotContainer/dial/getGoalDeg Robot/m_robotContainer/dial/getQuadDeg Robot/m_robotContainer/dial/getDeg Robot/m_robotContainer/dial/getCommandedVolts Robot/m_robotContainer/dial/getPidVolts Robot/m_robotContainer/dial/getIntegralVolts >/dev/null & R=$!
./nt.sh set /Tuning/Dial/RemoteCmd 7 >/dev/null
python -c "import time;time.sleep(2.5)"
for a in $ANG; do ./nt.sh set /Tuning/Dial/TestSetpointDeg $a >/dev/null; python -c "import time;time.sleep($H-0.15)"; done
./nt.sh set /Tuning/Dial/RemoteCmd 0 >/dev/null
wait $R
