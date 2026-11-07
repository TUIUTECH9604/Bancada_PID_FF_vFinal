#!/bin/bash
# uso: motiontest.sh velDegps accelDegps2 kA arquivo   (perfil 0 -> 90 -> 0, 3 s cada)
cd "$(dirname "$0")"
./nt.sh enabled || { echo "ROBO NAO HABILITADO (habilite no Driver Station)"; exit 1; }
./nt.sh set /Tuning/Dial/ProfMaxVelDegps $1 >/dev/null; ./nt.sh set /Tuning/Dial/ProfMaxAccelDegps2 $2 >/dev/null; ./nt.sh set /Tuning/Dial/kA $3 >/dev/null
./nt.sh set /Tuning/Dial/TestSetpointDeg 0 >/dev/null
./nt.sh rec 13 $4 Robot/m_robotContainer/dial/getDeg Robot/m_robotContainer/dial/getTargetDeg Robot/m_robotContainer/dial/getGoalDeg Robot/m_robotContainer/dial/getErrorDeg Robot/m_robotContainer/dial/getCommandedVolts >/dev/null & R=$!
./nt.sh set /Tuning/Dial/RemoteCmd 7 >/dev/null
python -c "import time;time.sleep(3.5)"
./nt.sh set /Tuning/Dial/TestSetpointDeg 90 >/dev/null
python -c "import time;time.sleep(3)"
./nt.sh set /Tuning/Dial/TestSetpointDeg 0 >/dev/null
python -c "import time;time.sleep(3)"
./nt.sh set /Tuning/Dial/RemoteCmd 0 >/dev/null
wait $R
python motion_ana.py $4
