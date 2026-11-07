#!/bin/bash
# uso: kdtest.sh kP kD arquivo
cd "$(dirname "$0")"
./nt.sh enabled || { echo "ROBO NAO HABILITADO (habilite no Driver Station)"; exit 1; }
./nt.sh set /Tuning/Dial/kP $1 >/dev/null; ./nt.sh set /Tuning/Dial/kD $2 >/dev/null
./nt.sh set /Tuning/Dial/TestSetpointDeg 0 >/dev/null
./nt.sh rec 17 $3 Robot/m_robotContainer/dial/getDeg Robot/m_robotContainer/dial/getTargetDeg Robot/m_robotContainer/dial/getCommandedVolts Robot/m_robotContainer/dial/getPidVolts Robot/m_robotContainer/dial/getVelRpm /FMSInfo/FMSControlData >/dev/null &
REC=$!
./nt.sh set /Tuning/Dial/RemoteCmd 1 >/dev/null
python -c "import time; time.sleep(4)"
./nt.sh set /Tuning/Dial/TestSetpointDeg 60 >/dev/null
python -c "import time; time.sleep(4)"
./nt.sh set /Tuning/Dial/TestSetpointDeg 0 >/dev/null
python -c "import time; time.sleep(4)"
./nt.sh set /Tuning/Dial/RemoteCmd 0 >/dev/null
wait $REC
python step_ana.py "$3"
