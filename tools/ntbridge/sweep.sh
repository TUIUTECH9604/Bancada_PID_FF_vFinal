#!/bin/bash
# uso: ./sweep.sh nome "kD=0.00015 kI=0.01 ..."   (parametros em /Tuning/Dial/, ao vivo)
# Aplica os parametros e roda o teste robusto. Os valores ficam aplicados ao terminar:
# restaure os de antes (ou reinicie o codigo) se nao for usar.
cd "$(dirname "$0")"
./nt.sh enabled || { echo "ROBO NAO HABILITADO"; exit 1; }
for kv in $2; do ./nt.sh set /Tuning/Dial/${kv%%=*} ${kv#*=} >/dev/null; done
./robusto.sh "rb_$1.csv"
