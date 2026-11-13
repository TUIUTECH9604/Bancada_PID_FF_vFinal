#!/bin/bash
# uso: [MEC=<mecanismo>] ./sweep.sh nome "kD=0.00015 kI=0.01 ..."   (ajustes ao vivo em /Tuning/<NOME>/)
# Aplica os ajustes (numa conexão só) e roda o teste robusto. Os valores FICAM aplicados ao terminar:
# restaure os de antes (ou reinicie o código) se não for usar. Para comparar várias: bateria.sh.
cd "$(dirname "$0")" || exit 1
source ./mec.sh
./nt.sh enabled || { echo "ROBO NAO HABILITADO"; exit 1; }
a=(); for kv in $2; do a+=("$TUN/${kv%%=*}" "${kv#*=}"); done
[ ${#a[@]} -gt 0 ] && ./nt.sh setm "${a[@]}"
./robusto.sh "rb_$1.csv"
