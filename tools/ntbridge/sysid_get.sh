#!/bin/bash
# Baixa o CSV do último SysId (RemoteCmd 9) do robô e ajusta kS, kG, kV, kA.
cd "$(dirname "$0")"
F=${1:-sysid_$(date +%Y%m%d_%H%M%S).csv}
scp -q lvuser@${NT_SERVER:-169.254.29.48}:/home/lvuser/sysid.csv "$F" || { echo "falha ao baixar"; exit 1; }
python sysid_fit.py "$F"
