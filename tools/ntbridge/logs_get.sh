#!/bin/bash
# Baixa os .wpilog do robô para tools/ntbridge/logs/ (pendrive /u/logs e disco interno).
#   ./logs_get.sh            lista o que tem no robô e baixa os 3 mais recentes
#   ./logs_get.sh 10         baixa os 10 mais recentes
#   ./logs_get.sh todos      baixa todos
# Depois: python log_ana.py logs/ARQUIVO.wpilog
# Robô: NT_SERVER (padrão 169.254.29.48, cabo; Wi-Fi 10.96.4.2). Não apaga nada.
cd "$(dirname "$0")" || exit 1
IP="${NT_SERVER:-169.254.29.48}"
QUANTOS="${1:-3}"
mkdir -p logs
# accept-new: na 1ª vez guarda a chave do roboRIO (é o robô da equipe, na rede da bancada)
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 -o LogLevel=ERROR)

LISTA=$(ssh "${SSH_OPTS[@]}" "lvuser@$IP" \
  'ls -1t /u/logs/*.wpilog /home/lvuser/logs/*.wpilog 2>/dev/null; echo "--"; df -h /home/lvuser /u 2>/dev/null' \
  < /dev/null) || { echo "não conectou em $IP (robô ligado? NT_SERVER certo?)"; exit 1; }
ARQS=$(echo "$LISTA" | sed '/^--$/,$d')
echo "$LISTA" | sed -n '/^--$/,$p' | tail -n +2
echo "No robô: $(echo "$ARQS" | grep -c wpilog) arquivo(s) .wpilog"
[ "$QUANTOS" = "todos" ] && SEL="$ARQS" || SEL=$(echo "$ARQS" | head -n "$QUANTOS")
for f in $SEL; do
  n=$(basename "$f")
  if [ -f "logs/$n" ]; then echo "  já baixado: $n"; continue; fi
  echo "  baixando $f"
  scp -q "${SSH_OPTS[@]}" "lvuser@$IP:$f" "logs/$n" < /dev/null || echo "  FALHOU: $f"
done
ls -lh logs | tail -n +2
