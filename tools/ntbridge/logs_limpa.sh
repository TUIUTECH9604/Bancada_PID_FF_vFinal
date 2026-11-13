#!/bin/bash
# APAGA logs antigos do DISCO INTERNO do robô (/home/lvuser/logs), mantendo os N mais recentes.
# O disco interno da roboRIO tem ~3 GB; cheio, o deploy pode falhar. Não mexe no pendrive.
#   ./logs_limpa.sh 3            mostra o que seria apagado (não apaga nada)
#   ./logs_limpa.sh 3 APAGAR     apaga de verdade (só depois de baixar com ./logs_get.sh todos)
cd "$(dirname "$0")" || exit 1
IP="${NT_SERVER:-169.254.29.48}"
MANTER="${1:?uso: ./logs_limpa.sh QUANTOS_MANTER [APAGAR]}"
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 -o LogLevel=ERROR)
ALVOS=$(ssh "${SSH_OPTS[@]}" "lvuser@$IP" \
  "ls -1t /home/lvuser/logs/*.wpilog /home/lvuser/logs/*.revlog 2>/dev/null | tail -n +$((MANTER + 1))" \
  < /dev/null) || { echo "não conectou em $IP"; exit 1; }
[ -z "$ALVOS" ] && { echo "nada a apagar"; exit 0; }
echo "Seriam apagados:"; echo "$ALVOS"
# confere que cada um já foi baixado para logs/ (os .revlog não são baixados: só avisa)
for f in $ALVOS; do
  case "$f" in *.wpilog) [ -f "logs/$(basename "$f")" ] || echo "  ATENÇÃO: $(basename "$f") NÃO foi baixado";; esac
done
if [ "$2" != "APAGAR" ]; then
  echo "(nada foi apagado; para apagar: ./logs_limpa.sh $MANTER APAGAR)"; exit 0
fi
ssh "${SSH_OPTS[@]}" "lvuser@$IP" "rm -f $(echo $ALVOS); df -h /home/lvuser | tail -1" < /dev/null
