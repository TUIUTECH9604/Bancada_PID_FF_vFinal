#!/bin/bash
# Teste de parada no PONTO FRACO do mecanismo (PARADA_ALVO do .conf), onde a gravidade não segura
# o mecanismo contra a folga e o D pode sustentar um ciclo-limite (na bancada: 270°, 12,5 Hz,
# 2026-11-12). Para cada valor do ajuste PARAM: N vezes (partindo de cada PARADA_DE, alternando) →
# PARADA_ALVO, segura e vê se a proteção do robô (OscGuard, laço de 5 ms) disparou; se disparou,
# conta e libera (OscReset). Robô HABILITADO. Depois: python log_ana.py logs/ARQUIVO.wpilog.
# uso: [MEC=<mecanismo>] ./testeparada.sh [N=10] [PARAM=DParadoFator] [valores="1 0.3 0"]
#      PARAM "nenhum": só as N paradas com os ajustes como estão (a bateria.sh usa assim)
cd "$(dirname "$0")" || exit 1
source ./mec.sh
N="${1:-10}"
PARAM="${2:-DParadoFator}"
VALORES="${3:-1 0.3 0}"
: "${PARADA_ALVO:?PARADA_ALVO faltando em mecanismos/$MEC.conf}"
PARTIDAS=(${PARADA_DE:-$INICIO})
./nt.sh enabled || { echo "ROBO NAO HABILITADO"; exit 1; }
if [ "$PARAM" = nenhum ]; then VALORES="atual"; ORIGINAL=""
else ORIGINAL=$(./nt.sh get $TUN/$PARAM | awk '{print $3}'); fi
volta() { [ -n "$ORIGINAL" ] && ./nt.sh set $TUN/$PARAM "$ORIGINAL" >/dev/null; }
./nt.sh setm $TUN/OscReset 1 $TUN/TestSetpointDeg $PARADA_ALVO >/dev/null
./nt.sh set $TUN/RemoteCmd 7 >/dev/null
for v in $VALORES; do
  [ -n "$ORIGINAL" ] && ./nt.sh set $TUN/$PARAM $v >/dev/null
  osc=0
  for i in $(seq 1 "$N"); do
    ./nt.sh enabled || { echo "PARADO: robo desabilitado"; volta; exit 1; }
    de=${PARTIDAS[$(( (i - 1) % ${#PARTIDAS[@]} ))]}
    ./nt.sh set $TUN/TestSetpointDeg $de >/dev/null
    python -c "import time;time.sleep(2.0)"
    ./nt.sh set $TUN/TestSetpointDeg $PARADA_ALVO >/dev/null
    python -c "import time;time.sleep(2.5)"
    if ./nt.sh get $T/isOscTripped | grep -q "= true"; then
      osc=$((osc + 1)); echo "  $PARAM $v, $i ($de>$PARADA_ALVO): OSCILOU (protecao cortou)"
      ./nt.sh set $TUN/OscReset 1 >/dev/null
    fi
  done
  echo "$PARAM $v: oscilou $osc de $N paradas em $PARADA_ALVO"
done
./nt.sh set $TUN/RemoteCmd 0 >/dev/null
volta
echo "FIM $(date +%H:%M:%S)${ORIGINAL:+ ($PARAM voltou a $ORIGINAL)}"
