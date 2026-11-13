#!/bin/bash
# Carrega a configuração do mecanismo para os scripts de teste ("source mec.sh").
#   MEC=<nome> escolhe mecanismos/<nome>.conf (padrão: bancada).
# Define: NOME, TELEM, UNIDADE, CIRCULAR, INICIO, ALVOS, ESPERA, GRANDE, TOL, PARADA_ALVO,
# PARADA_DE, TEMP_MAX, TEMP_VOLTA e os atalhos TUN=/Tuning/<NOME> e T=<TELEM>.
_MEC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MEC="${MEC:-bancada}"
_MEC_CONF="$_MEC_DIR/mecanismos/$MEC.conf"
if [ ! -f "$_MEC_CONF" ]; then
  echo "ERRO: não achei $_MEC_CONF (MEC=$MEC). Mecanismos: $(ls "$_MEC_DIR/mecanismos" | sed 's/\.conf$//' | tr '\n' ' ')"
  exit 1
fi
# shellcheck disable=SC1090
source "$_MEC_CONF"
: "${NOME:?NOME faltando em $_MEC_CONF}" "${TELEM:?TELEM faltando}" "${ALVOS:?ALVOS faltando}"
UNIDADE="${UNIDADE:-graus}"; CIRCULAR="${CIRCULAR:-1}"; ESPERA="${ESPERA:-2.6}"
GRANDE="${GRANDE:-60}"; TOL="${TOL:-1.0}"; TEMP_MAX="${TEMP_MAX:-55}"; TEMP_VOLTA="${TEMP_VOLTA:-45}"
INICIO="${INICIO:-$(echo $ALVOS | awk '{print $1}')}"
TUN="/Tuning/$NOME"
T="$TELEM"
export MEC
