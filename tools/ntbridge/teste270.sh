#!/bin/bash
# Atalho antigo (2026-11-12): paradas em 270° da bancada variando o DParadoFator.
# Agora é o testeparada.sh genérico (o ponto fraco vem do PARADA_ALVO do .conf).
# uso: ./teste270.sh [N=10] [fatores="1 0.3 0"]
cd "$(dirname "$0")" || exit 1
MEC="${MEC:-bancada}" exec ./testeparada.sh "${1:-10}" DParadoFator "${2:-1 0.3 0}"
