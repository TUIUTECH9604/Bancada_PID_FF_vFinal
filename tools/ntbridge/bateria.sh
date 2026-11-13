#!/bin/bash
# Bateria de testes, para qualquer mecanismo: cada configuração = BASE + uma mudança, N rodadas do
# robusto.sh (rodada 1 de todas, depois rodada 2 de todas, para o aquecimento não favorecer
# ninguém). A campanha (BASE, NOMES, MUDA) fica num arquivo em campanhas/ (ver campanhas/LEIA.md).
# Depois de cada rodada (se PARADAS > 0 e PARADA_ALVO no .conf): PARADAS paradas no ponto fraco com
# a mesma configuração (testeparada.sh), para pegar quem oscila parado (o pontua.py penaliza).
# Segurança: para se o robô for desabilitado; pausa com o motor parado acima de TEMP_MAX até
# TEMP_VOLTA (do .conf do mecanismo); CONFERE todos os ajustes antes de cada rodada (lição 12);
# restaura a BASE no fim.
# uso: [MEC=<mecanismo>] ./bateria.sh campanhas/ARQ.sh pasta_saida [rodadas: "1 2"] [configs: "a b"]
# Depois: python log_rodadas.py logs/ARQUIVO.wpilog   (a 5 ms; ou python compara.py pasta_saida)
cd "$(dirname "$0")" || exit 1
source ./mec.sh
CAMP="$1"; [ -f "$CAMP" ] || { echo "uso: ./bateria.sh campanhas/ARQ.sh pasta_saida [rodadas] [configs]"; exit 1; }
PARA_SE_ABORTAR=0
PARADAS="${PARADAS_POR_RODADA:-0}"   # do .conf; a campanha pode mudar (PARADAS=0 desliga)
# shellcheck disable=SC1090
source "$CAMP"
: "${BASE:?BASE faltando na campanha}"
[ "${#NOMES[@]}" = "${#MUDA[@]}" ] || { echo "ERRO: NOMES e MUDA com tamanhos diferentes"; exit 1; }
mkdir -p "$2"
OUT="$(cd "$2" && (pwd -W 2>/dev/null || pwd))"
RODADAS="${3:-1 2}"
SO="${4:-}"
cp "$CAMP" "$OUT/campanha.sh" 2>/dev/null   # guarda junto dos resultados o que foi testado
echo "Mecanismo $MEC ($TUN), campanha $CAMP: ${NOMES[*]}"

# "k=v k2=v2" → aplica tudo numa conexão só (setm). Chave repetida: vale a ÚLTIMA (a MUDA por
# cima da BASE) — publicar o mesmo tópico duas vezes na mesma conexão não garante a ordem.
aplica() {
  local -A m=(); local ordem=() a=() k
  for kv in $1; do
    k=${kv%%=*}
    [ -z "${m[$k]+x}" ] && ordem+=("$k")
    m[$k]=${kv#*=}
  done
  for k in "${ordem[@]}"; do a+=("$TUN/$k" "${m[$k]}"); done
  [ ${#a[@]} -gt 0 ] && ./nt.sh setm "${a[@]}" >/dev/null
}
# Confere se os valores ao vivo são os esperados (BASE com a MUDA por cima)
confere() {
  MSYS_NO_PATHCONV=1 python - "$TUN" "$1" "$2" <<'PY' || return 1
import subprocess, sys
tun, base, muda = sys.argv[1], sys.argv[2].split(), sys.argv[3].split()
esp = dict(kv.split("=", 1) for kv in base)
esp.update(dict(kv.split("=", 1) for kv in muda))
out = subprocess.run(["bash", "./nt.sh", "get"] + [f"{tun}/{k}" for k in esp], capture_output=True, stdin=subprocess.DEVNULL,
                     text=True, encoding="utf-8", errors="replace").stdout
lido = {}
for linha in out.splitlines():
    if " = " in linha:
        k, v = linha.split(" = ", 1)
        lido[k.strip().rsplit("/", 1)[-1]] = v.strip()
ruim = []
for k, v in esp.items():
    try:
        ok = abs(float(lido.get(k, "nan")) - float(v)) <= 1e-9 * max(1.0, abs(float(v)))
    except ValueError:
        ok = False
    if not ok:
        ruim.append(f"{k}={lido.get(k, '?')} (esperado {v})")
if ruim:
    print("PARADO: ajustes ao vivo diferentes do esperado: " + ", ".join(ruim))
    sys.exit(1)
PY
}
temp() { ./nt.sh get $T/getMotorTempC | awk '{print $3}'; }
quente() { python -c "import sys; sys.exit(0 if float('$1') >= $2 else 1)"; }

for rodada in $RODADAS; do
  for i in "${!NOMES[@]}"; do
    n=${NOMES[$i]}
    [ -n "$SO" ] && [[ " $SO " != *" $n "* ]] && continue
    ./nt.sh enabled || { echo "PARADO: robo desabilitado (antes de $n r$rodada)"; aplica "$BASE"; exit 1; }
    t=$(temp)
    if quente "$t" "$TEMP_MAX"; then
      echo "motor a $t C: esfriando (motor parado)..."
      ./nt.sh set $TUN/RemoteCmd 0 >/dev/null
      for k in $(seq 1 60); do
        python -c "import time; time.sleep(15)"
        ./nt.sh enabled || { echo "PARADO: robo desabilitado durante o resfriamento"; aplica "$BASE"; exit 1; }
        t=$(temp); quente "$t" "$TEMP_VOLTA" || break
      done
      echo "  voltou a $t C"
    fi
    aplica "$BASE $(echo ${MUDA[$i]})"
    confere "$BASE" "${MUDA[$i]}" || { aplica "$BASE"; exit 1; }
    echo "== $n r$rodada (${MUDA[$i]:-base}) temp $t C  $(date +%H:%M:%S)"
    res=$(./robusto.sh "$OUT/${n}_r$rodada.csv" 2>&1 | grep -E "ABORTADO|tremor rapido|NAO HABILITADO")
    echo "$res"
    if [ "$PARADAS" -gt 0 ] && [ -n "$PARADA_ALVO" ] && ! echo "$res" | grep -q "ABORTADO\|NAO HABILITADO"; then
      ./testeparada.sh "$PARADAS" nenhum 2>&1 | grep -E "oscilou|PARADO|NAO HABILITADO" | sed 's/^/  paradas: /'
      ./nt.sh enabled || { echo "PARADO: robo desabilitado (paradas de $n r$rodada)"; aplica "$BASE"; exit 1; }
    fi
    aplica "$BASE"
    # A proteção do robô (OscGuard) trava o controle até desabilitar: com a BASE de volta e o motor
    # parado, libera para a próxima configuração não ficar travada (a rodada já conta como ABORTADO)
    if echo "$res" | grep -q "protecao do robo"; then ./nt.sh set $TUN/OscReset 1 >/dev/null; fi
    if [ "$PARA_SE_ABORTAR" = 1 ] && echo "$res" | grep -q "ABORTADO"; then
      echo "PARADO: $n ficou instavel (PARA_SE_ABORTAR=1)"; break 2
    fi
  done
done
aplica "$BASE"
echo "FIM $(date +%H:%M:%S), motor a $(temp) C"
