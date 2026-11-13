#!/bin/bash
# uso: [MEC=<mecanismo>] ./robusto.sh arquivo.csv
# O TESTE QUE VALIDA GANHOS, para qualquer mecanismo com o MechanismSubsystem: percorre os ALVOS
# do mecanismos/<MEC>.conf (passos pequenos E movimentos grandes, lição 2), com perfil (RemoteCmd
# 7). PARADA AUTOMÁTICA pelo Wi-Fi: 4+ inversões de sentido (>0,7) em 1 s, duas vezes seguidas →
# desliga o comando. A proteção do PRÓPRIO robô (laço de 5 ms, OscGuard) pega a oscilação rápida
# que o Wi-Fi não vê. Robô HABILITADO. Imprime "ABORTADO" se ficou instável.
cd "$(dirname "$0")" || exit 1
source ./mec.sh
# Caminho no formato C:/... no Git Bash: o nt.sh desliga a conversão de caminhos e o Java no
# Windows não entende /c/... (assim a vigia não gravava e a parada automática não funcionava)
D="$(pwd -W 2>/dev/null || pwd)"
./nt.sh enabled || { echo "ROBO NAO HABILITADO"; exit 1; }
# A gravação termina quando o último alvo acaba (fim.flag), não num tempo fixo: pelo Wi-Fi cada
# ./nt.sh leva ~3 s e com tempo fixo os últimos alvos ficavam fora. DUR é só o teto de segurança.
DUR=900
rm -f "$D/abortou.flag" "$D/fim.flag"
./nt.sh set $TUN/TestSetpointDeg $INICIO >/dev/null
NT_STOPFILE="$D/fim.flag" ./nt.sh rec $DUR "$1" $T/getGoalDeg $T/getQuadDeg $T/getDeg \
  $T/getCommandedVolts $T/getProfileVelDegps >/dev/null & R=$!
( ruim=0; while kill -0 $R 2>/dev/null; do
    rm -f "$D/vigia.csv"
    ./nt.sh rec 1 "$D/vigia.csv" $T/getQuadDeg >/dev/null 2>&1
    if [ ! -f "$D/vigia.csv" ]; then
      # sem vigia não há parada automática: para o teste em vez de seguir sem proteção
      ./nt.sh set $TUN/RemoteCmd 0 >/dev/null; touch "$D/abortou.flag" "$D/fim.flag"
      echo "ABORTADO: a vigia nao conseguiu gravar (sem parada automatica)"; break
    fi
    inv=$(python osc.py "$D/vigia.csv" 2>/dev/null)  # 2>/dev/null: ao matar a vigia no fim, o python reclamava do pipe fechado (OSError 22)
    if [ "${inv:-0}" -ge 4 ]; then ruim=$((ruim+1)); else ruim=0; fi
    if [ $ruim -ge 2 ]; then
      ./nt.sh set $TUN/RemoteCmd 0 >/dev/null; touch "$D/abortou.flag" "$D/fim.flag"
      echo "ABORTADO: oscilando ($inv inversoes em 1 s)"; break
    fi
  done ) & W=$!
./nt.sh set $TUN/RemoteCmd 7 >/dev/null
python -c "import time;time.sleep(2.5)"
for a in $ALVOS; do
  [ -f "$D/abortou.flag" ] && break
  ./nt.sh set $TUN/TestSetpointDeg $a >/dev/null
  python -c "import time;time.sleep($ESPERA-0.15)"
done
python -c "import time;time.sleep($ESPERA)"  # o último alvo também assenta antes de parar
./nt.sh set $TUN/RemoteCmd 0 >/dev/null
touch "$D/fim.flag"
wait $R 2>/dev/null
kill $W 2>/dev/null
# Proteção do próprio robô (laço de 5 ms): pega oscilação rápida que a vigia pelo Wi-Fi não vê
if ./nt.sh get $T/isOscTripped 2>/dev/null | grep -q "= true"; then
  echo "ABORTADO: a protecao do robo cortou o motor (oscilacao rapida); desabilite para liberar"
  touch "$D/abortou.flag"
fi
if [ -f "$D/abortou.flag" ]; then echo "RESULTADO: ABORTADO (instavel)"; rm -f "$D/abortou.flag"; else python mapa_ana.py -r "$1"; fi
rm -f "$D/vigia.csv" "$D/fim.flag"
exit 0
