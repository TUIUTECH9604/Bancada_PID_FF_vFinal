#!/bin/bash
# ASSISTENTE DE AJUSTE de um mecanismo: mede, calcula, testa e RECOMENDA (uma pessoa aprova).
#
#   1 ref       robusto 2× com os valores de hoje (referência) e guarda os valores ao vivo
#   2 ff        kit só feedforward (RemoteCmd 11, MechTuneSoFF=1) a partir de FF_INICIO: kS/kG/kV
#   3 modelo    baixa o log a 5 ms; identifica kV/kA; kP/kD de partida pelo LQR
#   4 bateria   campanha GERADA: atual × modelo × modelo×0,7 × modelo×1,4 (+ kG do kit) × 2 rodadas
#   5 relatorio baixa o log; nota de cada configuração (+ paradas no ponto fraco); RELATORIO.md
#   6 volta2    se a melhor for a de ganho mais alto/baixo testado: +2 degraus para aquele lado,
#               bateria de novo e relatório com tudo (se a melhor está no meio, pula)
#
# uso: [MEC=<mecanismo>] ./autoajuste.sh <pasta> [etapas: "ref ff modelo bateria relatorio volta2"]
#      (robô HABILITADO nas etapas que movem: ref, ff, bateria, volta2; Disable para tudo na hora)
# NADA é gravado no código: os ajustes ao vivo voltam aos de partida no fim da bateria.
cd "$(dirname "$0")" || exit 1
source ./mec.sh
PASTA="${1:?uso: ./autoajuste.sh <pasta> [etapas]}"
ETAPAS="${2:-ref ff modelo bateria relatorio volta2}"
mkdir -p "$PASTA" logs
IP="${NT_SERVER:-169.254.29.48}"
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 -o LogLevel=ERROR)
CH="kG kS kA kP kI kD IZoneDeg IDeadbandDeg DFilterSec DParadoFator AntiFolga ProfMaxVelDegps ProfMaxAccelDegps2 VelkV"

habilitado() { ./nt.sh enabled || { echo "PARADO: robô não habilitado (etapa $1)"; exit 1; }; }
# Baixa (de novo: ele cresce) o .wpilog mais recente do robô; imprime o caminho local
baixa_log() {
  local f
  f=$(ssh "${SSH_OPTS[@]}" "lvuser@$IP" 'ls -1t /u/logs/FRC_*.wpilog /home/lvuser/logs/FRC_*.wpilog 2>/dev/null | head -1' < /dev/null)
  [ -n "$f" ] || { echo "ERRO: não achei .wpilog no robô ($IP)" >&2; return 1; }
  scp -q "${SSH_OPTS[@]}" "lvuser@$IP:$f" "logs/$(basename "$f")" < /dev/null || return 1
  echo "logs/$(basename "$f")"
}
le_atual() {
  local a=(); for k in $CH; do a+=("$TUN/$k"); done
  ./nt.sh get "${a[@]}" > "$PASTA/atual.txt"
  python autoajuste.py atual "$PASTA" "$PASTA/atual.txt"
}

for etapa in $ETAPAS; do
  echo "===== $etapa ($(date +%H:%M:%S)) ====="
  case $etapa in
    ref)
      habilitado ref
      le_atual
      for r in 1 2; do ./robusto.sh "$PASTA/ref_r$r.csv" | tail -1; done
      ;;
    ff)
      habilitado ff
      [ -f "$PASTA/estado.json" ] || le_atual
      ini="${FF_INICIO:-$INICIO}"
      echo "indo para $ini (início das passadas lentas, ±40° em volta)..."
      ./nt.sh setm $TUN/TestSetpointDeg "$ini" $TUN/RemoteCmd 7 >/dev/null
      python -c "import time;time.sleep(3)"
      ./nt.sh setm $TUN/RemoteCmd 0 $TUN/MechTuneSoFF 1 >/dev/null
      ./nt.sh set $TUN/RemoteCmd 11 >/dev/null
      fim=""
      for i in $(seq 1 40); do          # até ~4 min (cada leitura ~3 s pelo Wi-Fi + 3 s)
        python -c "import time;time.sleep(3)"
        ./nt.sh enabled || { echo "PARADO: desabilitado durante o feedforward"; break; }
        e=$(./nt.sh get /MechTune/Etapa | awk -F' = ' '{print $2}')
        echo "  kit: $e"
        if [ "$e" = "CONCLUIDO" ] || [ "$e" = "ABORTADO" ]; then fim=$e; break; fi
      done
      ./nt.sh get /MechTune/Status /MechTune/Relatorio
      ./nt.sh setm $TUN/RemoteCmd 0 $TUN/MechTuneSoFF 0 >/dev/null
      if [ "$fim" = "CONCLUIDO" ]; then
        ./nt.sh get /MechTune/Resultado/kS /MechTune/Resultado/kG /MechTune/Resultado/kV \
          /MechTune/Resultado/erroV /MechTune/Resultado/amostras > "$PASTA/ff.txt"
        python autoajuste.py ff "$PASTA" "$PASTA/ff.txt"
      else
        echo "AVISO: o feedforward pelo kit não concluiu ($fim): sigo sem ele (kG atual)"
      fi
      ;;
    modelo)
      LOG=$(baixa_log) || exit 1
      echo "log: $LOG"
      python autoajuste.py modelo "$PASTA" "$LOG" || exit 1
      ;;
    bateria)
      habilitado bateria
      python autoajuste.py campanha "$PASTA" || exit 1
      ./bateria.sh "$PASTA/campanha_auto.sh" "$PASTA/bateria" || { echo "PARADO: a bateria não terminou"; exit 1; }
      ;;
    relatorio)
      LOG=$(baixa_log) || exit 1
      echo "log: $LOG"
      python autoajuste.py relatorio "$PASTA" "$LOG"
      ;;
    volta2)
      python autoajuste.py campanha2 "$PASTA"
      case $? in
        0) ;;
        2) continue ;;          # a melhor está no meio da faixa: não precisa
        *) exit 1 ;;
      esac
      habilitado volta2
      ./bateria.sh "$PASTA/campanha_volta2.sh" "$PASTA/bateria2" || echo "AVISO: a 2ª volta parou antes do fim (ver acima): o relatório usa o que deu"
      LOG=$(baixa_log) || exit 1
      echo "log: $LOG"
      python autoajuste.py relatorio "$PASTA" "$LOG"
      ;;
    *) echo "etapa desconhecida: $etapa"; exit 1 ;;
  esac
done
