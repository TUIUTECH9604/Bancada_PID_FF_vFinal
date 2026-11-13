# Campanha: kD da bancada (8ª bateria 15:1, bat_20261112kd, 2026-11-12).
# O ponteiro mais pesado batia na folga DURANTE o movimento (~10 Hz) e o D realimentava.
# Resultado (log a 5 ms): kD 0,00015 → grandes 0,86/1,01° (era 1,9–2,7°); gravado no código.
# uso: MEC=bancada ./bateria.sh campanhas/bancada_kd.sh bat_XXXX

# BASE = valores do código (é o que fica aplicado no fim).
BASE="kG=0.09 AntiFolga=0 DParadoFator=0.3 kP=0.006 kI=0.02 kD=0.00015 IDeadbandDeg=0.5 kA=0.00005 ProfMaxAccelDegps2=4000 ProfMaxVelDegps=720"
# Cada configuração: um nome curto (vira o nome dos arquivos) e o que muda em relação à BASE.
NOMES=(kd1 kd05 kd025)
MUDA=("kD=0.0003"
      "kD=0.00015"
      "kD=0.000075")
# 1 = numa subida de ganhos, o 1º "ABORTADO" encerra a campanha (não testa os maiores).
PARA_SE_ABORTAR=0
