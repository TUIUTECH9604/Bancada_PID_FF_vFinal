# Campanhas de bateria

Uma campanha é um arquivo bash com três coisas (ver `bancada_kd.sh`):

```bash
BASE="kG=0.09 kP=0.006 kI=0.02 kD=0.00015 ..."   # valores do código (ficam aplicados no fim)
NOMES=(a b c)                                    # nome curto de cada configuração
MUDA=("kD=0.0003" "kD=0.00015" "kD=0.000075")    # o que cada uma muda em relação à BASE
PARA_SE_ABORTAR=0                                # 1 = para no 1º "ABORTADO" (subida de ganhos)
PARADAS=5                                        # paradas no PARADA_ALVO depois de cada rodada
                                                 # (padrão: PARADAS_POR_RODADA do .conf; 0 = não faz)
```

As chaves são os ajustes ao vivo do mecanismo (`/Tuning/<NOME>/<chave>`, os nomes de
`MechanismSubsystem`: `kP kI kD kG kS kV kA IZoneDeg IDeadbandDeg DFilterSec OutMax DParadoFator
ProfMaxVelDegps ProfMaxAccelDegps2 AntiFolga ...`).

Rodar (robô HABILITADO):

```bash
MEC=bancada ./bateria.sh campanhas/bancada_kd.sh bat_20261112kd     # rodadas 1 e 2
MEC=bancada ./bateria.sh campanhas/bancada_kd.sh bat_X "1" "kd05"   # só a rodada 1 de kd05
```

Antes de CADA rodada a bateria confere se todos os ajustes ao vivo são os esperados (BASE com a
MUDA por cima) e para se não forem (lição 12). Avaliar pelo log a 5 ms do pendrive:
`python log_rodadas.py logs/ARQUIVO.wpilog` (lição 19).

Regras que valeram na bancada:
- Mudar UMA coisa por vez e repetir 2 rodadas (lição 3).
- Subida de ganhos em degraus (×1,5 a ×2) com `PARA_SE_ABORTAR=1` (lição 15).
- Feedforward primeiro (kG, kV, kA), PID depois.
- Uma pessoa aprova antes de gravar no código; depois deploy e robusto 2× vindo do código.

Histórico das campanhas da bancada: `HISTORICO_bancada.md`.
