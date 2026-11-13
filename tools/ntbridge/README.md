# Bateria de testes de mecanismos (ponte NetworkTables + scripts)

Ferramentas de linha de comando (Git Bash, Windows) para testar e ajustar **qualquer mecanismo**
que use o `MechanismSubsystem` do robô: ler e escrever ajustes ao vivo, rodar os testes padrão,
comparar configurações e analisar o log a 5 ms. Nasceram na bancada do ponteiro (2026-11) e
foram tornadas genéricas em 2026-11-13.

## Instalação (uma vez por computador)

Precisa do WPILib 2026 (em `C:\Users\Public\wpilib\2026`) e de Python 3 com `numpy`
(`python -m pip install --user numpy`).

```bash
cd tools/ntbridge
./setup.sh        # copia as bibliotecas do WPILib para libs/ e compila a ponte
```

`libs/`, `out/`, `logs/` e os `.csv` não vão para o git: o `setup.sh` recria tudo.
Robô pelo Wi-Fi: `export NT_SERVER=10.96.4.2` (o padrão é `169.254.29.48`, a bancada por cabo).

## Escolher o mecanismo: `MEC=<nome>`

Cada mecanismo tem um arquivo `mecanismos/<nome>.conf` (veja `mecanismos/bancada.conf`, que
explica cada linha): nome no código (`/Tuning/<NOME>/`, `/Mech5ms/<NOME>/`), onde está a
telemetria, unidade, se dá a volta (`CIRCULAR`), os **alvos do teste robusto** (passos pequenos
E grandes, dentro do curso seguro), o que conta como movimento grande, o ponto fraco para o teste
de parada e os limites de temperatura. Todos os scripts abaixo aceitam `MEC=...` antes do
comando; sem ele, `bancada`.

Mecanismo novo: copie `bancada.conf`, troque os valores (principalmente `NOME`, `TELEM`, `ALVOS` e
`PARADA_ALVO`) e confira com `MEC=novo ./nt.sh get /Tuning/<NOME>/kP` (robô ligado) que o nome bate.
O roteiro completo (sensor → segurança → feedforward → perfil → PID → validação) está em
`docs/METODO_SUBSISTEMAS.md`.

## Ponte

```bash
./nt.sh list /Tuning                                  # tópicos e valores
./nt.sh get /Tuning/Dial/kP Robot/m_robotContainer/mechanism/getDeg
./nt.sh set /Tuning/Dial/kP 0.003
./nt.sh setm /Tuning/Dial/kP 0.003 /Tuning/Dial/kD 0.0001   # vários numa conexão (~3 s no total)
./nt.sh rec 10 dados.csv Robot/m_robotContainer/mechanism   # grava 10 s (t,topico,valor)
./nt.sh enabled && echo habilitado
```

Os tópicos do Epilogue ficam **sem barra inicial**. Pelo Wi-Fi cada chamada leva ~3 s.

## Testes (o mecanismo se MOVE — robô habilitado em Teleop; Disable para tudo na hora)

Usam o comando remoto `/Tuning/<NOME>/RemoteCmd` (7 = seguir `TestSetpointDeg` com perfil), que
só age habilitado e volta a 0 ao desabilitar. Além da vigia pelo Wi-Fi, o próprio robô tem a
**proteção contra oscilação** no laço de 5 ms (corta o motor; `isOscTripped`).

| Script | O que faz |
|---|---|
| `./robusto.sh arq.csv` | **o teste de referência**: percorre os `ALVOS` do .conf com perfil. Parada automática se oscilar. Imprime o resumo ou "ABORTADO" |
| `./bateria.sh campanhas/X.sh pasta` | **compara configurações**: BASE + uma mudança, 2 rodadas de cada (alternadas), conferindo TODOS os ajustes antes de cada rodada. Ver `campanhas/LEIA.md` |
| `./testeparada.sh [N] [PARAM] ["valores"]` | N paradas no ponto fraco (`PARADA_ALVO`) por valor de PARAM (padrão `DParadoFator`), contando oscilações |
| `./sweep.sh nome "kD=... kI=..."` | aplica ajustes ao vivo (ficam aplicados) e roda o robusto → `rb_nome.csv` |
| `./mapa.sh arq.csv` | passos curtos por todo o curso (`MAPA` do .conf) |

Exemplo:

```bash
export NT_SERVER=10.96.4.2
MEC=bancada ./robusto.sh ref_r1.csv
MEC=bancada ./bateria.sh campanhas/bancada_kd.sh bat_20261112kd
```

## Assistente de ajuste: `autoajuste.sh` (mede, calcula, testa e RECOMENDA)

```bash
export NT_SERVER=10.96.4.2
MEC=bancada ./autoajuste.sh aa_20261113                     # tudo (~1–1,5 h, robô habilitado)
MEC=bancada ./autoajuste.sh aa_20261113 "modelo relatorio"   # só as etapas de cálculo
```

| Etapa | O que faz | Robô |
|---|---|---|
| `ref` | guarda os ajustes ao vivo de partida e roda o robusto 2× (referência) | move |
| `ff` | kit **só feedforward** (RemoteCmd 11 com `MechTuneSoFF = 1`) a partir de `FF_INICIO`: passadas lentas nos dois sentidos → kS, kG, kV. Não aplica nada | move |
| `modelo` | baixa o log a 5 ms; `identifica.py` mede kV e kA nos movimentos normais; `ganhos.py` calcula kP/kD de partida pelo LQR (critérios `LQR_*` do .conf) | — |
| `bateria` | gera `campanha_auto.sh` (atual × modelo × modelo×0,7 × modelo×1,4, e o kG do kit se diferir >15%) e roda 2 rodadas de cada; depois de cada rodada, `PARADAS_POR_RODADA` paradas no ponto fraco (`PARADA_ALVO`) | move |
| `relatorio` | baixa o log; `pontua.py` dá a nota de cada configuração (pesos `NOTA_PESOS` do .conf; parada que oscilou pesa muito e tira a configuração da recomendação); escreve `RELATORIO.md` com a recomendação e a linha para a `MechanismConfig` | — |
| `volta2` | se a melhor foi a de ganho MAIS ALTO testado (ou mais baixo): testa ela de novo + 2 degraus para aquele lado (× `VOLTA2_FATOR` e ×²; subindo, em ordem e parando na 1ª que oscilar) e refaz o relatório com tudo. Se a melhor está no meio da faixa, pula | move |

Validado na bancada em 2026-11-13 (`aa_20261113`): recomendou ficar com os valores do código, os
mesmos achados à mão (relatório no CLAUDE.md). O teste de parada na bateria e a 2ª volta vieram
depois, a partir daquele resultado (a melhor nota foi a de kP mais alto) — falta uma rodada no robô.

Nada é gravado no código: **uma pessoa lê o relatório e aprova**; depois grava na
`MechanismConfig`, faz deploy e confirma com o robusto 2× vindo do código (e o `testeparada.sh`).
Peças avulsas: `python identifica.py LOG` (kS/kG/kV/kA de um log), `python ganhos.py kV kA`
(LQR), `python pontua.py LOG` (nota e recomendação de qualquer bateria).
O que se aprendeu na bancada: kV e kA saem bem do log de movimentos normais (kV 0,028–0,036 V/RPM
contra 0,0318 do código; kA 0,00005–0,00008); kG e kS NÃO (se confundem: precisam das passadas
lentas do kit). O LQR com o kV/kA do log deu kP 0,0060–0,0068 (o da bancada, achado por bateria, é
0,006) e kD ~0,000016 (o do código é 0,00015; na bateria, menos D foi melhor).

## Análise — decida pelo log a 5 ms (lição 19)

O robô grava o laço de controle **a cada 5 ms** no `.wpilog` (`/Mech5ms/<NOME>/`: posição, alvo,
destino, perfil, FF, P, I, D, tensão, corrente, bateria). Por padrão só com os logs num
**pendrive** na roboRIO (`/Tuning/<NOME>/FastLog = 1` força no disco interno).

```bash
./logs_get.sh                                    # baixa os 3 logs mais recentes para logs/
python log_rodadas.py logs/ARQ.wpilog            # UMA LINHA POR RODADA do robusto/bateria:
                                                 #  ajustes que mudaram (descobre sozinho), passada
                                                 #  média/máx, nos GRANDES, erro final, chegada,
                                                 #  tremor, 3 piores movimentos, bateria
python log_rodadas.py logs/ARQ.wpilog --csv r.csv  # idem + tabela para comparar
python log_ana.py logs/ARQ.wpilog [--de S --ate S] # movimento a movimento, com P/I/D no fim
python dslog_bateria.py 2026_11_12               # bateria minuto a minuto pelos logs do Driver Station
```

Pelo Wi-Fi (sem baixar o log), para acompanhar: `python mapa_ana.py -r arq.csv` e
`python compara.py pasta_da_bateria`. Eles perdem os picos curtos: não decida só por eles.

Os `.wpilog` também abrem no AdvantageScope (aba Mechanism: `NT:/SmartDashboard/<NOME>/Mecanismo`;
Line Graph: `/Mech5ms/<NOME>/deg` × `targetDeg`).

## SysId (kS, kG, kV, kA)

`./nt.sh set /Tuning/<NOME>/RemoteCmd 9`, espere `/SysId/Status` = "Concluído", volte a 0 e rode
`./sysid_get.sh`. Na bancada o kS e o kA do SysId atrapalharam como feedforward (lições 4 e 15);
o kG medido segurando nas horizontais e decidido por bateria foi o que valeu (lição 18).

## Antigos (só da bancada)

`kdtest.sh`, `proftest.sh`, `holdtest.sh`, `motiontest.sh` têm `/Tuning/Dial` fixo; o `robusto.sh`
e a `bateria.sh` substituem. `teste270.sh` = `testeparada.sh` da bancada.
