# Bancada PID/FF — equipe TUIUTECH 9604

Contexto para continuar o trabalho em qualquer computador (o Claude Code lê este arquivo
automaticamente). Conversa em **português**. Última atualização: 2026-11-12.

## Roteiro para a próxima conversa (LER PRIMEIRO)

O trabalho começou num notebook e continua em outro: a conversa e a memória anteriores NÃO
vieram junto — este arquivo é o que sobrou delas. Então:

0. **Descubra quem é e o que quer.** Se for outra pessoa ou **outro projeto/subsistema**
   (braço, elevador, shooter...), o guia é `docs/METODO_SUBSISTEMAS.md`: siga a ordem dele
   (entender o mecanismo → sensor → segurança → feedforward → perfil → PID → validação robusta →
   registrar). Os números desta bancada não servem para outro mecanismo; o método, sim.
1. **Cumprimente e situe o usuário**: diga em 2–3 linhas onde paramos (seção "Estado atual") e
   pergunte o que ele quer fazer. Não refaça testes já feitos sem motivo.
2. **Confira o ambiente** antes de usar o robô (uma vez por computador):
   - WPILib 2026 instalado? (`C:/Users/Public/wpilib/2026/jdk` no Windows; se estiver em outro
     lugar, use esse caminho no `JAVA_HOME` e no `WPILIB_HOME` do `setup.sh`).
   - `./gradlew build` e `./gradlew test` passam?
   - `tools/ntbridge/setup.sh` já rodou? (cria `libs/` e `out/`, que não vão para o git).
   - Python 3 com `numpy` (para as análises).
3. **Com o robô**: siga "Como trabalhar com o robô" — o usuário habilita/desabilita, deploy só
   desabilitado, explicar antes de mover o motor.
4. **Antes de mudar ganhos**, rode `tools/ntbridge/robusto.sh` na configuração atual para ter a
   referência; compare cada mudança com ele (de preferência 2 rodadas cada).
5. **Ao terminar algo**: commit em português explicando o porquê e os números medidos, e
   atualize este arquivo (estado, ganhos, lições). O push é o usuário que faz pelo GitHub
   Desktop — avise quando houver commits para enviar.
6. Envio: clone em `C:/CODE/2026/BANCADA/Bancada_PID_FF_vFinal` (ver "O que é"), que já usa a conta
   da equipe no push. Se der 403 (outro computador), é a conta do git sem permissão no
   repositório da equipe: peça para o usuário enviar pelo GitHub Desktop.

## Estado atual (2026-11-12) — REDUÇÃO 15:1; ponteiro com parafusos, kG 0,09, kD 0,00015, D parado 0,3

Em 2026-11-11 o NEO passou a mover o ponteiro por **3:1 × 5:1 (MAXPlanetary) + caixa de 90°
(1:1) = 15:1**, com eixo mais longo (o ponteiro continua com os +220 g de 2026-11-10). O Through
Bore ficou **depois** das caixas (no eixo do ponteiro). **Folga total ~5,5° no ponteiro** (soma
dos estágios, não tem como tirar). A caixa segura o ponteiro parado em quase todos os ângulos,
mas **na horizontal pode escorregar** (2026-11-12: em coast, foi de 0° a 330° sozinho).

**2026-11-12: ponteiro remontado com mais parafusos → zero 301,34 e kG 0,09 V** (ver "Ponteiro com
parafusos" abaixo). Antes disso: zero 270,91; kG **0,063 V**, kS 0,099 V e kV **0,0318 V/RPM do
ponteiro** (rampas com termo de velocidade, conferidos pelo SysId); limite **20 A**; ganhos
**kP 0,006 / kI 0,02 / kD 0,0003** (2× os antigos, escolhidos por bateria); perfil
**720°/s, 4000°/s² com kA 0,00005** ("a05", escolhido por bateria; antes 360/2000 com kA 0).

Resultado do f2 (`bat_20261111b`, 2 rodadas, ganhos ao vivo): tremor parado 0,10° méd /
0,79° máx (0–1 de 19 pontos), assenta 0,58 s, passa do alvo 0,25° méd / 1,9° máx, erro final
0,25°. Igual ou melhor que a referência do ponteiro leve sem redução.

**Confirmado no robô com os ganhos vindos do código** (deploy, `robusto.sh` 2×,
`bat_20261111c`): tremor parado 0,04–0,09° méd (0–1 de 19 pontos), assenta 0,42–0,66 s, passa do
alvo 0,3–0,5° méd (máx. 3,2° uma vez, 90°→270°: a folga em 270° fica sem gravidade), erro final
0,24°. Motor a 24–25 °C (com 15:1 quase não esquenta).

Perfil (`bat_20261111d` e `e`, 2 rodadas cada, ganhos acima): 360/2000 passava 0,35° méd do
alvo; 720/4000 com kA 0 passava 1,45°; **com kA 0,00005, 0,65° méd / 2,3° máx, ~20–25% mais
rápido** (90° em ~0,78 s, 180° em ~0,90 s contando o assentar). kA 0,0002 foi o mais preciso
(0,45° méd / 0,9° máx) mas assentava mais devagar (mesmo tempo total que 360/2000): alternativa
se precisão importar mais. 720/6000 passava 2°; acima de 720°/s o kV passa de 4 V
(`kMaxFFVolts`).

**a05 confirmado vindo do código** (deploy, `robusto.sh` 2×, `bat_20261111f`): passa do alvo
0,9° méd / 3,0° máx (um pouco acima da bateria, 0,65° / 2,3°), tremor parado 0,06–0,07°, assenta
0,5–0,6 s, erro final 0,24°, motor 25 °C. Se a passada incomodar: kA 0,0002 (mais preciso, mesmo
tempo total de 360/2000) ou voltar a 360/2000 com kA 0.

Deploy: o usuário liberou sempre (robô desabilitado); a permissão está em
`.claude/settings.local.json` (local, fora do git), com `JAVA_HOME` — rodar `./gradlew deploy`
sozinho, a partir da raiz do projeto. Pasta baixada como .zip (não é
repositório git): levar os arquivos para o clone do GitHub Desktop.

**Limites de curso (2026-11-11) — PENDENTE.** Código pronto e testado na simulação
(`MechanismLimitFinderSimTest`): `/Tuning/Dial/LimitsOn`, `LimitLowDeg`, `LimitHighDeg`
(batentes) e `LimitMarginDeg` (folga, 10°); com limites o alvo é cortado na faixa, o perfil anda
por DENTRO dela e tensão/velocidade não empurram para fora; SysId bloqueado. **No robô os limites
estão DESLIGADOS** (LimitsOn 0). A busca automática (`MechanismLimitFinder`, **RemoteCmd 10**) foi um
fracasso: com 15:1 empurrou forte demais (0,44 V já basta), **arrancou um parafuso** e o usuário
teve que segurar o ponteiro. **NÃO usar o RemoteCmd 10 com parafuso solto.** Próximo passo
combinado: marcar os batentes **à mão** (robô desabilitado, encostar de leve, segurar 3 s, ler
`getDeg`), gravar Low/High no código com folga e ligar LimitsOn. Em 2026-11-12 a busca ficou
bem mais suave (8°/s, 0,5 V, 5 A, detecção 0,12 s, ganho de velocidade 0,003 em vez de 0,02 —
o antigo era ~10× alto para 15:1 e chegava oscilando); achou os batentes na simulação. Mesmo
assim, só com batente firme.

**Disco da roboRIO: RESOLVIDO (2026-11-12).** Pendrive ROBOLOG (FAT32, 30 GB, capacidade
duvidosa "VendorCo") na USB da roboRIO, montado em `/u` (`/media/sda1`); os logs novos vão para
`/u/logs`. `/home/lvuser/logs` foi esvaziado a pedido do usuário, sem baixar: 767 MB da bancada
e 2,0 GB de logs da temporada (pastas `2026-01..08` e `BRSP_*`). Disco interno: 2,8 GB livres.
Ficaram em `/home/lvuser` `.jar` antigos de outros projetos (SUNSHINE, drivebase..., ~100 MB),
que não foram mexidos. Deploy com o nome novo feito e rodando.

**Camada IO e logs a 5 ms (2026-11-11, só simulação — falta conferir no robô).** Objetivo do
usuário: a bancada vira o laboratório da equipe para QUALQUER mecanismo. Etapa 1 feita:
  - `MechanismIO` (interface, formato do AdvantageKit) + `MechanismIOSparkMax` (fiação, CAN, inversões,
    configuração do Spark, simulação). O `MechanismSubsystem` não acessa mais hardware: lê
    `MechanismIO.Inputs` uma vez por ciclo de 5 ms. Trocar motor/sensor = nova implementação de
    `MechanismIO`. Simulação deu números IDÊNTICOS aos de antes da mudança.
  - `MechanismFastLog`: grava `/Mech5ms/...` a cada 5 ms no `.wpilog` (só habilitado/em movimento; por
    padrão só com logs no pendrive, `/Tuning/Dial/FastLog`). Teste: `MechanismFastLogSimTest`.
  - `tools/ntbridge/`: `wpilog.py` (leitor), `log_ana.py` (análise por movimento),
    `logs_get.sh` (baixa), `logs_limpa.sh` (limpa o disco interno, só com `APAGAR`).
  - Testes: `forkEvery = 1` no `build.gradle` (cada classe de teste num processo; a REVLib não
    deixa dois SparkMax com o mesmo CAN ID no mesmo processo).
**Conferido no robô em 2026-11-12** (deploy com camada IO + `MechanismConfig` + nomes novos,
`robusto.sh` 2×, `bat_20261112`): passa do alvo 1,0/0,8° méd (máx 2,2/1,9), tremor parado
0,05/0,09°, assenta 0,67/0,59 s, erro final 0,17/0,25° — igual a `bat_20261111f`. O log a 5 ms
no pendrive (`/Mech5ms`, 66.744 amostras, dt 5,0 ms, sem buracos) pelo `log_ana.py`: passa
1,28° méd / 3,66° máx (maior que o `mapa_ana` porque pega os picos que o NT a 20 ms perde), erro
final 0,22°. O `log_ana.py` foi corrigido: juntava o trecho sem alvo (comando terminado) ao
último movimento.
**Achado: o ponteiro NÃO fica parado em qualquer ângulo.** O modo do motor fora dos testes é
COAST (só o SysId liga o BRAKE). Ao fim da rodada 1 (RemoteCmd 0, 0 V), o ponteiro estava em 0°
(horizontal) e escorregou sozinho até 329,6° em ~1,5 s: o atrito da caixa não bastou ali.
**Mudado (2026-11-12, decisão do usuário): BRAKE por padrão** (`MechanismConfig.brakeWhenStopped`,
true na BANCADA). O X (`release()`) solta em COAST até o próximo movimento e funciona também
desabilitado (para mover com a mão / pendurar antes de calibrar o zero). O SysId continua em
BRAKE durante a sessão. O Spark só é reconfigurado quando o modo muda (`applyIdleMode` no
`periodic`). Telemetria: `mechanism/isBrakeMode`. O brake do NEO só curto-circuita as bobinas:
freia, mas não trava (na simulação desce ~2°/s na horizontal). **No robô segura:** parado em BRAKE
nas duas horizontais, 10 s cada (`bat_20261112/brake_0.csv` e `brake_180.csv`): 0° ficou entre
−0,39° e 0,00° (ruído de 1 passo do absoluto, 0,35°) e 180° variou 0,03°. Com 15:1, o brake +
o atrito da caixa seguram de verdade.
Etapa 2 feita (só simulação): `MechanismConfig` reúne o que muda por mecanismo (nome = pasta
/Tuning/{nome}, redução, gravidade NONE/COSINE/CONSTANT, zero, corrente, limites, ganhos e perfil
padrão; `BANCADA` = valores validados). `new MechanismSubsystem(config, io)`. Simulação idêntica;
`MechanismConfigSimTest` roda uma torre sem gravidade no mesmo código. Guia em
`docs/METODO_SUBSISTEMAS.md`, "Usar o código desta bancada em outro mecanismo".
Etapa 3 feita (só simulação): **`MechanismTuner`, RemoteCmd 11** — ajuste automático genérico
numa faixa de ±40° em volta de onde o mecanismo está (respeita limites; não procura batente nem
exige posição de repouso): confere o sentido → anda devagar (10 e 25°/s, os dois sentidos,
freando em malha fechada antes de virar) e ajusta kS/kG/kV com o modelo de gravidade da config →
ganhos da config × 0,5…4 com movimentos curtos e parada por oscilação → escolhe o melhor com
margem de 1,5× do primeiro que oscilou → aplica AO VIVO e publica /MechTune/Relatorio (com a
linha `.pid(...).feedforward(...)`). Simulação: bancada com kG zerado → mediu 0,063 (exato); torre
sem gravidade → kS 0,081 (física 0,077). Ressalva: a simulação não tem a folga nem a inércia da
caixa — lá nada oscilou até ×4, no robô ×3,5 oscilou. **Validar no robô antes de confiar.** A
malha de velocidade do kit usa ganho ~10× menor que a da busca de limites (que oscilava: com 15:1,
1 V ≈ 190°/s). Próximas: (3b) kit: perfil/kA e validação robusta dentro dele; mecanismos lineares; (4) precisão: chegar sempre pelo mesmo lado da folga. AdvantageKit completo só se o replay fizer
falta (o laço de 5 ms em `addPeriodic` complica o replay).

**Ponteiro com parafusos (2026-11-12).** O usuário mexeu no ponteiro e pôs mais peso (parafusos).
Pendurado em 270° na tabela, o sensor lia 300,4°: o zero foi recalibrado (ZeroNow, desabilitado)
para **301,34**. kG: as rampas (RemoteCmd 8, só kG/kS) deram 0,073 / kS 0,120, mas o ponteiro
**passou da horizontal dos dois lados** (270°→18° e depois →131,7°, até 60°/s, 0,55 V; o "chegou"
de 345° passou e a tensão desce devagar) — medição e trajeto não confiáveis (lição 18). Segurando
em 0° e 180° com o controle de posição, chegando pelos dois lados (`bat_20261112/kg_hold.csv`):
0° +0,16/+0,16/+0,16/+0,07 V, 180° −0,07/−0,06/−0,16/−0,14 V → **kG 0,123** (atrito parado ±0,05 V).
Bateria (`bat_20261112kg`, 2 rodadas): 0,063 / 0,09 / 0,123 → passa do alvo 0,7/0,65/0,65° méd
(máx 2,0/2,3/3,15°), erro final 0,27/0,22/0,18°, assenta 0,50/0,50/0,45 s; em 90°→180° (horizontal)
o 0,123 passou 0,0° e parou a 0,01° (o 0,063 passava 1,4/0,7° e parava +0,36°). Máximos de 2,9–3,4°
do 0,123 foram em passos de 15° perto de 90°. Gravado 0,123 (usuário aprovou) e conferido vindo
do código (`bat_20261112/kg123codigo_r1/r2`): passou **3,8/3,4° em 60→75 e 1,5/4,6° em 75→90** e a
vantagem em 180° não se repetiu (r1: 1,1°, +0,34°). Nos passos de subida perto de 90° a passada
CRESCE com o kG (pior caso: 0,063 → 1,1°; 0,09 → 2,2°; 0,123 → 3,4–4,6°): não é a folga ao acaso;
a medida "segurando" foi inflada pelo atrito parado. **ERRADO — corrigido pelo log a 5 ms:** o
kG 0,09 vindo do código também passou 4,1–4,5° (mapa_ana) e, medindo TODAS as rodadas do dia a
5 ms (`tools/ntbridge/log_rodadas.py`, separa cada robusto do .wpilog e anota o kG), a passada de
**3–5° em 60→75 e 75→90 aparece em todas**, inclusive com o ponteiro ANTIGO de manhã (3,66/3,62°),
e quase não depende do kG: 0,063 → 3,1–3,6°; 0,09 → 3,4–4,1°; 0,123 → 3,4–4,3° (código: 4,1–5,0°).
O mapa_ana (NT pelo Wi-Fi) perde esses picos ao acaso — a "tendência" era isso (lição 19). A 5 ms:
passa méd 0,8–1,1° em todos; erro final 0,26 / 0,22 / 0,17° (0,063 / 0,09 / 0,123). A passada perto
de 90° é a folga (~5,7°, gravidade ≈ 0 lá em cima) → item (4), chegar pelo mesmo lado da folga.
**Fica kG 0,09** (no código e no robô; o 0,123 não mostrou vantagem clara). Folga medida mexendo
à mão com o robô desabilitado (`bat_20261112/folga.csv`): 5,6–5,7° em 13 viradas, igual a antes.
A física do `MechanismSim` continua com kG 0,063.

**Anti-folga (2026-11-12) — item (4), FEITO na simulação, falta validar no robô.** O log a 5 ms
de um 60→75 mostrou a causa da passada de 3–5° perto de 90°: na frenagem do perfil a tensão vai a
~0 (o freio pede kA·4000 ≈ 0,2 V e a gravidade a 75° só dá ~0,02 V para manter o dente
encostado), o ponteiro solta e atravessa a folga (até 79,1°), bate e oscila ~0,8 s. Solução:
`MechanismConfig.antiBacklash(on, deg, fatorMax)` / ao vivo `/Tuning/Dial/AntiFolga`,
`AntiFolgaDeg` (8°), `AntiFolgaFatorMax` (0,7). Quando o destino tem gravidade fraca
(0,05 < |cos| < 0,7), o movimento chega CONTRA a gravidade e o passo é ≥ 3°, o perfil vai até
destino + 8° (no sentido contrário à gravidade) e volta A FAVOR dela: frear e segurar usam o mesmo
lado do dente. Na vertical exata (|cos| < 0,05, ex.: 90°/270°) não há lado preferido: vai direto.
`getGoalDeg` continua sendo o destino final; `isApproaching` = indo ao ponto intermediário;
`tunerProfileDone`/`isAtGoal` só valem no destino. `MechanismAntiFolgaSimTest` confere quando
acontece. **Na BANCADA está desligado** até a bateria `bat_20261112af` (af0 × af8, 2 rodadas;
no robusto afeta 45>60 e 60>75). `log_ana.py` e `mapa_ana.py` agora medem a passada só depois que
o perfil PAROU de vez (o robusto grava `getProfileVelDegps`); gravações antigas dão o mesmo.
**Resultado no robô (`bat_20261112af`, log a 5 ms `FRC_20261112_185653`):** 60>75 caiu de 4,12°
para 2,93/2,94° (não para ~1°); 45>60 não melhorou (af0 2,55/1,13°, af8 2,69/2,28°). Ganho pequeno:
**continua desligado**. Hipóteses: 8° pouco ou a ida de 8° já atravessa a folga de novo ao frear.
**⚠️ ACHADO: oscilação a 12,5 Hz, ±3,5°, parado em 270° (pendurado).** Na gravidade zero o
ponteiro fica solto na folga e o **D** (pp 1,6–1,7 V; P só 0,5 V) reage ao ponteiro batendo de
um lado ao outro e chuta o motor: ciclo-limite que não para sozinho. Aconteceu em **4 de ~16**
paradas em 270° desde que o ponteiro ganhou os parafusos (0 de 2 antes, de manhã); com e sem
anti-folga (que nem age em 270°); laço a 5,0 ms normal. **A vigia do robusto.sh NÃO pegou**
(NT pelo Wi-Fi não vê 12 Hz). Candidatos: D menor ou só em movimento perto do alvo; D pela
velocidade do motor (lado sem folga; o encoder do NEO tem atraso); filtro do D maior; e um
detector de oscilação NO ROBÔ (laço de 5 ms, pela quadratura) que corta o motor.
**Feito (simulação; falta deploy/robô):** (1) **proteção contra oscilação** no laço de 5 ms
(`oscGuardStep`, ligada por padrão): conta viradas de sentido > `OscSwingDeg` (1°) da quadratura
(sem ela, do absoluto); com ≥ `OscMaxReversals` (8) em 1 s corta o motor e TRAVA o controle de
posição até desabilitar (`isOscTripped`; `OscReset = 1` libera remotamente, para scripts). A
chegada ruim do 60>75 tem ~4 viradas; o ciclo-limite ~25/s. `MechanismOscGuardSimTest`. O
`robusto.sh` acusa "ABORTADO" se ela disparou. Pode atrapalhar o `MechanismTuner` (RemoteCmd 11),
que testa ganhos altos de propósito. (2) **D parado**: `MechanismConfig.dHoldFactor` / ao vivo
`DParadoFator` (padrão 1 = como antes) multiplica o D com o perfil terminado e |erro| < IZone.
Teste: `tools/ntbridge/teste270.sh [N] [fatores]` (N paradas 90/180→270 por fator, conta os
disparos da proteção; padrão 10 × "1 0.3 0").
**Resultado no robô (`bat_20261112/teste270.log`, log `FRC_20261112_192927`):** DParadoFator 1 →
**oscilou 9 de 10** paradas em 270° (a proteção cortou o motor em todas: ela funciona); **0,3 → 0
de 10; 0 → 0 de 10**. Com 0,3 e 0 o ponteiro termina parado de vez (quadratura sem variação no
último 0,5 s, absoluto 0,05–0,10° pp, erro final ±0,4°). Falta: robusto 2× com 0,3 e 0 para ver se
tirar o D parado piora as chegadas; depois gravar no código (`dHoldFactor`).
Bateria `bat_20261112dp` (2 rodadas, a 5 ms, comparado às rodadas com D cheio de `bat_20261112af`):
passa do alvo méd 1,16/1,10° (D×1) → **1,23/1,26° (×0,3)** → 1,37/1,36° (×0); máx ~4,1° em todos;
erro final 0,27° → **0,19/0,21°** → 0,20/0,17°; chegada 0,33–0,41 s em todos; tremor rápido máx
0,002–0,005° (×0,3) e até 0,034° (×0). **Gravado `dHoldFactor(0.3)` na BANCADA** (usuário aprovou
a sequência). **Vindo do código** (`bat_20261112/dp03codigo.log`, log `FRC_20261112_203437`):
teste270 **0 de 10** oscilações; robusto erro final 0,16/0,17°, tremor rápido 0,002°, mas passa
do alvo méd 2,29/1,32° a 5 ms (r1: 180>0 5,17°, 90>270 4,19°).
**⚠️ Deriva ao longo da tarde (não é o D):** nos movimentos grandes (`tools/ntbridge/log_grandes.py`)
a passada média subiu de 0,6–1,0° (~14:00–14:30, D×1) para 1,1–2,5° (~15:00, ainda D×1) e
1,5–3,4° no fim; 180>0 de 0,0° para 2–5°. Motor a 25 °C. **Bateria DESCARTADA** pelos logs do
Driver Station deste PC (`C:/Users/Public/Documents/FRC/Log Files/DSLogs/*.dslog`, v4, registro de
47 bytes a cada 20 ms, bateria = bytes 2–3 /256; script `tools/ntbridge/dslog_bateria.py`): mediana 12,7 V (13:45) → 12,5 V (17:00), menor
pico 12,1 V; os movimentos pedem ≤ 5 V. Sobra a mecânica (caixa / parafusos novos se acomodando).
De manhã, outra bateria (ou a mesma antes de carregar): 11,8 V às 08:42, 11,4 V às 10:27 e **queda
para 9,0 V às 10:42 e 8,7 V às 11:12** com o robô quase parado — bateria fraca (o usuário diz que
"dura quase nada"; trocou em 2026-11-12). Testar a capacidade (analisador de bateria) antes de usar
em robô que puxa corrente: perto de 6,8 V a roboRIO corta as saídas (brownout).
**Próximo passo (próxima sessão):** conferir a folga (mexer à mão, robô desabilitado, ~5,7° hoje
às 14:30) e os parafusos, e rodar o robusto 2× como referência nova.
**Tensão no log (2026-11-12, no robô; bateria nova: Spark 12,92 V, roboRIO 12,62 V — leem ~0,3 V diferente):** `/Mech5ms/busVolts` (no Spark) e `rioVolts` (roboRIO) a 5 ms,
`getBusVolts`/`getRioVolts`/`isBrownedOut` no Epilogue; `log_rodadas.py` mostra mín./média por
rodada. Decisão: o robô NÃO muda a velocidade pela bateria (imprevisível, como os ganhos); o
perfil é que deve caber na bateria fraca (pedir ≤ ~8–10 V com a compensação de 12 V).
Se voltar a ~1°, fica tudo como está (kG 0,09, D parado 0,3, proteção ligada, anti-folga
desligada).

**⚠️ Quadratura MORTA depois da troca de bateria (2026-11-12, noite).** No robusto
(`bat_20261112/noite_r1/r2`) a quadratura ficou entre −0,04 e −0,09° (vai e volta 1 contagem) com
o ponteiro dando a volta; `isQuadFault = true` e o D voltou ao antigo (absoluto, 0,35°) — o
usuário achou "meio estranho". O absoluto (mesmo Through Bore, mesma alimentação) funciona →
suspeita: **fio de DIO 1 ou DIO 2 solto/mau contato**. Toda a tarde a quadratura acompanhou o
absoluto (< 1° de diferença por log): NÃO explica a deriva da passada. Robusto com ela morta:
passa 1,6/1,1° méd (máx 4,4/3,4°), erro final 0,19/0,29°. Conferir: robô desabilitado, X (solta),
girar à mão e ver `getQuadDeg` mudar junto com `getDeg`.
**Conferido em seguida** (`bat_20261112/quadcheck.csv`, girando à mão ~92°): a quadratura acompanhou
o absoluto com inclinação **0,999** (sentido certo; DIO 1/2 NÃO estão trocados — trocados dariam
−1). Depois de religar, `isQuadFault` = false (ela só se desfaz reiniciando o código). Era mau
contato: se voltar, apertar/trocar o cabo do Through Bore na roboRIO.
**Robusto com tudo em ordem** (bateria nova, robô religado, quadratura ok; `bat_20261112/noite2_r1/r2`,
log `FRC_20261112_223729`): passa méd 1,78/1,33° (máx 4,98/4,11°), movimentos grandes 2,38/1,88°,
erro final 0,20/0,23°, bateria mín 11,8 V no Spark. **A passada maior CONTINUA** → não é bateria,
nem calor, nem algo que se acumula com o uso: mudou entre ~14:30 e ~15:00 e foi piorando aos
poucos (grandes: 0,6–1,0 → 1,1–1,3 → 1,45–1,9 → 1,6–3,4 → 1,9–2,4°). Às ~14:35 o ponteiro foi
girado à mão com o motor em BRAKE (força a caixa 15:1) e o código da anti-folga (desligada) entrou
às ~14:55. A piora gradual aponta para a MECÂNICA afrouxando aos poucos: conferir cada estágio
(3:1, 5:1, caixa de 90°), parafusos de fixação/eixo sextavado, colares, o Through Bore e os
parafusos novos do ponteiro; medir a folga de novo (era 5,6–5,7°).
**Folga medida de novo à noite** (`bat_20261112/folga_noite.csv`): 5,49–5,67° — **igual**. A caixa
não ganhou folga. **Comparando 180>0 a 5 ms** (14:30 bom × 15:00 e noite ruins): antes o D ficava
±0,1 V, a corrente suave e o erro ao setpoint estável (−5 a −6°); depois o **D pula ±0,4–1,2 V a
~10 Hz**, a corrente alterna 0 ↔ 17–22 A e o erro salta (−1 → −6,5 → +0,6°): o ponteiro BATE na
folga DURANTE o movimento e o D (velocidade do ponteiro) alimenta. A quadratura está limpa (0 passos
para trás no cruzeiro, em todos os logs) e o absoluto mostra o mesmo — é físico. Mesma família do
ciclo-limite de 12,5 Hz em 270° (que começou com o ponteiro mais pesado). Por que piorou aos poucos
na tarde: não explicado. **Próximo teste:** kD × 1 / × 0,5 / × 0,25 (com D parado 0,3), 2 rodadas,
avaliando a 5 ms (passada nos grandes, D pp no cruzeiro).
**Resultado (`bat_20261112kd`, log `FRC_20261112_223729`, a 5 ms, 2 rodadas):**
kD 0,0003 → passa méd 1,77/1,51°, grandes 2,74/1,91° (180>0 até 4,3°), chega 0,39/0,36 s, D no
cruzeiro desvio 0,46/0,37 V (pico 1,18 V); **kD 0,00015 → passa méd 1,06/1,17°, grandes
0,86/1,01° (180>0 0,0–0,2°), chega 0,29/0,31 s, erro final 0,20/0,24°, D desvio 0,11/0,12 V**;
kD 0,000075 → 1,18/1,19°, grandes 0,99/1,13°, D 0,06 V. Máximo pior em todos os kD menores
(5,0–5,3° contra 4,3–4,5°) — nos passos de subida perto de 90° (folga). O kD 0,0003 ficou ALTO
para o ponteiro com parafusos: o ponteiro batia na folga no movimento e o D realimentava.
**Gravado kD 0,00015** (usuário aprovou) e **VALIDADO vindo do código** (`bat_20261112/kd15codigo.log`,
log `FRC_20261113_001416`): robusto 2× a 5 ms passa méd 1,23/1,14°, movimentos grandes
**1,07/0,90°** (180>0 0,0°, 90>180 0,0–0,4°), chega 0,30/0,31 s, erro final 0,17/0,20°, bateria
mín 11,8 V; teste270 **0 de 10** oscilações. Continua: 60>75 e 75>90 passam ~4,8–5,0° (folga
perto da vertical, item 4 — a anti-folga com 8° só tirou ~1°).

**Bateria de testes GENÉRICA (2026-11-13) — objetivo do usuário: uma bateria que sirva para
vários subsistemas e descubra os valores de cada um.** Feito (sem robô; análises conferidas com
os logs antigos, números idênticos): `tools/ntbridge/mecanismos/<nome>.conf` descreve o mecanismo
(NOME, TELEM, UNIDADE, CIRCULAR, ALVOS do robusto, MAPA, GRANDE, TOL, PARADA_ALVO/DE, TEMP_MAX);
`MEC=<nome>` antes de qualquer script (padrão `bancada`). Carregadores `mec.sh`/`mec.py`.
`robusto.sh`, `bateria.sh`, `testeparada.sh` (o `teste270.sh` virou atalho dele), `sweep.sh`,
`mapa.sh`, `log_ana.py`, `log_rodadas.py` (agora descobre sozinho os ajustes que mudam entre as
rodadas, passada nos GRANDES pelo tamanho do passo, 3 piores, bateria, `--csv`; o `log_grandes.py`
foi absorvido), `mapa_ana.py`, `compara.py`. Campanhas de bateria em `campanhas/*.sh` (BASE, NOMES,
MUDA; `campanhas/LEIA.md`; histórico em `campanhas/HISTORICO_bancada.md`); a bateria agora confere
TODOS os ajustes antes de cada rodada. `nt.sh setm` aplica vários ajustes numa conexão (antes ~3 s
cada). README de `tools/ntbridge` reescrito. No robô: log a 5 ms em **`/Mech5ms/<NOME>/`** (antes
`/Mech5ms/`; as análises leem os dois), `/Tuning/<NOME>/FastLog`, RemoteCmd/ZeroNow e o
Mechanism2d (`<NOME>/Mecanismo`) pelo nome da config — deploy e rodada no robô feitos em
2026-11-13 (ok).
**Assistente de ajuste `autoajuste.sh` (2026-11-13, validado na bancada — ver abaixo).** Decisão
com o usuário: como as equipes de ponta, o robô MEDE e CALCULA sozinho, mas a DECISÃO é de uma
pessoa (não existe "valor perfeito": há uma faixa larga boa — kG 0,063–0,123 e kD ×0,25–×0,5 deram
quase o mesmo — e a precisão do mecanismo é limitada pela mecânica: folga 5,7°, absoluto 0,35°).
Etapas `ref ff modelo bateria relatorio` (README de tools/ntbridge). Peças: `identifica.py` (FF do
log a 5 ms: kV e kA confiáveis, kG/kS não — se confundem nos movimentos rápidos), `ganhos.py` (LQR
discreto com atraso, critérios LQR_TOL_POS/LQR_TOL_VEL/LQR_UMAX/ATRASO_MS no .conf: bancada 2°,
50°/s, 6 V, 10 ms → kP 0,0060–0,0068 com o kV/kA do log, o código tem 0,006; kD ~0,000016),
`pontua.py` (nota por configuração; empate < 10% → recomenda a mais conservadora e avisa "empate
técnico"; nas baterias de 2026-11-12 escolheu AntiFolga 0, DParado 0,3, kD 0,00015/0,000075 e kG
0,123≈0,09 — o mesmo que decidimos à mão), `autoajuste.py` (estado.json, campanha_auto.sh,
RELATORIO.md). No robô: `MechanismTuner` ganhou o modo **só feedforward** (`/Tuning/<NOME>/
MechTuneSoFF = 1`: mede kS/kG/kV, publica em /MechTune/Resultado/..., não aplica nada;
`MechanismTunerSoFFSimTest`: kG 0,062 contra 0,063 da física). A bateria libera a proteção
(OscReset) depois de uma rodada abortada por ela.
**VALIDADO NA BANCADA (2026-11-13, `tools/ntbridge/aa_20261113/RELATORIO.md`, log
`FRC_20261113_174040`, ~30 min de robô).** Deploy com os nomes por mecanismo ok. ref: passa
1,0/0,8° méd, erro final 0,21/0,25°. **Kit só FF (RemoteCmd 11) rodou pela 1ª vez no robô, sem
problema**: kG 0,059, kS 0,121 (atrito da caixa em movimento), kV 0,0237 V/RPM (baixo: a 10–25°/s
o atrito se mistura). Modelo pelo log: kV 0,0326 V/RPM (código 0,0318), kA 0,0000498 (código
0,00005), R² 0,98 → LQR kP 0,00676, kD 0,0000156. Bateria (5 configs × 2, nota pelo log a 5 ms):
mod14 (kP 0,0095, kI 0,0315, kD 0,0000156) 3,65 — melhor nos grandes (0,61°) e chega em 0,27 s;
atual 3,76 (4 rodadas: inclui as do ref); modeloG (kG 0,059) 3,95; modelo 4,14; mod07 4,14.
Máx 5,2–5,7° em todas (folga perto de 90°). **Recomendação: ficar como está** (empate < 10% →
a mais conservadora) — a mesma resposta achada à mão. Nada mudou no código. Para testar depois,
se quiser: kD baixo (LQR) + kP maior, com `testeparada.sh` em 270°.
Corrigido ao rodar: a conferência da `bateria.sh` chamava `./nt.sh` pelo Python (Windows não
roda bash: agora `bash ./nt.sh`) e o Git Bash convertia `/Tuning/Dial` em `C:/Program
Files/Git/Tuning/Dial` ao passar para o `python.exe` (`MSYS_NO_PATHCONV=1`); o `autoajuste.sh`
agora para se a bateria falhar. O `scp` do log (~200 MB) pelo Wi-Fi leva ~5 min.
**Melhorias depois da validação (2026-11-13, só offline — falta uma rodada no robô):**
(1) **paradas no ponto fraco dentro da bateria** (`PARADAS_POR_RODADA=5` no .conf; campanha pode
mudar com `PARADAS=`): depois de cada rodada, `testeparada.sh N nenhum` com a mesma configuração;
o `log_rodadas.segmentos()` separa esses trechos pelos alvos (só PARADA_ALVO/PARADA_DE) e conta
paradas e disparos (`isOscTripped`, que está no .wpilog); antes, o trecho do teste270 era contado
como rodada do robusto. O `pontua.py` soma `parada`×oscilações/rodada e NUNCA recomenda quem
oscilou. (2) **`volta2`** (etapa nova, padrão): se a melhor for a de fator mais alto (ou mais
baixo) entre modelo×0,7/1/1,4, testa ela de novo + ×`VOLTA2_FATOR` (1,3) e ×1,3² para aquele
lado (subindo: em ordem e `PARA_SE_ABORTAR=1`) e refaz o relatório com todos os logs (guardados
no estado.json). Na bancada daria mod14 → kP 0,0123 e 0,0160 (acima do 0,0105 que oscilou com kD
0,00015; o kD aqui é 10× menor; a proteção corta). (3) **`NOTA_PESOS` no .conf** (passa, grandes,
max, erro, chega, tremor, problema, parada; padrão em `mec.py`). Corrigido: o agrupamento
arredondava em 9 casas decimais (kD 1,56252e-5 virava 1,5625e-5 e não achava o nome da
candidata): agora 6 algarismos significativos. Tempo: ~4,5 min por configuração/rodada (antes
~1,6): assistente completo ~1–1,5 h.
**2ª volta no robô (2026-11-13, log `FRC_20261113_200047`):** mod14 (kP 0,0095, kD 0,0000156)
de novo: passa 1,0°, 0 de 5 paradas em 270°; **kP 0,0123 OSCILOU** logo no início → a bateria
parou (PARA_SE_ABORTAR) sem testar 0,0160. Juntando as voltas: atual 3,76 (melhor), mod14 3,80
(3 rodadas), modeloG 3,95 → **recomendação: ficar com os valores do código**. Corrigido: a
campanha2 contava como "testadas" candidatas só geradas; as rodadas de logs diferentes não se
juntavam (cada log achava "o que variou" só dentro dele: agora `pontua.agrupa` compara a
configuração inteira entre todos os logs). Cache da análise por log em `logs/.cache` (o
relatório passou de ~10 min para < 1 s).
**3ª volta (`volta3`, só offline — falta rodar no robô):** pedido do usuário — o assistente vai
ser usado em vários subsistemas e tem que ACHAR kP, kI e kD, não herdar. O LQR não dá kI (sem
atrito no modelo) nem vê a folga: em volta da escolhida testa kD ×0,5/×2 e kI 0/×2 (se kD 0: o
do LQR e o dobro; se kI 0: kP/`VOLTA3_TI_S` (0,3 s) e o dobro — dá 0,02 na bancada). O relatório
diz "de onde veio cada valor". Na bancada: atual, D05, D2, I0, I2 (~45 min). README ganhou o
roteiro "Usar num subsistema NOVO".
**⚠️ CORREÇÃO (mesmo dia): NÃO foi a mecânica — o LAÇO DE CONTROLE ficou 5× mais lento.** No
log a 5 ms, de t ≈ 4295 s em diante (robô desabilitado e parado, ~1 h de programa ligado) a thread
inteira do robô passou a acordar a cada **25,4 ms** (o callback de 5 ms E o Epilogue de 20 ms;
cada execução continuava rápida, Epilogue ~0,5 ms; CPU 90% livre; memória ok; sem "overrun" em
série). **"Restart Robot Code" resolveu** (4,95 ms) e o robusto voltou a passa 1,2° / erro final
0,14–0,19° (`bat_20261113/restart_r1/r2`). Causa não achada (agendador do WPILib/roboRIO; houve
~1.000 conexões da ponte NT e 30 perdas de comunicação). Feito (deploy): `getLoopPeriodMs` e
`isLoopSlow` (média do período, sempre medida) e aviso no DS se > 8 ms por 2 s; o `robusto.sh`
recusa rodar com o laço lento ("LACO LENTO") e a bateria para; `log_rodadas` guarda `laco_ms` por
rodada e o relatório do assistente deixa de fora (com aviso) as rodadas com laço > 8 ms. **Lição
20: se a passada piorar de repente com os mesmos ganhos, olhe `getLoopPeriodMs` antes da mecânica.**
**SysId com a 15:1 e kit do zero (2026-11-13 19:03, log `FRC_20261113_224249`,
`bat_20261113/sysid_15a1.csv`):** dados para calibrar a `MechanismSim` (J, atrito e folga da caixa
nunca foram recalibrados para a 15:1). Kit só-FF com kG = 0 e VelkV = 0 (como um subsistema novo):
**funcionou** — kS 0,118, kG 0,061, kV 0,0187 V/RPM (o kV das passadas lentas sai baixo: o do log
em velocidade normal é ~0,032). Próximo (na simulação, sem robô): **modo do zero** do assistente —
kit mede kS/kG/kV + etapa nova de acelerações curtas (kA) dentro de ±40°, a malha de velocidade do
kit pelo kV medido, campanha a partir do FF medido + LQR + kI = kP/0,3 s, pular o `ref` sem ganhos;
validar na bancada zerando tudo (resposta conhecida: kG ≈ 0,09, kP 0,006, kI 0,02, kD baixo).
**Simulação calibrada pela 15:1 e MODO DO ZERO (2026-11-13 noite; DEPLOY FEITO no mesmo dia — falta
o teste no robô).** `MechanismSim`: J 0,0056 kg·m² (era 0,000722, ponteiro sem redução), kS 0,114,
kG 0,052, motor livre 12/0,0318×15 ≈ 5660 RPM (era 4858) → `MechanismSimCalibracaoTest` (os
degraus/rampas do SysId na simulação, `build/sim_sysid.csv`): kV 0,03182 (real 0,03177), kS 0,107,
kG 0,053. SEM a folga de 5,6° ainda (a simulação passa menos do alvo que o real: próximo passo).
**kA não é mensurável num mecanismo rápido**: a bancada chega à velocidade final em ~6 ms; com a
derivada suavizada de 5 ms o kA sai 2–4× errado em qualquer teste (na simulação, verdadeiro 0,00003
→ identificado 0,000007 a 0,00014). Regra: `autoajuste.py modelo` usa kA = max(medido, kV × 10 ms)
e avisa abaixo de 50 ms (o LQR muda pouco: kA 0,00001–0,00005 → kP 0,0080–0,0065).
Kit: etapa nova **ACELERA** (4 pulsos de 80% de 3 V dentro da faixa, cortados a 300°/s ou 30% da
faixa, frenagem com a tensão ao contrário) — dá velocidade alta para o kV (só as passadas lentas
dão kV 4× baixo no robô real: 0,008 V/RPM). `identifica.py --aberta` (sem alvo de posição) e
`--csv` (CSV da simulação). **Modo do zero** (`kP` de partida 0): `ref` pulado, o kit começa de
ONDE o mecanismo está, `modelo` identifica pela janela do kit (kS/kG/kV sem atraso, kA com atraso
+ piso), a campanha usa o FF MEDIDO (kG do kit, VelkV e kA do modelo) e kI = kP/`VOLTA3_TI_S`,
candidatas em ordem crescente com `PARA_SE_ABORTAR=1`. `MechanismTunerZeroSimTest`: kit com TUDO
zero → kS 0,117, kG 0,052, kV 0,030; o PC → LQR kP 0,0067, kI 0,022, kD 0,000016, campanha
kP 0,0047/0,0067/0,0094 (as mesmas que funcionaram no robô). **Validar na bancada**: deploy (o kit
mudou), zerar kP/kI/kD/kG/VelkV/kA ao vivo, ponteiro perto de 0°, `autoajuste.sh` completo.
**3ª volta no robô (2026-11-13 ~17:20, log `FRC_20261113_200047`) e O MECANISMO MUDOU:** com os
MESMOS ganhos (atual), a passada foi de 1,0–1,3° (manhã; 16:35 ainda 1,4°) para **3,5–3,8°** (pior
caso 75>90 de 5,0° para 7,1–7,5°), bateria 11,4–11,8 V, motor 25 °C → mecânica (folga/algo
afrouxou entre ~16:40 e 17:20; antes disso o usuário balançou o ponteiro forte contra o motor nos
testes da proteção). **Conferir a folga e os parafusos** antes de testar de novo. Dentro da 3ª
volta (mesma condição): atual 1,00; kD×2 1,13; kD×0,5 1,14; kI×2 1,68 (vibrou parado); **sem kI
2,36** (erro final 0,75°, movimentos sem chegar) → o kI é necessário; os valores do código
continuam os melhores. O relatório antigo misturava as voltas e recomendou o mod07 da manhã:
**corrigido** — cada volta é comparada com a SUA referência (`registra_volta`/`separa_voltas`), a
recomendação só muda se ganhar da referência da volta por > 10%, e "⚠️ O MECANISMO MUDOU" se a
mesma configuração piorar > 1,3× entre voltas (detectou: atual 3,76 → 8,10). Recomendação final:
**atual** (kP 0,006, kI 0,02, kD 0,00015) — igual ao achado à mão, agora com kD e kI testados.
Download de log grande com o robô HABILITADO derrubou a comunicação do DS uma vez (o robô
desabilita sozinho); o usuário preferiu não limitar a banda: baixar com ele desabilitado.
**Proteção contra oscilação refeita (2026-11-13, deploy feito; falta conferir mexendo à mão).**
O usuário mexeu o ponteiro rápido na folga (Teleop, segurando 270°) e ela cortou o motor até
desabilitar — numa partida, um tranco ou o operador mudando de sentido perderia o mecanismo até
o fim. Log a 5 ms (`FRC_20261113_174040`, 5 disparos): mão = viradas a cada **95–150 ms
(3–5 Hz)**; o ciclo-limite de 2026-11-12 = **43–48 ms (~11 Hz)**. Agora
(`MechanismOscDetector`): só conta virada > `OscSwingDeg` (1°) que vem até `OscMaxIntervalMs`
(70 ms) depois da anterior; dispara com `OscMaxReversals` (8) SEGUIDAS. E, como as equipes de
ponta (o mecanismo fica pior, não morto): **nível 1** = desliga o D e continua controlando até
desabilitar; **nível 2** (oscila mesmo sem o D) = motor parado até o operador pedir OUTRO alvo.
`isOscTripped` = disparou (scripts continuam iguais; OscReset zera tudo); `getOscLevel` 0/1/2.
Testes: `MechanismOscDetectorTest` (11 Hz dispara em < 0,5 s; 2–6 Hz ±4° e ±40°, tranco que
amortece e tremor ±0,3° não disparam) e `MechanismOscGuardSimTest` (níveis 1 e 2, liberação por
alvo novo; na simulação, sem folga, só kP 0,5 oscila sem o D). Ressalva: oscilação lenta (< ~7 Hz,
ex.: integral caçando o alvo) não é pega — é menos violenta, e o kI só age perto do alvo.
**Refeita de novo no mesmo dia (deploy feito):** o usuário balançou MUITO rápido
(`FRC_20261113_193445`): viradas a cada **40–75 ms** = igual ao ciclo-limite → nível 1, nível 2
(parou), e o D só voltava ao desabilitar. Pelo sensor não dá para separar (testado: o sinal de
tensão × velocidade do ponteiro também é negativo no ciclo-limite — o ponteiro bate na folga; só a
velocidade do MOTOR separaria, e ela não está no log). Mudou a REAÇÃO: **operador** (padrão) =
desliga o D, que **volta sozinho depois de 2 s sem nenhuma virada**; no 3º disparo com o D
ligado fica sem D até desabilitar; **nunca para o motor**. **Teste automático** (`RemoteCmd ≠ 0`
→ `setOscStrict(true)`, trigger no RobotContainer) = como antes (nível 2 para o motor até outro
alvo; o D não volta sozinho). `MechanismOscGuardSimTest` cobre os dois modos.
**Conferido no robô** (`FRC_20261113_194430`, o usuário balançando a 40–100 ms em 270°): 3
disparos, D a 0,0000 V em cada um, voltou sozinho 2,0 s depois da última virada nos dois
primeiros; motor nunca parou (modo perfil o tempo todo). O 3º deixou sem D até desabilitar →
mudado para **3 disparos em 20 s** (trancos espaçados numa partida não se somam; o ciclo-limite
volta segundos depois do D). Teste na simulação e deploy feitos.

Histórico 2026-11-10 (sem redução, +220 g): kG 0,692 V e 35 A; passava 7–9° do alvo e oscilava
devagar parado; kA 0,000106, kD 0,0001 e kP 0,004 rejeitados (`bat_20261110`).

**Nomes (2026-11-12):** projeto **`Bancada_Testes_Mecanismos`** (o .jar no robô agora tem esse
nome; o antigo `Bancada_PID_FF_vFinal-main.jar` sobra em /home/lvuser até ser apagado). Classes
`Dial*` → `Mechanism*` (`MechanismSubsystem`, `MechanismIO`, `MechanismIOSparkMax`, `MechanismSim`,
`MechanismFastLog`, `MechanismSysId`, `MechanismLimitFinder`); o autoajuste antigo, só para braço
pendurado, virou `ArmAutoTuner`. Telemetria: `Robot/m_robotContainer/mechanism/...` (antes
`.../dial/...` — as gravações antigas em `bat_*` têm o nome antigo, os scripts leem as duas);
log a 5 ms: `/Mech5ms/`. A pasta de ajuste continua **`/Tuning/Dial/...`** (nome da configuração
`BANCADA`; decisão do usuário). Backup de antes da renomeação: não versionado (pasta temporária).
Pasta local renomeada para `C:\CODE\2026\BANCADA\Bancada_Testes_Mecanismos` (2026-11-12;
build e testes ok; não é clone git, levar os arquivos para o GitHub Desktop).

## O que é

Bancada de testes de controle de posição, no estilo de um braço de FRC:

- **NEO + Spark MAX** (CAN ID 2, firmware **26.1.5** — o 27 recusava conversion factors)
  com **redução 15:1** (3:1 × 5:1 MAXPlanetary + caixa de 90° 1:1, desde 2026-11-11;
  `kGearRatio = 15`, folga ~5,5° no ponteiro) movendo um **ponteiro de ~400 g + 220 g** (+ parafusos desde 2026-11-12) que
  gira sobre uma tabela de graus. Até 2026-11-10 o NEO era direto no eixo.
- **Through Bore V1** no eixo, ligado na **roboRIO** (não no Spark):
  - **DIO 0** = sinal absoluto (PWM): dá a **posição** (0,35° de resolução, não perde o zero).
  - **DIO 1 e 2** = quadratura A/B (8192 contagens/volta, 0,044°): dá a **velocidade** do kD.
    `kQuadInverted = true` (conferido à mão).
  - Pelo adaptador REV-11-1881 no Spark NÃO funcionava (contador cravado em 0,750).
- Ângulos: **0° = direita, anti-horário positivo, 270° = pendurado**. Zero gravado em
  `absZeroOffsetDeg = 301.34` (2026-11-12, ponteiro remontado; 270,91 em 2026-11-11; 223,89 antes).
- Repositório: github.com/TUIUTECH9604/**Bancada_PID_FF_vFinal** (público). O nome NÃO foi
  trocado no GitHub (conferido em 2026-11-13; só a pasta e o projeto se chamam
  `Bancada_Testes_Mecanismos`). Se renomear lá, o GitHub redireciona o nome antigo. Commits como
  `TUIUTECH9604 <teamtuiutech@gmail.com>`. **Clone git: `C:\CODE\2026\BANCADA\Bancada_PID_FF_vFinal`**
  (criado em 2026-11-13; esta pasta de trabalho NÃO é clone). Para enviar: copiar os arquivos
  desta pasta para o clone (sem build/, logs/, libs/, out/, bat_*, aa_*, .csv), commit lá e
  `git push` (o usuário autorizou). O clone tem `credential.https://github.com.username =
  TUIUTECH9604` (conta da equipe, login feito em 2026-11-13); sem isso o git usa a conta pessoal
  WeksleyLima, que leva 403. O GitHub Desktop também usa a conta da equipe.

## Arquitetura do controle (`MechanismSubsystem`)

- Malha de posição na **roboRIO a 5 ms** (`addPeriodic` no `Robot`), com
  **`motor.setControlFramePeriodMs(5)`** — sem isso a REVLib só manda a tensão a cada 20 ms e
  o ponteiro oscila (foi a causa do "tremor"; ver Lições).
- Tensão = **feedforward** (kG·cos θ + kV·v + kA·a do perfil) + **P + I + D** feitos à mão.
- **Perfil trapezoidal** sempre (sem perfil, passava 27° do alvo na descida).
- **kI** só perto do alvo (IZone 3°), **congela** dentro da zona morta (0,5°), zerado enquanto o
  perfil anda, com anti-windup.
- **D** = d(alvo)/dt − velocidade da quadratura **por diferença de posição a cada 5 ms**
  (NÃO `Encoder.getRate()`: ele repete a última velocidade com o ponteiro parado → oscilava).
  Se a quadratura discordar >5° do absoluto, é descartada e o D volta ao antigo.
- Velocidade (modo velocidade, pouco usado) roda no próprio Spark, slot 1.
- `MechanismSim`: física da simulação, calibrada pelo pêndulo real **sem redução** (J 0,000722 kg·m²);
  já tem a redução 15:1 e o kG novo, mas J e atrito da caixa NÃO foram recalibrados, e a parte do
  pêndulo solto do `MechanismSubsystemSimTest` só imprime (não reprova). `./gradlew test`, sem robô.
- `ArmAutoTuner` (modo Test ou RemoteCmd 8), `MechanismSysId` (RemoteCmd 9) e `MechanismLimitFinder`
  (RemoteCmd 10 — ver Estado atual antes de usar).
- **Hardware só pela camada `MechanismIO`** (`MechanismIOSparkMax` = Spark MAX + Through Bore na RIO);
  `MechanismFastLog` grava o laço de 5 ms no `.wpilog`.

## Ganhos atuais — redução 15:1 (2026-11-11; kG e kD 2026-11-12, no código)

| kG | kP | kI | IZone | zona morta I | kD | filtro D | kS posição | VelkS | VelkV | kA | perfil |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0,09 V | 0,006 | 0,02 | 3° | 0,5° | 0,00015 (× 0,3 parado) | 5 ms | 0 | 0 | 0,0318 V/RPM do ponteiro | 0,00005 V/(°/s²) | 720°/s, 4000°/s² |

Limite de corrente 20 A. Instável a partir de ~×3,5 desses (kP 0,0105): oscila a ~10 Hz.

## Ganhos validados com o ponteiro LEVE, sem redução (2026-11-09) — histórico

| kG | kP | kI | IZone | zona morta I | kD | filtro D | kS posição | VelkS | VelkV | kA | perfil |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0,362 V | 0,003 | 0,01 | 3° | 0,5° | 0,00015 | 5 ms | 0 | 0 | 0,00247 V/RPM | 0,0000786 V/(°/s²) | 720°/s, 4000°/s² |

Limite de corrente 20 A. Referência (`robusto.sh` e uso com o controle): tremor rápido
0,002–0,005°, parado 0,08–0,17°, assenta em 0,6–0,8 s (máx. ~1,3 s), passa do alvo 0,6–0,8° em
média (máx. ~1,9°), erro final ~0,2–0,3°.

## Lições (não repetir os erros)

1. **20 ms pode não bastar**: malha a 5 ms (`addPeriodic`) **e** tensão enviada a 5 ms
   (`setControlFramePeriodMs(5)`; o padrão da REVLib é 20 ms). Só a malha rápida não adianta.
   Detalhes, sintomas e cuidados (carga no CAN) em `docs/METODO_SUBSISTEMAS.md`, seção 6.1.
2. **Testar com movimentos grandes**: ganhos aprovados só com passos de 15° oscilavam ±10–25°
   depois de movimentos de 90°/180°. Validar com passos pequenos E grandes, com **parada
   automática** (4+ inversões de sentido >0,7° em 1 s).
3. Repetir cada teste (o "grudar e soltar" do atrito varia muito entre rodadas).
4. kS do SysId (0,077 V) não serve como feedforward: na posição faz vibrar; no movimento empurra
   além do alvo. O atrito real em movimento é ~0,015 V.
5. kD alto vibra a ~8 Hz (atraso). kP 0,004+ começa a vibrar.
6. A **corrente lida pelo Spark** dá saltos falsos com pouca tensão (27–34 A com 0,85 V) e marca
   0 A segurando: não usar para detectar travamento.
7. Perfil 1080/8000 passa do alvo (até 65° com 20 A; 18° na descida mesmo com 35 A). 720/4000 é
   o limite bom desta bancada.
8. NetworkTables só manda valores que mudam: ao analisar gravações, reamostrar no tempo.
9. O Mechanism2d tem números a cada **30°** (a versão a cada 5° foi rejeitada).
10. **Mudou o peso → refazer kG antes de tudo** (RemoteCmd 8; `AutoTuneSoKgKs = 1` faz só as
    rampas de kG/kS e não mexe em kP/kD). Com o kG velho o ponteiro cede 8–10°.
11. **NEO direto no eixo é fraco**: ~0,025 N·m/A (20 A ≈ 0,5 N·m). Ponteiro parado num ângulo
    fixo com a tensão subindo e a corrente cravada no limite = falta de força, não travamento.
    Acompanhar `getMotorTempC` (chegou a 46 °C em ~15 min de testes seguidos).
12. **Ajustes ao vivo se perdem quando o robô reinicia** (inclusive quedas): grave no código e
    faça deploy antes de testar; confira o kG antes de cada rodada (uma bateria inteira rodou
    com o kG antigo sem perceber).
13. O `robusto.sh` passava caminhos `/c/...` ao Java e a **parada automática não funcionava** no
    Git Bash (corrigido: `pwd -W`; se a vigia falhar, o teste para). Pelo Wi-Fi cada `nt.sh`
    leva ~3 s: a gravação agora termina no último alvo, não num tempo fixo.
14. **Redução muda quase tudo**: kG ÷ ~15 (+ atrito da caixa), kV × 15. O autoajuste supunha o
    ponteiro "quase parado" (<15 RPM) e ignorava kV·v: com 15:1 isso são ~0,3 V e o kG saía
    0,234 em vez de 0,063 (corrigido: desconta kV·v). Também supunha o ponteiro caindo sozinho a
    270° entre testes (autoajuste e SysId corrigidos para a caixa que segura).
15. **Escalar o PID pela inércia (kA) não funcionou aqui**: a conta dizia ×6,5; ×3,5 já oscilou a
    10 Hz e o melhor foi ×2. Com folga e atraso, subir em degraus (bateria com
    `PARA_SE_ABORTAR=1`). O kA do SysId de novo atrapalhou como feedforward (3ª vez): deixar 0.
17. **Com redução, "pouca tensão" não é "pouca força"**: 15:1 multiplica o torque por 15 e a
    inércia do rotor por 225 — encostar a 15°/s com 0,4 V arrancou um parafuso. A corrente lida
    (0,8 A) não mostrou nada (lição 6). Nunca encostar o motor em batente fraco de propósito.
18. **RemoteCmd 8 (rampas do `ArmAutoTuner`) não serve com a redução + brake**: o atrito parado
    é bem maior que o em movimento; depois de descolar o ponteiro acelera (45–60°/s), passa da
    horizontal e a medida fica ruim (2026-11-12). Medir kG segurando em 0°/180° pelos dois lados
    e decidir por bateria.
19. **Decidir com o log a 5 ms, não só com o `mapa_ana`/`compara.py`**: pelo NT (Wi-Fi) os
    picos curtos somem ao acaso (passos perto de 90°: mapa_ana 0,0–2,2°, log a 5 ms 3,1–3,7° na
    MESMA rodada) e criaram uma "tendência" falsa com o kG (2026-11-12). Use `log_rodadas.py`.
16. Ganhos baixos demais não vencem o atrito do **motor** (0,077 V, igual com ou sem redução):
    kP 0,0003 parava 5,5° antes do alvo (simulação).

## Como trabalhar com o robô (regras combinadas com o usuário)

- **Quem habilita/desabilita é o usuário**, no Driver Station; ele avisa ("habilitei" /
  "desabilitei"). Conferir: `/FMSInfo/FMSControlData` = 32 desabilitado, 33 habilitado
  (bit 0 = habilitado; 40 = E-Stop).
- **Deploy só com o robô desabilitado.** Testes que movem o motor: explicar antes e esperar ele
  habilitar.
- Testes remotos: `/Tuning/Dial/RemoteCmd` (1–11; 10 = busca de batentes, ver Estado atual; 11 = ajuste automático genérico; só age habilitado, zera ao desabilitar).
  Zero remoto: `/Tuning/Dial/ZeroNow = 1` (só age com o motor parado, como o botão Y). Todos os ganhos são ajustáveis ao vivo em
  `/Tuning/Dial/...` (voltam ao padrão do código ao reiniciar).
- Telemetria do Epilogue **sem barra inicial**: `Robot/m_robotContainer/mechanism/getDeg`, etc.
- Ponte NetworkTables e scripts de teste: `tools/ntbridge/` (ver o README de lá; rodar
  `./setup.sh` uma vez). Os principais: `robusto.sh` (o teste que valida ganhos, com parada
  automática), `sweep.sh` (aplica ganhos ao vivo e roda o robusto), `mapa.sh` + `mapa_ana.py`
  (mapa de 0° a 180° em passos de 15°), `bateria.sh` + `compara.py` (várias configurações × 2
  rodadas, com pausa se o motor passar de 55 °C). IP do robô por cabo: **169.254.29.48**; pelo
  **Wi-Fi: 10.96.4.2** (`export NT_SERVER=10.96.4.2`). Python precisa de `numpy`
  (`python -m pip install --user numpy`). Para o `sysid_get.sh` num computador novo, o SSH
  precisa conhecer a chave do roboRIO (`ssh lvuser@IP` uma vez e aceitar).
  E-Stop (`FMSControlData` = 40) só sai reiniciando o roboRIO.
  No AdvantageScope, usar esse IP em Preferences → roboRIO Address.
- Build/deploy: `JAVA_HOME` apontando para o JDK do WPILib 2026
  (`C:/Users/Public/wpilib/2026/jdk` no Windows); `./gradlew build`, `deploy`, `test`,
  `simulateJava`.

## Controle Xbox (porta 0)

A/B: 1º toque 0°, depois ±15° (RB + A/B = ±5°) · D-pad: 90°/0°/180°/270° · analógico direito:
ajuste fino · X: solta o ponteiro (COAST até o próximo movimento; vale desabilitado; sem o X,
parado = BRAKE) · Y segurado 1 s (ponteiro solto com X e pendurado): calibra o zero ·
LB + analógico esquerdo: manual. Vibra ao chegar no alvo.

## Fluxo de análise combinado com o usuário (2026-11-12)

O robô **não muda os próprios ganhos durante o uso** (decisão do usuário, como as equipes de
ponta: previsibilidade). Fluxo: (1) robô grava tudo (pendrive, `/Mech5ms` a 5 ms); (2) baixar os
logs (`logs_get.sh`) e olhar no **AdvantageScope**; (3) Claude analisa (`log_ana.py`,
`compara.py`, baterias) e **sugere** mudanças explicando o porquê; (4) testar a sugestão na
simulação/bancada; (5) só grava no código se passar no `robusto.sh` (2 rodadas) — **uma pessoa
aprova**. Feedforward que aprende sozinho e a versão "só observa" foram discutidos e NÃO
implementados (o usuário não quis por enquanto). AdvantageKit completo (replay) fica para o
robô de competição; a camada `MechanismIO` já está no formato dele.

## Ideias que ficaram para depois

- Perfil que freia mais suave do que acelera (para descidas rápidas a favor da gravidade).
- Atualizar o `ArmAutoTuner` para a lógica atual (hoje ele busca kD = 0,04·kP, sem o teste
  robusto).
- Levar a receita para o braço de competição: encoder absoluto no eixo + quadratura, SysId,
  perfil + feedforward completo, quadro de controle no período da malha, limites de curso por
  software, sem wrap de 360°.
