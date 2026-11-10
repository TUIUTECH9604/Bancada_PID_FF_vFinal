# Bancada PID/FF — equipe TUIUTECH 9604

Contexto para continuar o trabalho em qualquer computador (o Claude Code lê este arquivo
automaticamente). Conversa em **português**. Última atualização: 2026-11-09.

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
6. Se o `git push` pela linha de comando falhar com 403, é a conta do git do computador sem
   permissão no repositório da equipe: peça para o usuário enviar pelo GitHub Desktop.

## Estado atual (2026-11-09)

Bancada **pronta e validada**: o tremor visível foi resolvido (causa: quadro de controle do
Spark a 20 ms). Ganhos na tabela abaixo, gravados no código e no robô. Nada pendente no git.
Próximos passos possíveis estão em "Ideias que ficaram para depois".

## O que é

Bancada de testes de controle de posição, no estilo de um braço de FRC:

- **NEO + Spark MAX** (CAN ID 2, firmware **26.1.5** — o 27 recusava conversion factors)
  direto no eixo de um **ponteiro de ~400 g** que gira sobre uma tabela de graus.
- **Through Bore V1** no eixo, ligado na **roboRIO** (não no Spark):
  - **DIO 0** = sinal absoluto (PWM): dá a **posição** (0,35° de resolução, não perde o zero).
  - **DIO 1 e 2** = quadratura A/B (8192 contagens/volta, 0,044°): dá a **velocidade** do kD.
    `kQuadInverted = true` (conferido à mão).
  - Pelo adaptador REV-11-1881 no Spark NÃO funcionava (contador cravado em 0,750).
- Ângulos: **0° = direita, anti-horário positivo, 270° = pendurado**. Zero gravado em
  `kAbsZeroOffsetDeg = 223.89`.
- Repositório: github.com/TUIUTECH9604/Bancada_PID_FF_vFinal (público). Commits como
  `TUIUTECH9604 <teamtuiutech@gmail.com>`; o push é feito pelo **GitHub Desktop**.

## Arquitetura do controle (`DialSubsystem`)

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
- `DialSim`: física da simulação, calibrada pelo pêndulo real (J 0,000722 kg·m²; viradas batem
  a 1–2°). `DialSubsystemSimTest` roda com `./gradlew test`, sem robô.
- `DialAutoTuner` (modo Test ou RemoteCmd 8) e `DialSysId` (RemoteCmd 9).

## Ganhos atuais (validados no robô em 2026-11-09)

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

## Como trabalhar com o robô (regras combinadas com o usuário)

- **Quem habilita/desabilita é o usuário**, no Driver Station; ele avisa ("habilitei" /
  "desabilitei"). Conferir: `/FMSInfo/FMSControlData` = 32 desabilitado, 33 habilitado
  (bit 0 = habilitado; 40 = E-Stop).
- **Deploy só com o robô desabilitado.** Testes que movem o motor: explicar antes e esperar ele
  habilitar.
- Testes remotos: `/Tuning/Dial/RemoteCmd` (1–9; só age habilitado, zera ao desabilitar).
  Zero remoto: `/Tuning/Dial/ZeroNow = 1`. Todos os ganhos são ajustáveis ao vivo em
  `/Tuning/Dial/...` (voltam ao padrão do código ao reiniciar).
- Telemetria do Epilogue **sem barra inicial**: `Robot/m_robotContainer/dial/getDeg`, etc.
- Ponte NetworkTables e scripts de teste: `tools/ntbridge/` (ver o README de lá; rodar
  `./setup.sh` uma vez). Os principais: `robusto.sh` (o teste que valida ganhos, com parada
  automática), `sweep.sh` (aplica ganhos ao vivo e roda o robusto), `mapa.sh` + `mapa_ana.py`
  (mapa de 0° a 180° em passos de 15°). IP do robô por cabo: **169.254.29.48** (`NT_SERVER` para mudar).
  No AdvantageScope, usar esse IP em Preferences → roboRIO Address.
- Build/deploy: `JAVA_HOME` apontando para o JDK do WPILib 2026
  (`C:/Users/Public/wpilib/2026/jdk` no Windows); `./gradlew build`, `deploy`, `test`,
  `simulateJava`.

## Controle Xbox (porta 0)

A/B: 1º toque 0°, depois ±15° (RB + A/B = ±5°) · D-pad: 90°/0°/180°/270° · analógico direito:
ajuste fino · X: solta o ponteiro · Y segurado 1 s (ponteiro solto e pendurado): calibra o zero ·
LB + analógico esquerdo: manual. Vibra ao chegar no alvo.

## Ideias que ficaram para depois

- Perfil que freia mais suave do que acelera (para descidas rápidas a favor da gravidade).
- Atualizar o `DialAutoTuner` para a lógica atual (hoje ele busca kD = 0,04·kP, sem o teste
  robusto).
- Levar a receita para o braço de competição: encoder absoluto no eixo + quadratura, SysId,
  perfil + feedforward completo, quadro de controle no período da malha, limites de curso por
  software, sem wrap de 360°.
