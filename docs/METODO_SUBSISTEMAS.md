# Como ajustar o controle de qualquer subsistema (com o Claude ajudando)

Este guia é o **método** que usamos na bancada PID/FF da TUIUTECH 9604 (um ponteiro com NEO),
escrito para ser aplicado em **outro mecanismo e outro projeto**: braço, pulso, elevador, torre,
shooter/flywheel, intake com posição, módulo de swerve. Serve para quem é da equipe e para o
Claude Code numa conversa nova.

> **Para o Claude:** se a pessoa estiver em outro projeto ou subsistema, siga este guia na ordem.
> Antes de tudo, entenda o mecanismo (seção 1). Não pule a validação robusta (seção 7). Os
> números desta bancada **não servem** para outro mecanismo; o **processo**, sim. A
> implementação de referência está em `src/main/java/frc/robot/subsystems/MechanismSubsystem.java`.

## Ordem do trabalho

```
1. Entender o mecanismo  →  2. Sensor confiável  →  3. Segurança  →  4. Modelo (feedforward)
→  5. Perfil de movimento  →  6. PID  →  7. Validação robusta  →  8. Registrar
```

Cada etapa depende da anterior. O erro mais comum é começar pelo PID: ele não conserta um
sensor ruim, um atraso escondido nem um feedforward errado. Só mascara, até oscilar.

---

## 1. Entender o mecanismo (perguntas antes de escrever código)

| Pergunta | Por quê |
|---|---|
| Que tipo é? (braço/pulso, elevador, torre, flywheel...) | Define o modelo de feedforward (tabela da seção 4) |
| Motor(es), controlador, **redução**, firmware? | A redução muda tudo: torque, velocidade, resolução |
| Onde está o sensor? No **eixo do mecanismo** ou no motor? | Sensor no mecanismo enxerga folga e flexão; no motor, não |
| Tem **limites físicos** (batentes)? Qual a faixa útil? | Define limites de software e proíbe "dar a volta" |
| Massa e distância do centro de massa (aproximadas) | Dá uma estimativa de kG antes de medir (seção 4) |
| O que é "bom" para este mecanismo? (precisão, tempo) | Define os critérios da validação (seção 7) |

Estimativa de kG de um braço, para conferir a medição depois:
`kG ≈ m·g·r / (torque por volt do motor × redução × nº de motores)`.
Para o NEO, o torque por volt com o motor parado é ≈ 2,6 N·m / 12 V = 0,217 N·m/V.

## 2. Sensor confiável (o controle nunca é melhor que o sensor)

- **Posição:** de preferência um **encoder absoluto no eixo do mecanismo** (não perde o zero e
  enxerga folgas). Confira à mão: girando no sentido positivo, o valor tem que aumentar, e uma
  volta tem que dar 360°.
- **Velocidade:** o derivado de um absoluto com pouca resolução é ruidoso (0,35° a cada 5 ms vira
  degraus de 70°/s). Se possível, use também uma **quadratura** (no Through Bore, os mesmos fios
  A/B: 8192 contagens por volta) só para a velocidade.
- **Teste de sanidade:** gire à mão com o robô desabilitado e grave tudo. Os sensores têm que
  andar juntos, no mesmo sentido e na mesma escala. Na bancada, isso pegou um adaptador que
  "cravava" o valor e uma quadratura que contava ao contrário.

Armadilhas que já pegamos:
- O adaptador **REV-11-1881** não entrega o sinal absoluto ao Spark (com a chave do encoder em
  "A", o PWM vai para o pino de index e zera a contagem o tempo todo).
- `Encoder.getRate()` da WPILib **repete a última velocidade** quando o mecanismo para (ela mede
  o tempo entre pulsos). Calcule a velocidade pela **diferença de posição a cada ciclo**.
- Firmware 27 do Spark MAX recusou conversion factors; voltamos ao 26.1.5.

## 3. Segurança (antes de mover qualquer coisa sozinho)

- **Quem habilita e desabilita é a pessoa**, no Driver Station. O Claude não habilita o robô;
  ele explica o teste e espera a pessoa avisar.
- **Deploy só com o robô desabilitado** (confira `/FMSInfo/FMSControlData`: bit 0 = habilitado).
- Limite de corrente coerente com o mecanismo e o disjuntor. Se o mecanismo travar, quanta
  corrente ele vai puxar?
- **Limites de curso por software** se houver batentes, e **nunca** "caminho mais curto com wrap
  de 360°" num mecanismo com batentes: ele pode decidir dar a volta e bater.
- **Redução multiplica a força**: com 15:1, 0,4 V e 0,8 A lidos já arrancaram um parafuso usado
  como batente (2026-11-11). Nunca encostar o motor de propósito num batente fraco; marque os
  limites à mão. A corrente lida pelo controlador não mostra a força com pouca tensão.
- Testes automáticos com **parada automática** quando o mecanismo começa a oscilar (seção 7).
- Comandos remotos de teste que **só agem com o robô habilitado** e voltam a zero ao desabilitar
  (padrão `RemoteCmd` do `RobotContainer` desta bancada).

## 4. Modelo físico = feedforward (medir, não chutar)

O feedforward faz quase todo o trabalho; o PID só corrige o resto.

| Mecanismo | Feedforward | Classe da WPILib |
|---|---|---|
| Braço / pulso | kS·sinal(v) + **kG·cos θ** + kV·v + kA·a | `ArmFeedforward` |
| Elevador | kS·sinal(v) + **kG** (constante) + kV·v + kA·a | `ElevatorFeedforward` |
| Torre, flywheel, rolo, módulo de swerve | kS·sinal(v) + kV·v + kA·a | `SimpleMotorFeedforward` |

Como medir:
- **kG (braço/elevador):** rampa de tensão bem lenta para os dois lados, com o mecanismo quase
  parado (`ArmAutoTuner` faz isso). Ou SysId.
- **kS, kV, kA:** **SysId** (degraus e rampas em malha aberta). O `MechanismSysId` grava a 200 Hz e
  `tools/ntbridge/sysid_fit.py` ajusta o modelo. Compare o método direto com o "discreto"; na
  bancada, o kA que funcionou ao vivo foi o do método discreto.
- **Confira com a física:** kV de um NEO direto ≈ 12 V / 5676 RPM. O kA dá a inércia
  (J = kA × torque por volt). Na bancada, a inércia bateu com 400 g a ~4 cm.

Armadilhas:
- O **kS medido no SysId não é bom feedforward** de posição: perto do alvo ele troca de sinal e
  faz vibrar; em movimento, empurrava o mecanismo além do alvo (o atrito real em movimento era 5×
  menor). Comece com kS = 0 e só adicione se a validação pedir.
- A **corrente lida pelo Spark** dá saltos falsos com pouca tensão (27–34 A lidos com 0,85 V) e
  marca 0 A segurando o mecanismo. Não use para detectar travamento; use tensão alta com
  velocidade zero.

## 5. Perfil de movimento (sempre)

- **Nunca** mande o setpoint "em degrau" para um mecanismo com inércia: sem perfil, a bancada
  passava 27° do alvo; com perfil, 2°.
- `TrapezoidProfile` (ou MAXMotion no Spark). Alimente o feedforward com a **velocidade e a
  aceleração do perfil**.
- Os limites do perfil precisam caber no que o motor entrega **com o limite de corrente
  configurado**. Acima disso, o controle satura e o mecanismo passa do alvo (na bancada:
  1080°/s com 20 A passou 65°; 720°/s foi o ponto bom).
- Integral **zerada enquanto o perfil anda**; ela só entra quando o perfil chega.

## 6. PID (por último, e com atenção ao atraso)

### 6.1 20 ms pode não ser rápido o bastante: malha e envio da tensão a 5 ms

O `TimedRobot` roda a cada **20 ms**, e a REVLib manda a tensão ao Spark a cada **20 ms por
padrão**. Para mecanismos lentos isso basta. Para um mecanismo **rápido, leve ou com pouco atrito**
(pulso, ponteiro, braço com pouca redução), 20 ms é atraso demais: o kD que seria necessário
para amortecer vira vibração, e o mecanismo fica "mole" ou tremendo.

**Sintomas de que o atraso está limitando:**
- vibração em frequência fixa (na bancada, ~5–9 Hz) que aparece ao subir o kD;
- um kD que funciona em passos pequenos, mas oscila depois de movimentos grandes;
- filtrar mais a derivada **piora** (filtro = mais atraso);
- não se consegue um kP/kD que seja ao mesmo tempo firme e sem tremor.

**O que fazer (são DUAS coisas, e as duas são necessárias):**

1. **Rodar a malha a 5 ms**, num callback separado do loop de 20 ms:
   ```java
   // Robot.java
   addPeriodic(m_robotContainer::fastLoop, 0.005, 0.0025); // 200 Hz, deslocado do loop de 20 ms
   ```
   Os comandos continuam a 20 ms, mas só **definem o alvo**. Quem calcula perfil + FF + PID e
   chama `setVoltage` é o `fastLoop`. Ele deve parar o motor sozinho se o robô for desabilitado
   ou se nenhum comando renovar o pedido há ~100 ms.
2. **Mandar a tensão ao motor no mesmo ritmo:**
   ```java
   motor.setControlFramePeriodMs(5); // padrão da REVLib: 20 ms
   ```
   **Sem isso, a malha de 5 ms é desperdiçada:** ela calcula, mas o Spark só recebe a cada 20 ms.
   Foi exatamente o que aconteceu na bancada. Fizemos o loop de 5 ms, ele "quase não ajudou", e
   só dias depois descobrimos que a tensão continuava saindo a 20 ms. Depois da correção, a
   mesma configuração que oscilava ±10° ficou estável, com tremor de 0,005°.

**Como diagnosticar:** grave a tensão aplicada (`getAppliedOutput() × getBusVoltage()`, ou o
que o código manda) e o tempo real entre os ciclos (`getLoopDtMs` na bancada). Se a tensão
"escada" a cada 20 ms enquanto a malha roda a 5 ms, o envio é o gargalo.

**Cuidados:**
- **Barramento CAN:** cada motor a 5 ms gera 4× mais mensagens. Para 1 ou 2 mecanismos críticos
  tudo bem; não faça isso em todos os motores do robô. Acompanhe a utilização do CAN no Driver
  Station.
- **CPU da roboRIO:** a malha de 5 ms tem que ser leve (sem alocar objetos, sem logs pesados).
  Confira se não aparece "loop overrun".
- **Alternativa melhor quando o sensor está ligado no Spark:** malha **dentro do Spark**
  (posição/MAXMotion a ~1 kHz, com feedforward pelo `arbFF` ou o `FeedForwardConfig`). Aí não
  há atraso de CAN no controle. Na bancada não deu porque o encoder está na roboRIO.
- Teste sempre com a validação robusta (seção 7): o atraso só se revela nos movimentos grandes.

### 6.2 Os ganhos

- **kP:** suba até começar a passar do alvo ou vibrar; volte um pouco.
- **kD:** é o amortecimento, mas com atraso ele vira vibração (~8 Hz na bancada). Precisa de uma
  velocidade limpa (seção 2) e de pouco filtro: filtrar mais **aumentou** a vibração.
- **kI:** só perto do alvo (IZone), com **zona morta** (congela a menos de ~0,5°) e anti-windup.
  Sem isso, a integral e o atrito "caçam" o alvo.
- Ganhos ajustáveis ao vivo (`TunableNumber` em `/Tuning/...`) para testar sem deploy.

## 7. Validação robusta (o passo que mais enganou a gente)

Um teste fácil aprova ganhos que oscilam no uso real. Na bancada, ganhos aprovados com passos de
15° oscilaram ±10–25° depois de um movimento de 180° com o controle.

- O teste precisa ter **passos pequenos E movimentos grandes e rápidos**, como no uso real, em
  **várias posições** (para um braço, a gravidade muda com o ângulo).
- **Parada automática**: 4+ inversões de sentido (>0,7°) em 1 s, duas vezes seguidas = oscilação
  → desliga o comando na hora (`tools/ntbridge/osc.py` e `robusto.sh`).
- **Repita** cada configuração (2 rodadas no mínimo): o atrito faz os números variarem muito.
- Métricas (ver `mapa_ana.py`): **tremor rápido**, **quanto mexe parado**, **tempo para
  assentar**, **overshoot** (medido pelo lado de onde o mecanismo chegou) e **erro final**.
- No fim, **a pessoa testa com o controle** enquanto o Claude grava e mede. Se ela vir algo
  estranho, acredite: os números podem estar medindo a coisa errada.
- Mude **uma coisa por vez** e compare com a referência.

## 8. Registrar

- Commit explicando **o porquê e os números medidos** (antes/depois), não só "ajustei ganhos".
- Atualize o `CLAUDE.md` do projeto: estado atual, ganhos, lições novas.
- Guarde os testes que validaram (scripts no repositório, não numa pasta temporária).

---

## Como trabalhar com o Claude ao vivo no seu projeto

1. **Leve as ferramentas:** copie `tools/ntbridge/` para o seu projeto e rode `./setup.sh` uma vez
   (precisa do WPILib 2026 e de Python 3 com numpy). A ponte (`nt.sh get/set/rec/list/enabled`)
   funciona com **qualquer** código de robô que publique no NetworkTables.
2. **IP do robô:** `export NT_SERVER=...`. Por cabo direto costuma ser `169.254.x.x`; pelo rádio,
   `10.TE.AM.2`. O mesmo vale para o AdvantageScope (Preferences → roboRIO Address).
3. **Publique a telemetria** que o Claude precisa analisar (Epilogue `@Logged` ou SmartDashboard):
   posição, velocidade, alvo, alvo do perfil, tensão aplicada, parcelas FF/P/I/D, corrente,
   modo. Os tópicos do Epilogue ficam **sem barra inicial** (`Robot/...`).
4. **Crie um comando remoto de teste** (como `RemoteCmd`) e um alvo ajustável (como
   `TestSetpointDeg`), para o Claude rodar testes sem controle.
5. Os scripts `robusto.sh`, `mapa.sh` e `mapa_ana.py` usam os tópicos desta bancada
   (`Robot/m_robotContainer/mechanism/...`). No seu projeto, troque esses nomes.
6. Peça ao Claude: *"leia `docs/METODO_SUBSISTEMAS.md` e me ajude a ajustar o [mecanismo]"*. Ele
   vai fazer as perguntas da seção 1 antes de mexer em ganhos.

## Usar o código desta bancada em outro mecanismo

O código foi separado para servir de padrão de teste (2026-11-11):

1. **Configuração** (`subsystems/MechanismConfig.java`): tudo o que muda de um mecanismo para
   outro — nome (pasta `/Tuning/{nome}/`), redução, modelo de gravidade (`NONE` giratório na
   horizontal, `COSINE` braço, `CONSTANT` carga fixa), zero do sensor, corrente, limites de curso e
   os ganhos/perfil padrão. Comece copiando: `MechanismConfig.BANCADA.toBuilder("Braco")...build()`.
   Ganhos iniciais conservadores (kP baixo, perfil lento) e **sem** kA até medir.
2. **Hardware** (`subsystems/MechanismIO.java`): implemente a interface para o motor e o sensor do
   mecanismo (o `MechanismIOSparkMax` é o exemplo: Spark MAX + Through Bore na roboRIO). Kraken/TalonFX
   ou encoder ligado no controlador = outra implementação; o controle não muda.
3. `new MechanismSubsystem(config, io)` — o resto (perfil, FF + PID a 5 ms, limites, testes remotos,
   autoajuste, SysId, gravação a 5 ms) é o mesmo.
4. Simulação primeiro: `MechanismConfigSimTest` mostra um mecanismo diferente (torre sem
   gravidade) rodando no mesmo código. Escreva um teste assim antes de ligar o mecanismo real.

5. **Ajuste automático genérico** (`commands/MechanismTuner.java`, RemoteCmd 11): confere o
   sentido, mede kS/kG/kV andando devagar numa faixa de ±40° (dentro dos limites, sem procurar
   batente), sobe os ganhos em degraus com parada por oscilação e escolhe com margem de 1,5×.
   Resultado em `/MechTune/Relatorio`. Depois, **valide com o `robusto.sh`** (2 rodadas) antes de
   gravar na configuração; perfil e kA continuam na `bateria.sh`.

Ainda não suportado: mecanismos **lineares** (elevador em metros). O `ArmAutoTuner` antigo
(RemoteCmd 8) supõe braço com gravidade cos θ pendurado em 270°; prefira o `MechanismTuner`.

## Referência nesta bancada

| Assunto | Onde ver |
|---|---|
| Malha de 5 ms, FF + PID à mão, integral com zona morta, quadratura no D | `MechanismSubsystem.java` |
| Autoajuste de kG/kS e busca de kP/kD | `commands/ArmAutoTuner.java` |
| SysId em malha aberta gravando a 200 Hz | `commands/MechanismSysId.java` + `tools/ntbridge/sysid_fit.py` |
| Simulação calibrada pela física medida + teste automático | `MechanismSim.java` + `src/test/.../MechanismSubsystemSimTest.java` |
| Teste robusto com parada automática | `tools/ntbridge/robusto.sh`, `osc.py`, `mapa_ana.py` |
| Várias configurações × 2 rodadas, com resumo | `tools/ntbridge/bateria.sh` + `compara.py` |
| Configuração do mecanismo e camada de hardware | `MechanismConfig.java`, `MechanismIO.java`, `MechanismIOSparkMax.java` |
| Gravação a 5 ms no robô e análise | `MechanismFastLog.java` + `tools/ntbridge/logs_get.sh`, `log_ana.py`, `wpilog.py` |
| Ajuste automático genérico (sentido, FF, ganhos em degraus) | `commands/MechanismTuner.java` + testes `MechanismTuner*SimTest` |
| Limites de curso (por dentro da faixa) | `MechanismSubsystem` (`setLimits`), teste `MechanismLimitFinderSimTest` |
| Histórico do porquê de cada decisão | `git log` (mensagens com os números) e `CLAUDE.md` |
