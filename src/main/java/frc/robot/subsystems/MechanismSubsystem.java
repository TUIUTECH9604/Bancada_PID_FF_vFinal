package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.function.DoubleSupplier;

/**
 * Bancada de testes: NEO + Spark MAX girando o ponteiro sobre a tabela de graus, com um
 * Through Bore (V1) no eixo, ligado na roboRIO (DIO 0, sinal ABSOLUTO por PWM: fio branco).
 *
 * <p>Ângulos: 0° = ponteiro apontando para a DIREITA (horizontal), positivo = sentido
 * ANTI-HORÁRIO, vendo a placa de frente. Com a placa na vertical, esse ângulo é exatamente o θ do
 * feedforward de gravidade:
 *   tensão = kS·sinal(erro) + kG·cos(θ) + PID
 *
 * <p>Sensores:
 *   - Through Bore (absoluto, 0–360°, ~0,35° de resolução) na roboRIO: realimentação do PID de
 *     POSIÇÃO, que roda na roboRIO a 200 Hz (callback de 5 ms; o Spark não enxerga sensores
 *     ligados na RIO). Dial/FastLoop = 0 volta para 20 ms.
 *     Não perde o zero ao desligar.
 *   - Encoder interno do NEO: realimentação do PID de VELOCIDADE, que roda no próprio Spark.
 *
 * <p>Ganhos de posição continuam em "fração da bateria (12 V) por grau", como antes.
 * kD é por SEGUNDO. P, I e D são calculados à mão em {@link #driveWithPid} (não no
 * PIDController da WPILib).
 *
 * <p>Calibração do zero (uma vez, ou quando mexer no encoder): ponteiro pendurado e parado →
 * botão Y (ou /Tuning/Dial/ZeroNow = 1). O código calcula o offset para o ângulo atual virar
 * ZeroRefDeg (270°) e imprime no Riolog o valor para copiar em MechanismConfig (absZeroOffsetDeg).
 *
 * <p>Modo PERFIL: tensão = kG·cos(θ) + VelkS·sinal(v) + VelkV·v + PID   (v = velocidade do perfil)
 *
 * <p>Modo VELOCIDADE (slot 1 do Spark): tensão = kS·sinal(alvo) + kV·alvo + PID.
 * A gravidade NÃO é compensada de propósito.
 */
@Logged
public class MechanismSubsystem extends SubsystemBase {

  // ===================== Mecanismo (CONFIRA) =====================
  // Fiação, CAN ID e inversões do motor/quadratura ficam no MechanismIOSparkMax (camada de hardware).
  // A quadratura (0,044°) é usada só para a VELOCIDADE (a derivada do PID); a posição vem do
  // absoluto, que não perde o zero.





  private static final double kNominalVolts = 12.0;

  /** Período do callback rápido de controle (registrado no Robot com addPeriodic). */
  public static final double kFastPeriodSec = 0.005;
  private static final int kFastPeriodMs = 5;
  /** Se nenhum comando renovar o pedido nesse tempo, o callback rápido para o motor. */
  private static final double kRequestTimeoutSec = 0.1;


  // ===================== Ganhos e testes (editar no Elastic: /Tuning/Dial/...) =====================
  @NotLogged
  // 15:1 (2026-11-11): 2× os do ponteiro sem redução, escolhidos por bateria (robusto.sh, 2 rodadas
  // cada): ×1 tremia parado 0,8° em média (11 de 19 pontos), ×2 tremeu 0,1° (0–1 de 19), assenta
  // em 0,58 s, passa 0,25° em média; ×3,5 oscilou a ~10 Hz (limite). Abaixo de ×1 não vence o
  // atrito do motor (0,077 V).
  private final TunableNumber kP;
  @NotLogged
  private final TunableNumber kI;
  @NotLogged
  private final TunableNumber kD;
  @NotLogged
  private final TunableNumber iZone;
  /**
   * Zona morta da integral: com |erro| menor que isto, a integral CONGELA (não cresce nem zera).
   * Sem ela, a integral e o atrito estático fazem o ponteiro "caçar" o alvo (±1–1,5° medido).
   * 0 = desliga (integral comum).
   */
  @NotLogged
  private final TunableNumber iDeadband;
  /**
   * 1 = malha de posição no callback rápido (kFastPeriodSec, 5 ms). 0 = no ciclo normal de 20 ms,
   * como antes. Dá para trocar ao vivo e comparar.
   */
  @NotLogged
  private final TunableNumber fastLoop;
  /** Filtro passa-baixa da derivada (constante de tempo, s). 0 = sem filtro. */
  @NotLogged
  private final TunableNumber dFilterSec;
  /**
   * De onde vem a velocidade da derivada: 1 = quadratura do Through Bore (fina), 0 = diferença
   * de leituras do absoluto (antigo). Dá para trocar ao vivo e comparar.
   */
  @NotLogged
  private final TunableNumber useQuadForD;
  @NotLogged
  // kS medido = 0,091 V, mas no modo posição ele troca de sinal a cada cruzamento do alvo e faz
  // o ponteiro tremer (~8 Hz). O kI (só dentro do IZone) vence o atrito parado sem isso.
  private final TunableNumber kS;
  @NotLogged
  // 15:1 (2026-11-11): 0,063 V, medido nas rampas do autoajuste com o termo de velocidade
  // (V = kS·sinal(v) + kG·cos θ + kV·v; kS 0,099 V, erro 0,021 V). Histórico: 0,692 V com +220 g
  // sem redução; 0,362 V sem o peso.
  private final TunableNumber kG;
  @NotLogged
  private final TunableNumber outMax;

  @NotLogged
  private final TunableNumber testSetpointDeg;
  @NotLogged
  private final TunableNumber stepADeg;
  @NotLogged
  private final TunableNumber stepBDeg;
  @NotLogged
  private final TunableNumber stepCDeg;
  @NotLogged
  private final TunableNumber stepDDeg;
  @NotLogged
  private final TunableNumber stepEDeg;
  @NotLogged
  private final TunableNumber stepPeriodSec;
  @NotLogged
  private final TunableNumber openLoopVolts;

  // ---- Velocidade (/Tuning/Dial/Vel...) — RPM do ponteiro ----
  @NotLogged
  private final TunableNumber velKP;
  @NotLogged
  // 0: o kS do SysId (0,077 V) empurrava o ponteiro na frenagem e ele passava ~4° do alvo; o
  // atrito real em movimento é bem menor (pêndulo: ~0,015 V)
  private final TunableNumber velKS;
  @NotLogged
  // V por RPM do PONTEIRO, 15:1: 0,0318 medido nas rampas (= 0,00212 V/RPM no motor, perto do
  // SysId do NEO sem redução, 0,00223). Conferir com o SysId.
  private final TunableNumber velKV;
  /**
   * kA: volts por grau/s² de aceleração (SysId da bancada, 2026-11-07). Usado no FF do perfil:
   * V = kS·sinal(v) + kG·cos θ + kV·v + kA·a. 0 = desliga (compara com e sem).
   */
  @NotLogged
  // 15:1: 0,00005 (bateria bat_20261111e, perfil 720/4000): passa 0,65° méd do alvo contra 1,45°
  // com kA 0. 0,0002 passava ainda menos (0,45° / máx. 0,9°) mas assentava mais devagar. O kA do
  // SysId (0,000512) piorou tudo (passou até 8° e errou 1,3°).
  private final TunableNumber kA;
  @NotLogged
  private final TunableNumber velOutMax;
  @NotLogged
  private final TunableNumber velTargetRpm;
  @NotLogged
  private final TunableNumber velStepARpm;
  @NotLogged
  private final TunableNumber velStepBRpm;

  // ---- Perfil de movimento (/Tuning/Dial/Prof...) ----
  @NotLogged
  // 15:1: 720°/s, 4000°/s² com kA 0,00005 (bat_20261111d/e): 90° em ~0,78 s e 180° em ~0,90 s
  // contando o assentar, ~20–25% mais rápido que 360/2000. Acima de 720°/s o kV sozinho passa de
  // cfg.maxFFVolts (4 V); 720/6000 passava 2° méd do alvo.
  private final TunableNumber profMaxVelDegps;
  @NotLogged
  private final TunableNumber profMaxAccelDegps2;
  // Anti-folga (ver MechanismConfig.antiBacklashOn): /Tuning/Dial/AntiFolga, AntiFolgaDeg,
  // AntiFolgaFatorMax
  // Proteção contra oscilação (/Tuning/Dial/OscGuard, OscSwingDeg, OscMaxReversals,
  // OscMaxIntervalMs) e D parado (/Tuning/Dial/DParadoFator) — ver oscGuardStep() e driveWithPid()
  @NotLogged
  private final TunableNumber oscGuard;
  @NotLogged
  private final TunableNumber oscSwingDeg;
  @NotLogged
  private final TunableNumber oscMaxReversals;
  @NotLogged
  private final TunableNumber oscMaxIntervalMs;
  @NotLogged
  private final TunableNumber dParadoFator;
  /** 1 = libera a proteção contra oscilação já disparada (para scripts de teste); volta a 0. */
  @NotLogged
  private final TunableNumber oscReset;
  @NotLogged
  private final TunableNumber antiFolga;
  @NotLogged
  private final TunableNumber antiFolgaDeg;
  @NotLogged
  private final TunableNumber antiFolgaFatorMax;

  // ---- Limites de curso (batentes físicos) — /Tuning/Dial/Limit... ----
  // Achados pelo MechanismLimitFinder (RemoteCmd 10). LimitLowDeg = batente alcançado girando no sentido
  // NEGATIVO (horário); LimitHighDeg = no POSITIVO (anti-horário). A faixa permitida vai de Low a
  // High no sentido anti-horário, com LimitMarginDeg de folga antes de cada batente.
  @NotLogged
  private final TunableNumber limitsOn;
  @NotLogged
  private final TunableNumber limitLowDeg;
  @NotLogged
  private final TunableNumber limitHighDeg;
  @NotLogged
  private final TunableNumber limitMarginDeg;

  // ---- Encoder absoluto ----
  /** Ângulo que o botão Y grava. 270° = ponteiro pendurado para baixo, parado. */
  @NotLogged
  private final TunableNumber zeroRefDeg;
  /** Zero do Through Bore (graus). Começa com cfg.absZeroOffsetDeg; o botão Y atualiza. */
  @NotLogged
  private final TunableNumber absZeroOffsetDeg;

  // ===================== Estado =====================
  private enum Mode { STOPPED, OPEN_LOOP, CLOSED_LOOP, PROFILED, VELOCITY, SYSID }

  @NotLogged
  private Mode mode = Mode.STOPPED;
  /** Setpoint de posição. Pode sair de 0–360 no modo perfil (o perfil é contínuo). */
  @NotLogged
  private double targetDeg = 0.0;
  @NotLogged
  private double targetRpm = 0.0;
  /** Destino FINAL do perfil (o que foi pedido), contínuo. */
  @NotLogged
  private double goalDeg = 0.0;
  /** Anti-folga: pedido (0–360°) para o qual a aproximação já foi decidida (NaN = nenhum). */
  @NotLogged
  private double approachForDeg = Double.NaN;
  /** Anti-folga: ponto intermediário = destino + este desvio, enquanto approachPending. */
  @NotLogged
  private double approachOffsetDeg = 0.0;
  @NotLogged
  private boolean approachPending = false;
  /** Proteção contra oscilação: disparou alguma vez desde que habilitou (aviso; ver oscLevel). */
  @NotLogged
  private boolean oscTripped = false;
  /**
   * 0 = normal; 1 = D desligado (oscilou uma vez; o mecanismo continua funcionando até
   * desabilitar); 2 = oscilou de novo sem o D: motor parado até um comando para OUTRO alvo.
   */
  @NotLogged
  private int oscLevel = 0;
  /** Alvo (0–360°) que estava pedido quando o nível 2 parou o motor. */
  @NotLogged
  private double oscStopGoalDeg = Double.NaN;
  /**
   * true = teste automático rodando (RemoteCmd ≠ 0: bateria, kit...): a proteção pode chegar ao
   * nível 2 (parar o motor). false = operador no controle: nunca para o motor e o D volta sozinho.
   */
  @NotLogged
  private boolean oscStrict = false;
  /**
   * Quando disparou com o D ligado (só os últimos kOscStrikeWindowSec contam: kOscMaxStrikes
   * nessa janela = ciclo-limite de verdade, o D não volta mais; trancos espaçados não se somam).
   */
  @NotLogged
  private final java.util.ArrayDeque<Double> oscStrikeTimes = new java.util.ArrayDeque<>();
  @NotLogged
  private double oscTripTime = Double.NEGATIVE_INFINITY;
  /** Sem virada nenhuma por este tempo (s), o D volta (só com o operador; até 3 disparos). */
  private static final double kOscRestoreSec = 2.0;
  private static final int kOscMaxStrikes = 3;
  private static final double kOscStrikeWindowSec = 20.0;
  /** Ciclo-limite confirmado (kOscMaxStrikes na janela): o D não volta até desabilitar. */
  @NotLogged
  private boolean oscDNeverBack = false;
  @NotLogged
  private double oscLastDeg = Double.NaN;
  @NotLogged
  private double oscContDeg = 0.0;
  @NotLogged
  private final MechanismOscDetector oscDetector = new MechanismOscDetector();
  @NotLogged
  private TrapezoidProfile.State profState = new TrapezoidProfile.State();
  /** Aceleração pedida pelo perfil no passo atual (graus/s²). */
  @NotLogged
  private double profAccel = 0.0;
  @NotLogged
  private double commandedVolts = 0.0;
  @NotLogged
  private double ffVolts = 0.0;
  @NotLogged
  private double pidVolts = 0.0;
  /** Integral do erro (graus·s), feita à mão para ter zona morta. */
  @NotLogged
  private double errIntegral = 0.0;
  @NotLogged
  private double integralVolts = 0.0;
  /** Partes P e D da tensão do PID (antes do limite OutMax), para a gravação a 5 ms. */
  @NotLogged
  private double pTermVolts = 0.0;
  @NotLogged
  private double dTermVolts = 0.0;
  /** Derivada do erro (graus/s), filtrada. */
  @NotLogged
  private double errDeriv = 0.0;
  @NotLogged
  private double prevError = 0.0;
  @NotLogged
  private boolean firstStep = true;
  /** Destino pedido pelo comando no modo perfil (0–360°). */
  @NotLogged
  private double goalRequestDeg = 0.0;
  /** Último instante em que um comando renovou o pedido de posição. */
  @NotLogged
  private double requestTime = -1.0;
  @NotLogged
  private double lastFastTime = -1.0;
  @NotLogged
  private double loopDtMs = 0.0;
  /** Período do callback rápido (média móvel, sempre medido, mesmo parado/desabilitado). */
  @NotLogged
  private double loopPeriodMs = kFastPeriodSec * 1000.0;
  @NotLogged
  private double loopSlowSince = Double.NaN;
  @NotLogged
  private double loopWarnTime = Double.NEGATIVE_INFINITY;
  /** Laço "lento" acima disto (ms) por kLoopSlowSec: o controle piora muito (lição 1). */
  private static final double kLoopSlowMs = 8.0;
  private static final double kLoopSlowSec = 2.0;
  /** O Spark está em BRAKE? A simulação usa para saber se o motor freia. Começa em COAST (IO). */
  @NotLogged
  private boolean brakeMode = false;
  /** Solto pelo botão X / release(): COAST até o próximo movimento (se a config pede BRAKE). */
  @NotLogged
  private boolean released = false;
  /** Sessão de SysId aberta: BRAKE entre os testes. */
  @NotLogged
  private boolean sysIdSession = false;

  // ---- SysId: tensão em malha aberta + gravação a 200 Hz (no callback rápido) ----
  private static final int kSysIdMaxSamples = 12000; // 60 s a 5 ms
  @NotLogged
  private final double[][] sysIdBuf = new double[kSysIdMaxSamples][6];
  @NotLogged
  private int sysIdN = 0;
  @NotLogged
  private double sysIdVolts = 0.0;
  @NotLogged
  private int sysIdTest = 0;
  @NotLogged
  private double sysIdLastWrapped = Double.NaN;
  @NotLogged
  private double sysIdCont = 0.0;
  @NotLogged
  private double stepStartTime = 0.0;

  // Ângulo contínuo (sem salto 360→0) e velocidade, derivados do Through Bore em periodic()
  @NotLogged
  private boolean contInit = false;
  @NotLogged
  private double lastWrappedDeg = 0.0;
  @NotLogged
  private double contDeg = 0.0;
  @NotLogged
  private double lastTime = 0.0;
  @NotLogged
  private double velRpm = 0.0;
  @NotLogged
  private final LinearFilter velFilter = LinearFilter.movingAverage(5);
  @NotLogged
  private boolean warnedDisconnected = false;

  // ===================== Hardware (só pela camada IO) =====================
  /** O que muda de um mecanismo para outro (redução, gravidade, limites, ganhos padrão). */
  @NotLogged
  private final MechanismConfig cfg;
  @NotLogged
  private final MechanismIO io;
  /** Sensores lidos uma vez no início de cada ciclo de 5 ms (fastLoop). */
  @NotLogged
  private final MechanismIO.Inputs in = new MechanismIO.Inputs();
  /** Gravação a 5 ms no .wpilog (null = desligada; o Robot liga). */
  @NotLogged
  private MechanismFastLog fastLog = null;
  /** Diferença (graus) entre quadratura e absoluto contínuo, fixada quando a quadratura é zerada. */
  @NotLogged
  private double quadOffsetDeg = Double.NaN;
  /** Quadratura descartada (discordou do absoluto): volta para a derivada antiga até reiniciar. */
  @NotLogged
  private boolean quadFault = false;
  @NotLogged
  private double prevTargetDeg = 0.0;
  @NotLogged
  private double prevQuadDeg = 0.0;
  // Desenho da bancada (Elastic / AdvantageScope / Glass: SmartDashboard/Dial/Mecanismo)
  @NotLogged
  private final Mechanism2d mech = new Mechanism2d(3.0, 3.0);
  private static final double kMechCenter = 1.5;
  /** Distância do centro aos números da escala. */
  private static final double kLabelRadius = 1.34;
  /** Tamanho de cada dígito "de 7 segmentos" (unidades do desenho). */
  private static final double kDigitW = 0.06;
  private static final double kDigitH = 0.11;
  private static final double kDigitGap = 0.035;
  /** Raio onde começam as marcas da escala (o ponteiro tem 0,98). */
  private static final double kScaleRadius = 1.0;
  @NotLogged
  private final MechanismLigament2d ponteiro;
  @NotLogged
  private final MechanismLigament2d alvo;

  /** A bancada ({@link MechanismConfig#BANCADA}) no hardware real (ou na física do MechanismSim). */
  public MechanismSubsystem() {
    this(MechanismConfig.BANCADA);
  }

  /** Outra configuração no hardware da bancada (Spark MAX + Through Bore). */
  public MechanismSubsystem(MechanismConfig cfg) {
    // O quadro de controle a 5 ms (kFastPeriodMs) é essencial: a REVLib manda a tensão a cada 20 ms
    // por padrão e a malha de 5 ms só chegava ao motor a cada 20 ms (causa do "tremor").
    this(cfg, new MechanismIOSparkMax(cfg.currentLimitA, kFastPeriodMs, cfg.velKP, cfg.velOutMax,
        cfg.absInverted, cfg.absZeroOffsetDeg));
  }

  /** Qualquer mecanismo: a configuração dele e a camada de hardware dele. */
  public MechanismSubsystem(MechanismConfig cfg, MechanismIO io) {
    this.cfg = cfg;
    this.io = io;
    kP = tn("kP", cfg.kP);
    kI = tn("kI", cfg.kI);
    kD = tn("kD", cfg.kD);
    iZone = tn("IZoneDeg", cfg.iZoneDeg);
    iDeadband = tn("IDeadbandDeg", cfg.iDeadbandDeg);
    fastLoop = tn("FastLoop", 1.0);
    dFilterSec = tn("DFilterSec", cfg.dFilterSec);
    useQuadForD = tn("UseQuadForD", 1.0);
    kS = tn("kS", cfg.kS);
    kG = tn("kG", cfg.kG);
    outMax = tn("OutMax", cfg.outMax);
    testSetpointDeg = tn("TestSetpointDeg", 0.0);
    stepADeg = tn("StepADeg", 0.0);
    stepBDeg = tn("StepBDeg", 45.0);
    stepCDeg = tn("StepCDeg", 90.0);
    stepDDeg = tn("StepDDeg", 135.0);
    stepEDeg = tn("StepEDeg", 180.0);
    stepPeriodSec = tn("StepPeriodSec", 1.0);
    openLoopVolts = tn("OpenLoopVolts", 0.0);
    velKP = tn("VelkP", cfg.velKP);
    velKS = tn("VelkS", cfg.velKS);
    velKV = tn("VelkV", cfg.kV);
    kA = tn("kA", cfg.kA);
    velOutMax = tn("VelOutMax", cfg.velOutMax);
    velTargetRpm = tn("VelTargetRPM", 120.0);
    velStepARpm = tn("VelStepARPM", 60.0);
    velStepBRpm = tn("VelStepBRPM", 120.0);
    profMaxVelDegps = tn("ProfMaxVelDegps", cfg.profMaxVelDegps);
    profMaxAccelDegps2 = tn("ProfMaxAccelDegps2", cfg.profMaxAccelDegps2);
    oscGuard = tn("OscGuard", 1.0);
    oscSwingDeg = tn("OscSwingDeg", 1.0);
    oscMaxReversals = tn("OscMaxReversals", 8.0);
    oscMaxIntervalMs = tn("OscMaxIntervalMs", 70.0);
    dParadoFator = tn("DParadoFator", cfg.dHoldFactor);
    oscReset = tn("OscReset", 0.0);
    antiFolga = tn("AntiFolga", cfg.antiBacklashOn ? 1.0 : 0.0);
    antiFolgaDeg = tn("AntiFolgaDeg", cfg.antiBacklashDeg);
    antiFolgaFatorMax = tn("AntiFolgaFatorMax", cfg.antiBacklashFactorMax);
    limitsOn = tn("LimitsOn", cfg.limitsOn ? 1.0 : 0.0);
    limitLowDeg = tn("LimitLowDeg", cfg.limitLowDeg);
    limitHighDeg = tn("LimitHighDeg", cfg.limitHighDeg);
    limitMarginDeg = tn("LimitMarginDeg", cfg.limitMarginDeg);
    zeroRefDeg = tn("ZeroRefDeg", cfg.zeroRefDeg);
    absZeroOffsetDeg = tn("AbsZeroOffsetDeg", cfg.absZeroOffsetDeg);

    System.out.println("[Mecanismo] Hardware: " + io.description());
    io.configureVelocityGains(velKP.get(), velOutMax.get());
    io.updateInputs(in);

    desenharEscala();
    MechanismRoot2d centro = mech.getRoot("eixo", kMechCenter, kMechCenter);
    alvo = centro.append(new MechanismLigament2d("alvo", 1.0, 0.0, 2, new Color8Bit(Color.kGray)));
    ponteiro = centro.append(
        new MechanismLigament2d("ponteiro", 0.98, 270.0, 6, new Color8Bit(Color.kOrange)));
    SmartDashboard.putData(cfg.name + "/Mecanismo", mech); // na bancada: Dial/Mecanismo
  }

  /**
   * Escala de graus em volta do eixo, como a tabela da bancada: marca curta a cada 5°, média a
   * cada 15° (o passo dos botões A/B), longa e colorida em 0°, 90°, 180° e 270°. 0° = direita,
   * positivo anti-horário. (O Mechanism2d não desenha texto, por isso só marcas.)
   */
  private void desenharEscala() {
    for (int deg = 0; deg < 360; deg += 5) {
      boolean cardeal = deg % 90 == 0;
      boolean media = deg % 15 == 0;
      double comprimento = cardeal ? 0.16 : media ? 0.10 : 0.05;
      Color8Bit cor = cardeal ? new Color8Bit(Color.kCyan)
          : media ? new Color8Bit(Color.kWhite) : new Color8Bit(Color.kDimGray);
      double rad = Math.toRadians(deg);
      MechanismRoot2d raiz = mech.getRoot(String.format("marca%03d", deg),
          kMechCenter + kScaleRadius * Math.cos(rad), kMechCenter + kScaleRadius * Math.sin(rad));
      raiz.append(new MechanismLigament2d("m", comprimento, deg, cardeal ? 3 : 2, cor));
      if (deg % 30 == 0) {
        escrever(Integer.toString(deg), kMechCenter + kLabelRadius * Math.cos(rad),
            kMechCenter + kLabelRadius * Math.sin(rad), cardeal ? Color.kCyan : Color.kWhite);
      }
    }
  }

  /** Segmentos acesos de cada dígito (a b c d e f g, como num display de relógio). */
  private static final String[] kSegmentos = {
    "abcdef", "bc", "abged", "abgcd", "fgbc", "afgcd", "afgedc", "abc", "abcdefg", "abcdfg"
  };

  /**
   * "Escreve" um número com traços (o Mechanism2d não desenha texto), centrado em (cx, cy).
   * Cada segmento é uma raiz no ponto inicial com um traço horizontal (0°) ou vertical (90°).
   */
  private void escrever(String texto, double cx, double cy, Color cor) {
    double largura = texto.length() * kDigitW + (texto.length() - 1) * kDigitGap;
    double x0 = cx - largura / 2.0;
    double y0 = cy - kDigitH / 2.0;
    double meia = kDigitH / 2.0;
    for (int i = 0; i < texto.length(); i++) {
      double x = x0 + i * (kDigitW + kDigitGap);
      for (char seg : kSegmentos[texto.charAt(i) - '0'].toCharArray()) {
        // {x inicial, y inicial, ângulo, comprimento} de cada segmento
        double[] g = switch (seg) {
          case 'a' -> new double[] {x, y0 + kDigitH, 0, kDigitW};
          case 'b' -> new double[] {x + kDigitW, y0 + meia, 90, meia};
          case 'c' -> new double[] {x + kDigitW, y0, 90, meia};
          case 'd' -> new double[] {x, y0, 0, kDigitW};
          case 'e' -> new double[] {x, y0, 90, meia};
          case 'f' -> new double[] {x, y0 + meia, 90, meia};
          default -> new double[] {x, y0 + meia, 0, kDigitW}; // 'g'
        };
        MechanismRoot2d raiz = mech.getRoot(
            String.format("num%s_%d%c", texto, i, seg), g[0], g[1]);
        raiz.append(new MechanismLigament2d("s", g[3], g[2], 2, new Color8Bit(cor)));
      }
    }
  }

  // ===================== Leituras =====================

  /** Leitura bruta do Through Bore, 0 a 1 (sem inversão nem offset). */
  public double getAbsRaw() {
    return in.absRaw;
  }

  public boolean isAbsConnected() {
    return in.absConnected;
  }

  /** Frequência do PWM do Through Bore (~975 Hz quando conectado). */
  public double getAbsFrequencyHz() {
    return in.absFreqHz;
  }

  private double signedRawDeg() {
    double deg = in.absRaw * 360.0;
    return cfg.absInverted ? -deg : deg;
  }

  /** Ângulo do ponteiro, 0 a 360° (Through Bore). */
  public double getDeg() {
    return wrap360(signedRawDeg() - absZeroOffsetDeg.get());
  }

  public boolean isAbsoluteSensor() {
    return true;
  }

  /** Ângulo contínuo pelo Through Bore (sem salto em 360°). Só diferenças importam. */
  public double getContinuousDeg() {
    return contDeg;
  }

  /** Ângulo contínuo pelo encoder do NEO, para comparar com o Through Bore. */
  public double getMotorDeg() {
    return in.motorRot * 360.0 / cfg.gearRatio;
  }

  /** Ângulo contínuo pela quadratura (graus; só diferenças importam). */
  public double getQuadDeg() {
    return in.quadDeg;
  }

  /** Velocidade pela quadratura, em RPM (para comparar com getVelRpm). */
  public double getQuadVelRpm() {
    return in.quadRateDegps / 6.0;
  }

  /** Quadratura menos absoluto contínuo, desde que a quadratura foi zerada (deveria ficar ~0). */
  public double getQuadDriftDeg() {
    return Double.isNaN(quadOffsetDeg) ? 0.0 : in.quadDeg - contDeg - quadOffsetDeg;
  }

  public boolean isQuadFault() {
    return quadFault;
  }

  private boolean quadOk() {
    return useQuadForD.get() > 0.5 && !quadFault && !Double.isNaN(quadOffsetDeg);
  }

  /** Velocidade do ponteiro em RPM (derivada do Through Bore, média de 5 ciclos). */
  public double getVelRpm() {
    return velRpm;
  }

  /** Velocidade pelo encoder do NEO (é a que o PID de velocidade do Spark usa). */
  public double getMotorVelRpm() {
    return in.motorRpm / cfg.gearRatio;
  }

  public double getAbsZeroOffsetDeg() {
    return absZeroOffsetDeg.get();
  }

  public double getTargetRpm() {
    return mode == Mode.VELOCITY ? targetRpm : Double.NaN;
  }

  public double getVelErrorRpm() {
    return mode == Mode.VELOCITY ? targetRpm - getMotorVelRpm() : 0.0;
  }

  public String getMode() {
    return mode.name();
  }

  private boolean isPositionMode() {
    return mode == Mode.CLOSED_LOOP || mode == Mode.PROFILED;
  }

  /** Setpoint do instante, em 0–360°. */
  public double getTargetDeg() {
    return isPositionMode() ? wrap360(targetDeg) : Double.NaN;
  }

  /** Erro pelo caminho mais curto, de −180° a +180°. */
  public double getErrorDeg() {
    return isPositionMode() ? angErr(targetDeg, getDeg()) : 0.0;
  }

  public double getGoalDeg() {
    return mode == Mode.PROFILED ? wrap360(goalDeg) : Double.NaN;
  }

  public double getProfileAccelDegps2() {
    return mode == Mode.PROFILED ? profAccel : 0.0;
  }

  public double getProfileVelDegps() {
    return mode == Mode.PROFILED ? profState.velocity : 0.0;
  }

  public double getAppliedVolts() {
    return in.appliedVolts;
  }

  public double getCommandedVolts() {
    return commandedVolts;
  }

  public double getFFVolts() {
    return ffVolts;
  }

  /** Intervalo real entre passos do callback rápido (deveria ser ~5 ms). */
  public double getLoopDtMs() {
    return loopDtMs;
  }

  /** Período do callback rápido, média móvel, medido SEMPRE (deveria ser ~5 ms). */
  public double getLoopPeriodMs() {
    return loopPeriodMs;
  }

  /**
   * O laço rápido está lento (> kLoopSlowMs há mais de kLoopSlowSec)? Em 2026-11-13 a thread do
   * robô passou sozinha de 5 para 25,4 ms (e o Epilogue de 20 para 25,4 ms), com o robô parado,
   * depois de ~1 h de programa ligado: a passada foi de ~1° para ~3,5° com os mesmos ganhos.
   * "Restart Robot Code" resolveu. Causa não achada (não era o código do controle: o Epilogue
   * continuava levando 0,5 ms por execução).
   */
  public boolean isLoopSlow() {
    return !Double.isNaN(loopSlowSince)
        && Timer.getFPGATimestamp() - loopSlowSince > kLoopSlowSec;
  }

  /** Parte P (antes do limite OutMax), para a gravação a 5 ms. */
  @NotLogged
  public double getPTermVolts() {
    return pTermVolts;
  }

  /** Parte D (antes do limite OutMax), para a gravação a 5 ms. */
  @NotLogged
  public double getDTermVolts() {
    return dTermVolts;
  }

  /** Modo como número: 0 parado, 1 malha aberta, 2 posição, 3 perfil, 4 velocidade, 5 SysId. */
  @NotLogged
  public int getModeCode() {
    return mode.ordinal();
  }

  /** Parte do kI dentro de PidVolts. */
  public double getIntegralVolts() {
    return integralVolts;
  }

  public double getPidVolts() {
    return pidVolts;
  }

  public double getCurrentA() {
    return in.currentA;
  }

  /**
   * Tensão da bateria no controlador do motor (V). O controlador compensa a tensão (12 V), mas só
   * até onde a bateria tem: se ela afunda perto do que o perfil pede, o mecanismo perde força.
   */
  public double getBusVolts() {
    return in.busVolts;
  }

  /** Tensão da bateria vista pela roboRIO (V). */
  public double getRioVolts() {
    return RobotController.getBatteryVoltage();
  }

  /** A roboRIO entrou em brownout (bateria abaixo de ~6,8 V: saídas cortadas)? */
  public boolean isBrownedOut() {
    return RobotController.isBrownedOut();
  }

  /** Temperatura do NEO (°C), lida pelo Spark. */
  public double getMotorTempC() {
    return in.tempC;
  }

  /** Resistência do NEO: 12 V / 105 A (corrente com o rotor travado). */
  private static final double kMotorOhms = 12.0 / 105.0;

  /**
   * Corrente acima de limitA, só contando leituras FISICAMENTE possíveis. Com pouca tensão o Spark
   * lê saltos falsos (21,8 A lidos com 0,74 V em 2026-11-10, quando o máximo real é
   * 0,74 / 0,114 ≈ 6,5 A): a leitura só vale se |V| / R alcançar limitA. Abaixo disso quem
   * protege é o limite de corrente do próprio Spark (cfg.currentLimitA).
   */
  @NotLogged
  public boolean isCurrentAbove(double limitA) {
    return Math.abs(commandedVolts) / kMotorOhms >= limitA && getCurrentA() > limitA;
  }

  // ===================== Utilidades de ângulo =====================

  private static double wrap360(double deg) {
    return MathUtil.inputModulus(deg, 0.0, 360.0);
  }

  private static double wrap180(double deg) {
    return MathUtil.inputModulus(deg, -180.0, 180.0);
  }

  // ===================== Limites de curso =====================

  /** Limites ligados e com uma faixa útil (maior que 1° depois das folgas)? */
  public boolean limitsActive() {
    return limitsOn.get() > 0.5 && arcSpan() > 1.0;
  }

  /** Limite suave do lado negativo (batente − folga), 0–360°. */
  private double softLow() {
    return wrap360(limitLowDeg.get() + limitMarginDeg.get());
  }

  /** Tamanho da faixa permitida entre os limites suaves (graus, anti-horário). */
  private double arcSpan() {
    return MathUtil.inputModulus(limitHighDeg.get() - limitLowDeg.get(), 0.0, 360.0)
        - 2.0 * limitMarginDeg.get();
  }

  /**
   * Posição dentro da faixa: 0 = limite suave negativo, arcSpan() = positivo. Fora da faixa dá
   * negativo ou maior que arcSpan(), pelo lado do limite mais perto.
   */
  private double arcPos(double deg) {
    double d = MathUtil.inputModulus(deg - softLow(), 0.0, 360.0);
    double span = arcSpan();
    if (d > span && d - span > 360.0 - d) {
      d -= 360.0;
    }
    return d;
  }

  /** Ângulo trazido para dentro da faixa (sem limites, devolve o mesmo). */
  private double clampToArc(double deg) {
    if (!limitsActive()) return deg;
    return wrap360(softLow() + MathUtil.clamp(arcPos(deg), 0.0, arcSpan()));
  }

  /** Erro alvo − medida: sem limites, pelo caminho mais curto; com limites, por dentro da faixa. */
  private double angErr(double target, double meas) {
    return limitsActive() ? arcPos(target) - arcPos(meas) : wrap180(target - meas);
  }

  /** Com limites: o sinal (tensão ou velocidade) empurra para FORA de um limite já alcançado? */
  private boolean pushingOut(double signal) {
    if (!limitsActive() || signal == 0.0) return false;
    double p = arcPos(getDeg());
    return (p <= 0.0 && signal < 0.0) || (p >= arcSpan() && signal > 0.0);
  }

  /** Alvo trazido para dentro da faixa permitida (sem limites ligados, devolve o mesmo, 0–360°). */
  public double clampTargetDeg(double deg) {
    return clampToArc(wrap360(deg));
  }

  /**
   * Quanto dá para andar a partir do ângulo atual sem passar de um limite suave, em cada sentido
   * (graus, ≥ 0): {para o lado negativo, para o lado positivo}. Sem limites: {∞, ∞}.
   */
  public double[] roomDeg() {
    if (!limitsActive()) {
      return new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
    }
    double p = arcPos(getDeg());
    return new double[] {Math.max(0.0, p), Math.max(0.0, arcSpan() - p)};
  }

  /** Limite suave negativo (NaN sem limites), para o painel. */
  public double getSoftLowDeg() {
    return limitsActive() ? softLow() : Double.NaN;
  }

  /** Limite suave positivo (NaN sem limites), para o painel. */
  public double getSoftHighDeg() {
    return limitsActive() ? wrap360(limitHighDeg.get() - limitMarginDeg.get()) : Double.NaN;
  }

  /** Grava os batentes medidos (ângulos de contato, 0–360°) e liga ou desliga os limites. */
  public void setLimits(double lowDeg, double highDeg, boolean on) {
    limitLowDeg.set(wrap360(lowDeg));
    limitHighDeg.set(wrap360(highDeg));
    limitsOn.set(on ? 1.0 : 0.0);
  }

  /** Liga/desliga os limites sem mexer nos valores (o MechanismLimitFinder desliga enquanto procura). */
  public void setLimitsOn(boolean on) {
    limitsOn.set(on ? 1.0 : 0.0);
  }

  @NotLogged
  public boolean isLimitsOn() {
    return limitsOn.get() > 0.5;
  }

  @NotLogged
  public double getLimitMarginDeg() {
    return limitMarginDeg.get();
  }

  /** Muda o limite de corrente do Spark (o MechanismLimitFinder usa um menor para não forçar). */
  public void setCurrentLimitA(int amps) {
    io.setCurrentLimit(amps);
  }

  /** Volta ao limite de corrente normal (cfg.currentLimitA). */
  public void restoreCurrentLimit() {
    setCurrentLimitA(cfg.currentLimitA);
  }

  // ===================== Controle =====================

  /** Tensão que segura o peso no ângulo dado, pelo modelo de gravidade do mecanismo (kG·fator). */
  public double gravityFFVolts(double deg) {
    return kG.get() * cfg.gravity.factor(deg);
  }

  /** A configuração do mecanismo (só leitura). */
  @NotLogged
  public MechanismConfig getConfig() {
    return cfg;
  }

  /** Cria um número ajustável em /Tuning/{nome do mecanismo}/{chave}. */
  private TunableNumber tn(String key, double defaultValue) {
    return new TunableNumber(cfg.name + "/" + key, defaultValue);
  }

  private void stop() {
    mode = Mode.STOPPED;
    commandedVolts = 0.0;
    ffVolts = 0.0;
    pidVolts = 0.0;
    integralVolts = 0.0;
    io.stop();
  }

  /**
   * BRAKE ou COAST conforme a config, o botão X e o SysId. Só reconfigura o Spark quando muda
   * (configure() vai pelo CAN e espera resposta: nunca a cada ciclo).
   */
  private void applyIdleMode() {
    boolean want = sysIdSession || (cfg.brakeWhenStopped && !released);
    if (want != brakeMode) {
      io.setBrake(want);
      brakeMode = want;
    }
  }

  /**
   * Solta o mecanismo: motor parado e em COAST até o próximo movimento (botão X). Para mover com
   * a mão ou deixar pendurar antes de calibrar o zero. Funciona também desabilitado.
   */
  public void release() {
    stop();
    released = true;
    applyIdleMode();
  }

  /** O motor está em BRAKE agora? */
  public boolean isBrakeMode() {
    return brakeMode;
  }

  private void setVolts(double volts) {
    released = false;
    mode = Mode.OPEN_LOOP;
    commandedVolts = pushingOut(volts) ? 0.0 : MathUtil.clamp(volts, -kNominalVolts, kNominalVolts);
    ffVolts = 0.0;
    pidVolts = 0.0;
    io.setVoltage(commandedVolts);
  }

  /** Sem sensor, nada de malha fechada de posição. */
  private boolean sensorOk() {
    if (in.absConnected) {
      warnedDisconnected = false;
      return true;
    }
    if (!warnedDisconnected) {
      System.out.println("[Mecanismo] Sensor absoluto DESCONECTADO: motor parado");
      warnedDisconnected = true;
    }
    stop();
    return false;
  }

  private void enterPositionMode(Mode m) {
    released = false;
    if (!isPositionMode()) {
      oscDetector.reset(); // a contagem de viradas recomeça
      approachForDeg = Double.NaN; // a aproximação é decidida de novo para o próximo pedido
      errIntegral = 0.0;
      firstStep = true; // sem "chute" da derivada no primeiro passo
    }
    mode = m;
  }

  private boolean useFastLoop() {
    return fastLoop.get() > 0.5;
  }

  /**
   * Pedido de posição SEM perfil. Os comandos chamam a cada 20 ms; quem calcula a tensão é
   * controlStep() — aqui mesmo (FastLoop = 0) ou no callback rápido de 5 ms (FastLoop = 1).
   */
  private void setTarget(double deg) {
    double req = clampToArc(wrap360(deg));
    if (oscBlocks(req)) return;
    if (!sensorOk()) return;
    enterPositionMode(Mode.CLOSED_LOOP);
    targetDeg = req;
    requestTime = Timer.getFPGATimestamp();
    if (!useFastLoop()) controlStep(0.02);
  }

  /** Pedido de posição COM perfil (o perfil avança dentro de controlStep). */
  private void setProfiledGoal(double deg) {
    double req = clampToArc(wrap360(deg));
    if (oscBlocks(req)) return;
    if (!sensorOk()) return;
    enterPositionMode(Mode.PROFILED);
    goalRequestDeg = req;
    requestTime = Timer.getFPGATimestamp();
    if (!useFastLoop()) controlStep(0.02);
  }

  /** Reinicia o perfil a partir de onde o ponteiro está agora. */
  private void resetProfile() {
    profState = new TrapezoidProfile.State(getDeg(), getVelRpm() * 6.0); // RPM → graus/s
    profAccel = 0.0;
  }

  /**
   * Chamado pelo Robot a cada kFastPeriodSec (addPeriodic). Só age com o robô habilitado, em modo
   * posição e com um pedido recente de um comando (se o comando acabar, o motor para sozinho).
   */
  public void fastLoop() {
    double now = Timer.getFPGATimestamp();
    double dt = lastFastTime < 0 ? kFastPeriodSec : now - lastFastTime;
    lastFastTime = now;
    watchLoopPeriod(now, dt);
    // Simulação: física primeiro (motor parado em COAST = solto, sem freio). Depois todos os
    // sensores são lidos de uma vez: o controle deste ciclo usa essa leitura.
    io.simulationStep(MathUtil.clamp(dt, 0.001, 0.03), mode == Mode.STOPPED && !brakeMode);
    io.updateInputs(in);
    fastStep(now, dt);
    if (fastLog != null) fastLog.record(this, dt);
  }

  /** Vigia o período do laço rápido e avisa no Driver Station se ficar lento (ver isLoopSlow). */
  private void watchLoopPeriod(double now, double dt) {
    if (dt <= 0.0 || dt > 0.5) return; // primeiro passo / pausa longa (ex.: deploy)
    loopPeriodMs += (dt * 1000.0 - loopPeriodMs) * 0.02; // média de ~50 passos
    if (loopPeriodMs <= kLoopSlowMs) {
      loopSlowSince = Double.NaN;
      return;
    }
    if (Double.isNaN(loopSlowSince)) loopSlowSince = now;
    if (now - loopSlowSince > kLoopSlowSec && now - loopWarnTime > 60.0) {
      loopWarnTime = now;
      DriverStation.reportWarning(String.format("[Mecanismo] Laço de controle LENTO: %.1f ms "
          + "(deveria ser %.0f ms) — o mecanismo passa mais do alvo. Reinicie o código do robô "
          + "(Restart Robot Code no Driver Station).", loopPeriodMs, kFastPeriodSec * 1000.0),
          false);
    }
  }

  private void fastStep(double now, double dt) {
    if (mode == Mode.SYSID) {
      sysIdStep(now);
      return;
    }
    if (!useFastLoop() || !isPositionMode()) return;
    if (!DriverStation.isEnabled() || now - requestTime > kRequestTimeoutSec) {
      stop();
      return;
    }
    if (!sensorOk()) return;
    loopDtMs = dt * 1000.0;
    if (oscGuardStep(now)) return;
    controlStep(MathUtil.clamp(dt, 0.001, 0.03));
  }

  /**
   * Nível 2 da proteção: o motor fica parado enquanto o pedido for o MESMO alvo de quando parou;
   * um pedido para outro alvo (o operador mandou o mecanismo para outro lugar) libera, com o D
   * ainda desligado (nível 1).
   *
   * @return true se o pedido foi bloqueado (motor parado)
   */
  private boolean oscBlocks(double reqDeg) {
    if (oscLevel < 2) return false;
    if (Math.abs(angErr(reqDeg, oscStopGoalDeg)) <= 0.5) {
      stop();
      return true;
    }
    oscLevel = 1;
    oscDetector.reset();
    System.out.printf("[Mecanismo] Novo alvo (%.1f°): volta a controlar, ainda SEM o D%n", reqDeg);
    return false;
  }

  /**
   * Proteção contra oscilação, no laço de 5 ms (o NT pelo Wi-Fi não vê 12 Hz: a vigia do
   * robusto.sh deixou passar um ciclo-limite de ±3,5° a 12,5 Hz em 270° — 2026-11-12).
   *
   * <p>Detecta só oscilação RÁPIDA (MechanismOscDetector: OscMaxReversals viradas maiores que
   * OscSwingDeg seguidas, cada uma até OscMaxIntervalMs depois da anterior; pela quadratura, sem
   * ela pelo absoluto). Mão, tranco e mudança de sentido do operador (3–5 Hz) não disparam.
   *
   * <p>A mão balançando MUITO rápido na folga (40–75 ms, 2026-11-13) é igual ao ciclo-limite
   * pelo sensor (nem o sinal de tensão × velocidade separa: o ponteiro bate na folga nos dois
   * casos). Então o que muda é a REAÇÃO, como as equipes de ponta (na partida o mecanismo fica
   * pior, nunca morto):
   * <ul>
   *   <li>Disparou: desliga o D (é o D que sustenta o ciclo-limite; sem o D parado, 0 de 10
   *       oscilaram em 270°) e continua controlando (nível 1).
   *   <li>Operador no controle: o D VOLTA sozinho depois de kOscRestoreSec sem nenhuma virada; se
   *       disparar kOscMaxStrikes vezes em kOscStrikeWindowSec com o D ligado (o ciclo-limite volta
   *       segundos depois do D; trancos numa partida são espaçados), fica sem D até desabilitar.
   *       Nunca para o
   *       motor.
   *   <li>Teste automático (oscStrict, RemoteCmd ≠ 0 — os scripts testam ganhos ruins de
   *       propósito): disparar de novo sem o D para o motor (nível 2) até pedir OUTRO alvo.
   * </ul>
   * isOscTripped fica true desde o 1º disparo (os scripts contam; OscReset zera).
   *
   * @return true se o motor está parado pela proteção
   */
  private boolean oscGuardStep(double now) {
    if (oscLevel >= 2) {
      stop();
      return true;
    }
    if (oscGuard.get() < 0.5) return false;
    while (!oscStrikeTimes.isEmpty() && now - oscStrikeTimes.peekFirst() > kOscStrikeWindowSec
        && !oscDNeverBack) {
      oscStrikeTimes.removeFirst();
    }
    if (oscLevel == 1 && !oscStrict && !oscDNeverBack
        && now - Math.max(oscTripTime, oscDetector.lastReversal()) > kOscRestoreSec) {
      oscLevel = 0;
      System.out.printf("[Mecanismo] Parou de vibrar: D de volta (disparos: %d de %d)%n",
          oscStrikeTimes.size(), kOscMaxStrikes);
    }
    double x;
    if (quadOk()) {
      x = in.quadDeg;
    } else { // absoluto em graus contínuos (soma as diferenças pelo caminho mais curto)
      double d = getDeg();
      oscContDeg += Double.isNaN(oscLastDeg) ? 0.0 : wrap180(d - oscLastDeg);
      oscLastDeg = d;
      x = oscContDeg;
    }
    double swing = Math.max(0.1, oscSwingDeg.get());
    int needed = (int) Math.max(3.0, oscMaxReversals.get());
    double maxInterval = Math.max(0.01, oscMaxIntervalMs.get() / 1000.0);
    if (!oscDetector.step(now, x, swing, maxInterval, needed)) return false;
    oscTripped = true;
    oscTripTime = now;
    oscDetector.reset();
    if (oscLevel == 0) {
      oscLevel = 1;
      oscStrikeTimes.addLast(now);
      if (oscStrikeTimes.size() >= kOscMaxStrikes) oscDNeverBack = true;
      System.out.printf("[Mecanismo] VIBRANDO (%d viradas > %.1f°, a cada <= %.0f ms) em %.1f°: "
          + "D DESLIGADO (o mecanismo continua funcionando; %s)%n", needed, swing,
          maxInterval * 1000.0, getDeg(), oscStrict ? "teste automático"
              : !oscDNeverBack ? "volta depois de " + kOscRestoreSec + " s calmo"
              : "disparou " + kOscMaxStrikes + " vezes em " + kOscStrikeWindowSec
                  + " s: fica sem D até desabilitar");
      return false;
    }
    if (!oscStrict) return false; // operador: nunca para o motor (o D já está desligado)
    oscLevel = 2;
    oscStopGoalDeg = mode == Mode.PROFILED ? goalRequestDeg : wrap360(targetDeg);
    stop();
    System.out.printf("[Mecanismo] OSCILANDO de novo, mesmo sem o D, em %.1f°: motor PARADO até "
        + "pedir outro alvo%n", getDeg());
    return true;
  }

  /** Zera a proteção contra oscilação (ao desabilitar o robô ou com OscReset). */
  private void resetOscGuard() {
    oscTripped = false;
    oscLevel = 0;
    oscStrikeTimes.clear();
    oscDNeverBack = false;
    oscTripTime = Double.NEGATIVE_INFINITY;
    oscStopGoalDeg = Double.NaN;
    oscLastDeg = Double.NaN;
    oscContDeg = 0.0;
    oscDetector.reset();
  }

  /** A proteção contra oscilação disparou desde que habilitou (D desligado ou motor parado)? */
  public boolean isOscTripped() {
    return oscTripped;
  }

  /** 0 = normal; 1 = D desligado pela proteção; 2 = motor parado até pedir outro alvo. */
  public int getOscLevel() {
    return oscLevel;
  }

  /**
   * Teste automático rodando (RemoteCmd ≠ 0)? Só então a proteção pode parar o motor; com o
   * operador ela no máximo desliga o D, que volta sozinho (ver oscGuardStep).
   */
  public void setOscStrict(boolean strict) {
    oscStrict = strict;
  }

  /** Liga a gravação a 5 ms (chamado pelo Robot depois de iniciar o DataLogManager). */
  public void enableFastLog(MechanismFastLog log) {
    fastLog = log;
  }

  /** Movimentos menores que isto não usam a aproximação anti-folga (ex.: ajuste fino). */
  private static final double kApproachMinMoveDeg = 3.0;
  /** Gravidade "nula" (|fator| abaixo disto, ex.: a ±3° da vertical): sem lado preferido. */
  private static final double kApproachMinFactor = 0.05;

  /**
   * Anti-folga: decide, uma vez por pedido novo, se o perfil passa do destino e volta. Só quando a
   * gravidade no destino é fraca (perto da vertical ela não segura o ponteiro contra o dente na
   * frenagem e ele atravessa a folga, passando 3–5° do alvo — 2026-11-12) e o movimento chegaria
   * CONTRA ela. Voltando a favor da gravidade, frear e segurar usam o mesmo lado do dente.
   */
  private void planApproach() {
    approachForDeg = goalRequestDeg;
    approachPending = false;
    approachOffsetDeg = 0.0;
    if (antiFolga.get() < 0.5) return;
    double move = goalDeg - profState.position;
    if (Math.abs(move) < kApproachMinMoveDeg) return;
    double f = cfg.gravity.factor(goalRequestDeg);
    if (Math.abs(f) < kApproachMinFactor || Math.abs(f) >= antiFolgaFatorMax.get()) return;
    double gravidade = -Math.signum(f); // sentido (em graus) para onde a gravidade puxa
    if (Math.signum(move) == gravidade) return; // já chega a favor da gravidade
    double off = -gravidade * Math.max(0.0, antiFolgaDeg.get());
    if (limitsActive()) {
      double p = arcPos(wrap360(goalRequestDeg + off));
      if (p < 0.0 || p > arcSpan()) return; // o ponto intermediário sairia da faixa permitida
    }
    approachOffsetDeg = off;
    approachPending = true;
  }

  /** O perfil está indo ao ponto intermediário da aproximação anti-folga? */
  public boolean isApproaching() {
    return mode == Mode.PROFILED && approachPending;
  }

  /** Um passo da malha de posição: avança o perfil (se houver), FF + PID → tensão. */
  private void controlStep(double dt) {
    double meas = getDeg();
    double gravity = gravityFFVolts(meas);
    double ff;
    boolean allowIntegral;

    if (mode == Mode.PROFILED) {
      // O perfil é contínuo: escolhe a cópia do destino (±360°) mais perto de onde ele está
      // Com limites: anda por DENTRO da faixa (o caminho mais curto pode atravessar um batente)
      goalDeg = profState.position + angErr(goalRequestDeg, profState.position);
      if (goalRequestDeg != approachForDeg) {
        planApproach();
      }
      // Anti-folga: primeiro até o ponto intermediário, depois o destino (a favor da gravidade)
      double profGoal = approachPending ? goalDeg + approachOffsetDeg : goalDeg;
      TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
          Math.max(1.0, profMaxVelDegps.get()), Math.max(1.0, profMaxAccelDegps2.get())));
      double prevVel = profState.velocity;
      profState = profile.calculate(dt, profState, new TrapezoidProfile.State(profGoal, 0.0));
      profAccel = (profState.velocity - prevVel) / dt;
      targetDeg = profState.position;
      if (approachPending && Math.abs(profState.position - profGoal) < 1e-6
          && Math.abs(profState.velocity) < 1e-6) {
        approachPending = false; // chegou ao ponto intermediário: agora volta para o destino
      }

      double velDegps = profState.velocity;
      double error = angErr(targetDeg, meas);
      double motion;
      if (Math.abs(velDegps) > 1.0) {
        motion = velKS.get() * Math.signum(velDegps) + velKV.get() * (velDegps / 6.0)
            + kA.get() * profAccel;
      } else {
        motion = Math.abs(error) > 0.5 ? kS.get() * Math.signum(error) : 0.0;
      }
      ff = gravity + motion;
      allowIntegral = Math.abs(profState.position - goalDeg) < 1e-6
          && Math.abs(profState.velocity) < 1e-6;
    } else {
      double error = angErr(targetDeg, meas);
      double friction = Math.abs(error) > 0.5 ? kS.get() * Math.signum(error) : 0.0;
      ff = gravity + friction;
      allowIntegral = true;
    }
    driveWithPid(ff, meas, allowIntegral, dt);
  }

  /**
   * FF + PID → tensão no Spark. P, I e D feitos aqui (não no PIDController) porque o passo dt
   * muda entre 5 ms e 20 ms.
   *
   * @param allowIntegral false = integral zerada. No perfil, integrar durante o movimento
   *     carrega a integral (o erro de seguimento fica dentro do IZone) e, ao chegar, o ponteiro fica
   *     "caçando" o alvo (testado: ±2° a 3 Hz). Por isso o perfil só libera o kI ao terminar.
   */
  private void driveWithPid(double ff, double meas, boolean allowIntegral, double dt) {
    ffVolts = MathUtil.clamp(ff, -cfg.maxFFVolts, cfg.maxFFVolts);
    double error = angErr(wrap360(targetDeg), meas);

    // Derivada do erro, com passa-baixa (cada passo do encoder vira um pico em 5 ms)
    if (firstStep) {
      prevError = error;
      prevTargetDeg = targetDeg;
      prevQuadDeg = in.quadDeg;
      errDeriv = 0.0;
      firstStep = false;
    }
    double rawDeriv;
    double quadDeg = in.quadDeg;
    if (quadOk()) {
      // d(erro)/dt = d(alvo)/dt − velocidade pela quadratura. A velocidade vem da DIFERENÇA de
      // posição a cada passo (degraus de 0,044°/5 ms = 9°/s), não de Encoder.getRate(): o
      // getRate mede o tempo entre pulsos e, com o ponteiro parado, repete a última velocidade —
      // o kD empurrava um ponteiro parado e ele oscilava ±0,7° a 6 Hz (testado na bancada).
      double targetRate = wrap180(targetDeg - prevTargetDeg) / dt;
      rawDeriv = targetRate - (quadDeg - prevQuadDeg) / dt;
    } else {
      rawDeriv = wrap180(error - prevError) / dt;
    }
    prevQuadDeg = quadDeg;
    prevTargetDeg = targetDeg;
    prevError = error;
    double tau = Math.max(0.0, dFilterSec.get());
    errDeriv += (rawDeriv - errDeriv) * (dt / (tau + dt));

    // Integral com IZone, zona morta e anti-windup
    double ki = kI.get();
    if (!allowIntegral || ki <= 0.0 || Math.abs(error) > iZone.get()) {
      errIntegral = 0.0;                       // longe do alvo ou perfil andando: zera
    } else if (Math.abs(error) > iDeadband.get()) {
      errIntegral += error * dt;               // perto, mas fora da zona morta: integra
    }                                          // dentro da zona morta: congela
    double maxI = ki > 0.0 ? outMax.get() / ki : 0.0; // a parte I sozinha nunca passa de OutMax
    errIntegral = MathUtil.clamp(errIntegral, -maxI, maxI);
    double iOut = ki * errIntegral;
    integralVolts = iOut * kNominalVolts;

    // D parado: com o perfil terminado e perto do alvo, o D só reage ao ponteiro solto na folga
    // (gravidade fraca) e pode sustentar um ciclo-limite (12,5 Hz em 270°, 2026-11-12)
    double kd = kD.get();
    if (allowIntegral && Math.abs(error) < iZone.get()) {
      kd *= MathUtil.clamp(dParadoFator.get(), 0.0, 1.0);
    }
    if (oscLevel >= 1) kd = 0.0; // a proteção contra oscilação desligou o D (oscGuardStep)
    pTermVolts = kP.get() * error * kNominalVolts;
    dTermVolts = kd * errDeriv * kNominalVolts;
    double out = MathUtil.clamp(kP.get() * error + kd * errDeriv + iOut,
        -outMax.get(), outMax.get());
    pidVolts = out * kNominalVolts;
    commandedVolts = MathUtil.clamp(ffVolts + pidVolts, -kNominalVolts, kNominalVolts);
    io.setVoltage(commandedVolts);
  }

  /** Um passo do SysId: aplica a tensão pedida e grava uma amostra. */
  private void sysIdStep(double now) {
    if (!DriverStation.isEnabled() || now - requestTime > kRequestTimeoutSec) {
      stop();
      return;
    }
    double wrapped = getDeg();
    if (Double.isNaN(sysIdLastWrapped)) {
      sysIdCont = wrapped;
    } else {
      sysIdCont += wrap180(wrapped - sysIdLastWrapped);
    }
    sysIdLastWrapped = wrapped;

    commandedVolts = pushingOut(sysIdVolts) ? 0.0 : sysIdVolts;
    io.setVoltage(commandedVolts);
    if (sysIdN < kSysIdMaxSamples) {
      double[] r = sysIdBuf[sysIdN++];
      r[0] = now;
      r[1] = sysIdTest;
      r[2] = sysIdVolts;
      r[3] = getAppliedVolts();
      r[4] = sysIdCont;
      r[5] = getMotorVelRpm();
    }
  }

  /** Começa uma sessão de SysId: limpa a gravação e põe o motor em BRAKE (freia entre testes). */
  public void sysIdBegin() {
    sysIdN = 0;
    sysIdLastWrapped = Double.NaN;
    sysIdSession = true;
    released = false;
    applyIdleMode();
  }

  /** Pede tensão em malha aberta para o teste {@code test} (chamar a cada ciclo). */
  public void sysIdRun(int test, double volts) {
    if (!sensorOk()) return;
    if (mode != Mode.SYSID || test != sysIdTest) {
      sysIdLastWrapped = Double.NaN; // o ângulo contínuo recomeça a cada teste
    }
    mode = Mode.SYSID;
    sysIdTest = test;
    sysIdVolts = MathUtil.clamp(volts, -kNominalVolts, kNominalVolts);
    requestTime = Timer.getFPGATimestamp();
  }

  /** Termina a sessão: motor parado, volta ao modo da config (BRAKE/COAST) e grava o CSV. */
  public void sysIdEnd(String path) {
    stop();
    sysIdSession = false;
    applyIdleMode();
    try (PrintWriter w = new PrintWriter(new FileWriter(path))) {
      w.println("t,test,cmdV,appV,posDeg,neoRpm");
      for (int i = 0; i < sysIdN; i++) {
        double[] r = sysIdBuf[i];
        w.printf("%.5f,%d,%.4f,%.4f,%.4f,%.2f%n", r[0], (int) r[1], r[2], r[3], r[4], r[5]);
      }
      System.out.println("[SysId] " + sysIdN + " amostras gravadas em " + path);
    } catch (IOException e) {
      System.out.println("[SysId] erro ao gravar " + path + ": " + e.getMessage());
    }
  }

  public int getSysIdSamples() {
    return sysIdN;
  }

  private void setVelocity(double rpm) {
    if (pushingOut(rpm)) {
      stop();
      return;
    }
    released = false;
    mode = Mode.VELOCITY;
    targetRpm = MathUtil.clamp(rpm, -cfg.maxVelRpm, cfg.maxVelRpm);
    pidVolts = 0.0;

    ffVolts = Math.abs(targetRpm) < 1e-3
        ? 0.0
        : velKS.get() * Math.signum(targetRpm) + velKV.get() * targetRpm;
    ffVolts = MathUtil.clamp(ffVolts, -kNominalVolts, kNominalVolts);
    commandedVolts = ffVolts;

    io.setVelocity(targetRpm * cfg.gearRatio, ffVolts);
  }

  private void updateGainsIfChanged() {
    // kP/kI/kD/IZone são lidos direto a cada passo da malha (na RIO: barato)
    if (velKP.hasChanged() | velOutMax.hasChanged()) {
      io.configureVelocityGains(velKP.get(), velOutMax.get());
    }
    if (absZeroOffsetDeg.hasChanged()) {
      contInit = false; // o ângulo saltou: recomeça o ângulo contínuo
    }
  }

  /** Atualiza o ângulo contínuo e a velocidade a partir do Through Bore. */
  private void updateContinuous() {
    double now = Timer.getFPGATimestamp();
    double wrapped = getDeg();
    if (!contInit || !in.absConnected) {
      contInit = in.absConnected;
      lastWrappedDeg = wrapped;
      lastTime = now;
      contDeg = wrapped;
      velFilter.reset();
      velRpm = 0.0;
      quadOffsetDeg = Double.NaN; // a quadratura é recasada com o absoluto no próximo ciclo
      return;
    }
    double delta = wrap180(wrapped - lastWrappedDeg);
    double dt = now - lastTime;
    contDeg += delta;
    lastWrappedDeg = wrapped;
    lastTime = now;
    if (dt > 1e-4) {
      velRpm = velFilter.calculate(delta / dt / 6.0); // graus/s → RPM
    }

    // Confere a quadratura contra o absoluto: se um fio soltar ou a contagem estiver invertida,
    // ela para de concordar e é descartada (o PID volta para a derivada antiga)
    if (Double.isNaN(quadOffsetDeg)) {
      quadOffsetDeg = in.quadDeg - contDeg;
    } else if (!quadFault && Math.abs(getQuadDriftDeg()) > cfg.quadMaxDriftDeg) {
      quadFault = true;
      System.out.printf("[Mecanismo] Quadratura DESCARTADA: discordou %.1f° do absoluto. Confira os "
          + "fios A/B e kQuadInverted (%s)%n", getQuadDriftDeg(), io.description());
    }
  }

  /** Calcula o offset para que o ângulo ATUAL passe a valer ZeroRefDeg. */
  private void calibrateAbsZero() {
    if (!in.absConnected) {
      System.out.println("[Mecanismo] Zero NÃO calibrado: Through Bore desconectado");
      return;
    }
    double offset = wrap360(signedRawDeg() - zeroRefDeg.get());
    absZeroOffsetDeg.set(offset);
    contInit = false;
    System.out.printf(
        "[Mecanismo] Zero calibrado: ponteiro = %.1f°. Copie para a configuração do mecanismo: "
            + "MechanismConfig absZeroOffsetDeg(%.2f)%n", zeroRefDeg.get(), offset);
  }

  // ===================== API do autoajuste (usada pelo ArmAutoTuner) =====================

  public void tunerVolts(double volts) {
    setVolts(volts);
  }

  public void tunerPosition(double deg) {
    setTarget(deg);
  }

  public void tunerVelocity(double rpm) {
    setVelocity(rpm);
  }

  public void tunerStop() {
    stop();
  }

  /** Reinicia o perfil a partir do estado atual (chamar antes de uma sequência de perfis). */
  public void tunerProfileStart() {
    resetProfile();
  }

  /** Posição COM perfil até deg (chamar a cada ciclo). */
  public void tunerProfiled(double deg) {
    setProfiledGoal(deg);
  }

  /** O perfil já chegou ao destino (setpoint parado no alvo)? */
  public boolean tunerProfileDone() {
    return mode == Mode.PROFILED
        && Math.abs(profState.position - goalDeg) < 1e-6 && Math.abs(profState.velocity) < 1e-6;
  }

  /** Com o ponteiro pendurado e parado: o ângulo está a ±15° de ZeroRefDeg? */
  public boolean tunerCheckHanging() {
    return in.absConnected && Math.abs(wrap180(getDeg() - zeroRefDeg.get())) <= 15.0;
  }

  @NotLogged
  public double getKP() {
    return kP.get();
  }

  @NotLogged
  public double getKI() {
    return kI.get();
  }

  @NotLogged
  public double getKD() {
    return kD.get();
  }

  @NotLogged
  public double getKG() {
    return kG.get();
  }

  @NotLogged
  public double getVelKS() {
    return velKS.get();
  }

  @NotLogged
  public double getVelKV() {
    return velKV.get();
  }

  @NotLogged
  public double getVelOutMax() {
    return velOutMax.get();
  }

  public void setKG(double v) {
    kG.set(v);
  }

  public void setKS(double v) {
    kS.set(v);
  }

  public void setKP(double v) {
    kP.set(v);
  }

  public void setKI(double v) {
    kI.set(v);
  }

  public void setKD(double v) {
    kD.set(v);
  }

  public void setVelKS(double v) {
    velKS.set(v);
  }

  public void setVelKV(double v) {
    velKV.set(v);
  }

  public void setVelKP(double v) {
    velKP.set(v);
  }

  public void setVelOutMax(double v) {
    velOutMax.set(v);
  }

  // ===================== Comandos =====================

  @NotLogged
  public Command stopCmd() {
    return run(this::stop).withName("Parado");
  }

  @NotLogged
  public Command holdTunableSetpointCmd() {
    return run(() -> setTarget(testSetpointDeg.get())).withName("Setpoint");
  }

  /** Sequência de degraus em vai-e-volta: A → B → C → D → E → D → C → B → A ... */
  private double sequenceTarget(double elapsedSec) {
    double[] pontos = {stepADeg.get(), stepBDeg.get(), stepCDeg.get(), stepDDeg.get(), stepEDeg.get()};
    int n = pontos.length;
    int ciclo = 2 * (n - 1);
    long passo = (long) Math.floor(elapsedSec / Math.max(0.2, stepPeriodSec.get())) % ciclo;
    int indice = (int) (passo < n ? passo : ciclo - passo);
    return pontos[indice];
  }

  @NotLogged
  public Command stepTestCmd() {
    return startRun(
        () -> stepStartTime = Timer.getFPGATimestamp(),
        () -> setTarget(sequenceTarget(Timer.getFPGATimestamp() - stepStartTime)))
        .withName("Degrau");
  }

  @NotLogged
  public Command profiledSetpointCmd() {
    return startRun(this::resetProfile, () -> setProfiledGoal(testSetpointDeg.get()))
        .withName("Perfil setpoint");
  }

  @NotLogged
  public Command profiledStepCmd() {
    return startRun(
        () -> {
          resetProfile();
          stepStartTime = Timer.getFPGATimestamp();
        },
        () -> setProfiledGoal(sequenceTarget(Timer.getFPGATimestamp() - stepStartTime)))
        .withName("Perfil degrau");
  }

  /**
   * Perfil até o ângulo fornecido, reavaliado a cada ciclo (o alvo pode mudar enquanto roda).
   * Usado pelo controle: os botões mudam o alvo e o ponteiro vai até ele com perfil.
   */
  public Command profiledHoldCmd(DoubleSupplier deg) {
    return startRun(this::resetProfile, () -> setProfiledGoal(deg.getAsDouble()))
        .withName("Perfil (controle)");
  }

  /** Motor parado (nenhum comando de movimento rodando)? */
  public boolean isStopped() {
    return mode == Mode.STOPPED;
  }

  /** Chegou ao destino do perfil e está a menos de tolDeg dele? */
  public boolean isAtGoal(double tolDeg) {
    return tunerProfileDone() && Math.abs(wrap180(goalDeg - getDeg())) < tolDeg;
  }

  @NotLogged
  public Command velocityHoldCmd() {
    return run(() -> setVelocity(velTargetRpm.get())).withName("Velocidade");
  }

  @NotLogged
  public Command velocityStepCmd() {
    return startRun(
        () -> stepStartTime = Timer.getFPGATimestamp(),
        () -> {
          double t = Timer.getFPGATimestamp() - stepStartTime;
          boolean first = ((long) Math.floor(t / Math.max(0.2, stepPeriodSec.get()))) % 2 == 0;
          setVelocity(first ? velStepARpm.get() : velStepBRpm.get());
        })
        .withName("Degrau velocidade");
  }

  @NotLogged
  public Command openLoopTunableCmd() {
    return run(() -> setVolts(openLoopVolts.get())).withName("Malha aberta");
  }

  public Command manualCmd(DoubleSupplier axis) {
    return run(() -> setVolts(MathUtil.applyDeadband(axis.getAsDouble(), 0.08) * cfg.manualMaxVolts))
        .withName("Manual");
  }

  /** Calibra o zero: o ângulo atual vira ZeroRefDeg (270° = pendurado). Funciona desabilitado. */
  @NotLogged
  public Command zeroCmd() {
    return runOnce(this::calibrateAbsZero)
        .ignoringDisable(true)
        .withName("Calibrar zero");
  }

  // ===================== Loop =====================

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() && (oscTripped || oscDetector.chain() > 0)) {
      resetOscGuard();
    }
    if (oscReset.get() > 0.5) { // liberação remota (scripts de teste): volta ao nível 0, com o D
      oscReset.set(0.0);
      if (oscTripped) System.out.println("[Mecanismo] Proteção contra oscilação liberada (OscReset)");
      resetOscGuard();
    }
    ponteiro.setAngle(getDeg());
    double t = getTargetDeg();
    alvo.setLength(Double.isNaN(t) ? 0.0 : 1.1);
    alvo.setAngle(Double.isNaN(t) ? 0.0 : t);
    updateGainsIfChanged();
    applyIdleMode();
    updateContinuous();
  }
}
