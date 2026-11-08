package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.RobotBase;
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
 * kD agora é por SEGUNDO (PIDController da WPILib).
 *
 * <p>Calibração do zero (uma vez, ou quando mexer no encoder): ponteiro pendurado e parado →
 * botão Y (ou /Tuning/Dial/ZeroNow = 1). O código calcula o offset para o ângulo atual virar
 * ZeroRefDeg (270°) e imprime no Riolog o valor para copiar em {@link #kAbsZeroOffsetDeg}.
 *
 * <p>Modo PERFIL: tensão = kG·cos(θ) + VelkS·sinal(v) + VelkV·v + PID   (v = velocidade do perfil)
 *
 * <p>Modo VELOCIDADE (slot 1 do Spark): tensão = kS·sinal(alvo) + kV·alvo + PID.
 * A gravidade NÃO é compensada de propósito.
 */
@Logged
public class DialSubsystem extends SubsystemBase {

  // ===================== Hardware (CONFIRA) =====================
  private static final int kCanId = 2;

  /** Porta DIO da roboRIO com o fio de sinal (branco = PWM absoluto) do Through Bore. */
  private static final int kAbsDioPort = 0;

  /** Voltas do MOTOR para 1 volta do PONTEIRO (1.0 = ponteiro direto no eixo do NEO). */
  private static final double kGearRatio = 1.0;

  /** true se, com tensão positiva, o ponteiro girar no sentido HORÁRIO (precisa ser anti-horário). */
  private static final boolean kInverted = false;

  /** true se o ângulo do Through Bore DIMINUIR quando o ponteiro gira no sentido anti-horário. */
  private static final boolean kAbsInverted = false;

  /** Zero do Through Bore, em graus. Calibre com o botão Y e copie aqui o valor do Riolog. */
  private static final double kAbsZeroOffsetDeg = 223.89;

  private static final int kCurrentLimitA = 20;
  private static final double kMaxFFVolts = 4.0;
  private static final double kManualMaxVolts = 3.0;
  private static final double kNominalVolts = 12.0;

  /** Período do callback rápido de controle (registrado no Robot com addPeriodic). */
  public static final double kFastPeriodSec = 0.005;
  /** Se nenhum comando renovar o pedido nesse tempo, o callback rápido para o motor. */
  private static final double kRequestTimeoutSec = 0.1;

  /** Segurança: velocidade máxima do ponteiro aceita no modo velocidade. */
  private static final double kMaxVelRpm = 300.0;

  // ===================== Ganhos e testes (editar no Elastic: /Tuning/Dial/...) =====================
  @NotLogged
  private final TunableNumber kP = new TunableNumber("Dial/kP", 0.003);
  @NotLogged
  private final TunableNumber kI = new TunableNumber("Dial/kI", 0.004);
  @NotLogged
  private final TunableNumber kD = new TunableNumber("Dial/kD", 0.00012);
  @NotLogged
  private final TunableNumber iZone = new TunableNumber("Dial/IZoneDeg", 3.0);
  /**
   * Zona morta da integral: com |erro| menor que isto, a integral CONGELA (não cresce nem zera).
   * Sem ela, a integral e o atrito estático fazem o ponteiro "caçar" o alvo (±1–1,5° medido).
   * 0 = desliga (integral comum).
   */
  @NotLogged
  private final TunableNumber iDeadband = new TunableNumber("Dial/IDeadbandDeg", 0.5);
  /**
   * 1 = malha de posição no callback rápido (kFastPeriodSec, 5 ms). 0 = no ciclo normal de 20 ms,
   * como antes. Dá para trocar ao vivo e comparar.
   */
  @NotLogged
  private final TunableNumber fastLoop = new TunableNumber("Dial/FastLoop", 1.0);
  /** Filtro passa-baixa da derivada (constante de tempo, s). 0 = sem filtro. */
  @NotLogged
  private final TunableNumber dFilterSec = new TunableNumber("Dial/DFilterSec", 0.005);
  @NotLogged
  // kS medido = 0,091 V, mas no modo posição ele troca de sinal a cada cruzamento do alvo e faz
  // o ponteiro tremer (~8 Hz). O kI (só dentro do IZone) vence o atrito parado sem isso.
  private final TunableNumber kS = new TunableNumber("Dial/kS", 0.0);
  @NotLogged
  private final TunableNumber kG = new TunableNumber("Dial/kG", 0.362);
  @NotLogged
  private final TunableNumber outMax = new TunableNumber("Dial/OutMax", 0.2);

  @NotLogged
  private final TunableNumber testSetpointDeg = new TunableNumber("Dial/TestSetpointDeg", 0.0);
  @NotLogged
  private final TunableNumber stepADeg = new TunableNumber("Dial/StepADeg", 0.0);
  @NotLogged
  private final TunableNumber stepBDeg = new TunableNumber("Dial/StepBDeg", 45.0);
  @NotLogged
  private final TunableNumber stepCDeg = new TunableNumber("Dial/StepCDeg", 90.0);
  @NotLogged
  private final TunableNumber stepDDeg = new TunableNumber("Dial/StepDDeg", 135.0);
  @NotLogged
  private final TunableNumber stepEDeg = new TunableNumber("Dial/StepEDeg", 180.0);
  @NotLogged
  private final TunableNumber stepPeriodSec = new TunableNumber("Dial/StepPeriodSec", 1.0);
  @NotLogged
  private final TunableNumber openLoopVolts = new TunableNumber("Dial/OpenLoopVolts", 0.0);

  // ---- Velocidade (/Tuning/Dial/Vel...) — RPM do ponteiro ----
  @NotLogged
  private final TunableNumber velKP = new TunableNumber("Dial/VelkP", 0.0005);
  @NotLogged
  private final TunableNumber velKS = new TunableNumber("Dial/VelkS", 0.077);
  @NotLogged
  private final TunableNumber velKV = new TunableNumber("Dial/VelkV", 0.00247);
  /**
   * kA: volts por grau/s² de aceleração (SysId da bancada, 2026-11-07). Usado no FF do perfil:
   * V = kS·sinal(v) + kG·cos θ + kV·v + kA·a. 0 = desliga (compara com e sem).
   */
  @NotLogged
  private final TunableNumber kA = new TunableNumber("Dial/kA", 0.0000786);
  @NotLogged
  private final TunableNumber velOutMax = new TunableNumber("Dial/VelOutMax", 0.5);
  @NotLogged
  private final TunableNumber velTargetRpm = new TunableNumber("Dial/VelTargetRPM", 120.0);
  @NotLogged
  private final TunableNumber velStepARpm = new TunableNumber("Dial/VelStepARPM", 60.0);
  @NotLogged
  private final TunableNumber velStepBRpm = new TunableNumber("Dial/VelStepBRPM", 120.0);

  // ---- Perfil de movimento (/Tuning/Dial/Prof...) ----
  @NotLogged
  private final TunableNumber profMaxVelDegps = new TunableNumber("Dial/ProfMaxVelDegps", 240.0);
  @NotLogged
  private final TunableNumber profMaxAccelDegps2 = new TunableNumber("Dial/ProfMaxAccelDegps2", 1200.0);

  // ---- Encoder absoluto ----
  /** Ângulo que o botão Y grava. 270° = ponteiro pendurado para baixo, parado. */
  @NotLogged
  private final TunableNumber zeroRefDeg = new TunableNumber("Dial/ZeroRefDeg", 270.0);
  /** Zero do Through Bore (graus). Começa com kAbsZeroOffsetDeg; o botão Y atualiza. */
  @NotLogged
  private final TunableNumber absZeroOffsetDeg =
      new TunableNumber("Dial/AbsZeroOffsetDeg", kAbsZeroOffsetDeg);

  // ===================== Estado =====================
  private enum Mode { STOPPED, OPEN_LOOP, CLOSED_LOOP, PROFILED, VELOCITY, SYSID }

  @NotLogged
  private Mode mode = Mode.STOPPED;
  /** Setpoint de posição. Pode sair de 0–360 no modo perfil (o perfil é contínuo). */
  @NotLogged
  private double targetDeg = 0.0;
  @NotLogged
  private double targetRpm = 0.0;
  @NotLogged
  private double goalDeg = 0.0;
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
  /** O Spark está em BRAKE (só durante o SysId)? A simulação usa para saber se o motor freia. */
  @NotLogged
  private boolean brakeMode = false;

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

  // ===================== Hardware =====================
  @NotLogged
  private final SparkMax motor = new SparkMax(kCanId, MotorType.kBrushless);
  @NotLogged
  private final RelativeEncoder motorEncoder = motor.getEncoder();
  @NotLogged
  private final SparkClosedLoopController sparkPid = motor.getClosedLoopController();
  @NotLogged
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(kAbsDioPort);
  /** Física simulada (só no computador; no robô real é null). */
  @NotLogged
  private final DialSim sim =
      RobotBase.isSimulation() ? new DialSim(motor, absEncoder, kAbsInverted, kAbsZeroOffsetDeg) : null;

  // Desenho da bancada (Elastic / AdvantageScope / Glass: SmartDashboard/Dial/Mecanismo)
  @NotLogged
  private final Mechanism2d mech = new Mechanism2d(2.4, 2.4);
  @NotLogged
  private final MechanismLigament2d ponteiro;
  @NotLogged
  private final MechanismLigament2d alvo;

  public DialSubsystem() {
    System.out.println("[Dial] Firmware do Spark MAX: " + motor.getFirmwareString());

    // Through Bore V1: PWM de ~975,6 Hz, pulso de 1 a 1024 µs num período de 1025 µs
    absEncoder.setAssumedFrequency(975.6);
    absEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

    // Motor (essencial). Realimentação do Spark = encoder do NEO (padrão), usada só na velocidade.
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.inverted(kInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(kCurrentLimitA)
        .voltageCompensation(kNominalVolts);
    applyVelGains(cfg);
    configureOrThrow("motor", cfg, ResetMode.kResetSafeParameters);

    // Filtro de velocidade do encoder do NEO (opcional: se recusado, fica o padrão)
    SparkMaxConfig neo = new SparkMaxConfig();
    neo.encoder.uvwMeasurementPeriod(32).uvwAverageDepth(8);
    tryConfigure("filtro do NEO", neo);

    motorEncoder.setPosition(0.0);

    MechanismRoot2d centro = mech.getRoot("eixo", 1.2, 1.2);
    alvo = centro.append(new MechanismLigament2d("alvo", 1.0, 0.0, 2, new Color8Bit(Color.kGray)));
    ponteiro = centro.append(
        new MechanismLigament2d("ponteiro", 1.0, 270.0, 6, new Color8Bit(Color.kOrange)));
    SmartDashboard.putData("Dial/Mecanismo", mech);
  }

  /** Slot 1 = velocidade, em RPM do motor (= ponteiro com redução 1:1). */
  private void applyVelGains(SparkMaxConfig cfg) {
    cfg.closedLoop
        .pid(velKP.get(), 0.0, 0.0, ClosedLoopSlot.kSlot1)
        .outputRange(-velOutMax.get(), velOutMax.get(), ClosedLoopSlot.kSlot1);
  }

  private void configureOrThrow(String name, SparkMaxConfig c, ResetMode reset) {
    motor.configure(c, reset, PersistMode.kNoPersistParameters); // lança exceção se recusado
    System.out.println("[Dial] configure " + name + ": ok");
  }

  private void tryConfigure(String name, SparkMaxConfig c) {
    try {
      motor.configure(c, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      System.out.println("[Dial] configure " + name + ": ok");
    } catch (RuntimeException e) {
      System.out.println("[Dial] configure " + name + " RECUSADO (usando padrão): " + e.getMessage());
    }
  }

  // ===================== Leituras =====================

  /** Leitura bruta do Through Bore, 0 a 1 (sem inversão nem offset). */
  public double getAbsRaw() {
    return absEncoder.get();
  }

  public boolean isAbsConnected() {
    return absEncoder.isConnected();
  }

  /** Frequência do PWM do Through Bore (~975 Hz quando conectado). */
  public double getAbsFrequencyHz() {
    return absEncoder.getFrequency();
  }

  private double signedRawDeg() {
    double deg = absEncoder.get() * 360.0;
    return kAbsInverted ? -deg : deg;
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
    return motorEncoder.getPosition() * 360.0 / kGearRatio;
  }

  /** Velocidade do ponteiro em RPM (derivada do Through Bore, média de 5 ciclos). */
  public double getVelRpm() {
    return velRpm;
  }

  /** Velocidade pelo encoder do NEO (é a que o PID de velocidade do Spark usa). */
  public double getMotorVelRpm() {
    return motorEncoder.getVelocity() / kGearRatio;
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
    return isPositionMode() ? wrap180(targetDeg - getDeg()) : 0.0;
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
    return motor.getAppliedOutput() * motor.getBusVoltage();
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

  /** Parte do kI dentro de PidVolts. */
  public double getIntegralVolts() {
    return integralVolts;
  }

  public double getPidVolts() {
    return pidVolts;
  }

  public double getCurrentA() {
    return motor.getOutputCurrent();
  }

  // ===================== Utilidades de ângulo =====================

  private static double wrap360(double deg) {
    return MathUtil.inputModulus(deg, 0.0, 360.0);
  }

  private static double wrap180(double deg) {
    return MathUtil.inputModulus(deg, -180.0, 180.0);
  }

  // ===================== Controle =====================

  private void stop() {
    mode = Mode.STOPPED;
    commandedVolts = 0.0;
    ffVolts = 0.0;
    pidVolts = 0.0;
    integralVolts = 0.0;
    motor.stopMotor();
  }

  private void setVolts(double volts) {
    mode = Mode.OPEN_LOOP;
    commandedVolts = MathUtil.clamp(volts, -kNominalVolts, kNominalVolts);
    ffVolts = 0.0;
    pidVolts = 0.0;
    motor.setVoltage(commandedVolts);
  }

  /** Sem sensor, nada de malha fechada de posição. */
  private boolean sensorOk() {
    if (absEncoder.isConnected()) {
      warnedDisconnected = false;
      return true;
    }
    if (!warnedDisconnected) {
      System.out.println("[Dial] Through Bore DESCONECTADO (DIO " + kAbsDioPort + "): motor parado");
      warnedDisconnected = true;
    }
    stop();
    return false;
  }

  private void enterPositionMode(Mode m) {
    if (!isPositionMode()) {
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
    if (!sensorOk()) return;
    enterPositionMode(Mode.CLOSED_LOOP);
    targetDeg = wrap360(deg);
    requestTime = Timer.getFPGATimestamp();
    if (!useFastLoop()) controlStep(0.02);
  }

  /** Pedido de posição COM perfil (o perfil avança dentro de controlStep). */
  private void setProfiledGoal(double deg) {
    if (!sensorOk()) return;
    enterPositionMode(Mode.PROFILED);
    goalRequestDeg = wrap360(deg);
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
    if (sim != null) {
      // física primeiro; o controle lê os sensores novos. Motor parado em COAST = solto (sem freio)
      sim.step(MathUtil.clamp(dt, 0.001, 0.03), mode == Mode.STOPPED && !brakeMode);
    }
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
    controlStep(MathUtil.clamp(dt, 0.001, 0.03));
  }

  /** Um passo da malha de posição: avança o perfil (se houver), FF + PID → tensão. */
  private void controlStep(double dt) {
    double meas = getDeg();
    double gravity = kG.get() * Math.cos(Math.toRadians(meas));
    double ff;
    boolean allowIntegral;

    if (mode == Mode.PROFILED) {
      // O perfil é contínuo: escolhe a cópia do destino (±360°) mais perto de onde ele está
      goalDeg = profState.position + wrap180(goalRequestDeg - profState.position);
      TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
          Math.max(1.0, profMaxVelDegps.get()), Math.max(1.0, profMaxAccelDegps2.get())));
      double prevVel = profState.velocity;
      profState = profile.calculate(dt, profState, new TrapezoidProfile.State(goalDeg, 0.0));
      profAccel = (profState.velocity - prevVel) / dt;
      targetDeg = profState.position;

      double velDegps = profState.velocity;
      double error = wrap180(targetDeg - meas);
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
      double error = wrap180(targetDeg - meas);
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
    ffVolts = MathUtil.clamp(ff, -kMaxFFVolts, kMaxFFVolts);
    double error = wrap180(wrap360(targetDeg) - meas);

    // Derivada do erro, com passa-baixa (cada passo do encoder vira um pico em 5 ms)
    if (firstStep) {
      prevError = error;
      errDeriv = 0.0;
      firstStep = false;
    }
    double rawDeriv = wrap180(error - prevError) / dt;
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

    double out = MathUtil.clamp(kP.get() * error + kD.get() * errDeriv + iOut,
        -outMax.get(), outMax.get());
    pidVolts = out * kNominalVolts;
    commandedVolts = MathUtil.clamp(ffVolts + pidVolts, -kNominalVolts, kNominalVolts);
    motor.setVoltage(commandedVolts);
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

    commandedVolts = sysIdVolts;
    motor.setVoltage(sysIdVolts);
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
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.idleMode(IdleMode.kBrake);
    brakeMode = true;
    tryConfigure("brake (SysId)", cfg);
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

  /** Termina a sessão: motor parado, volta para COAST e grava o CSV. */
  public void sysIdEnd(String path) {
    stop();
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.idleMode(IdleMode.kCoast);
    tryConfigure("coast", cfg);
    brakeMode = false;
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
    mode = Mode.VELOCITY;
    targetRpm = MathUtil.clamp(rpm, -kMaxVelRpm, kMaxVelRpm);
    pidVolts = 0.0;

    ffVolts = Math.abs(targetRpm) < 1e-3
        ? 0.0
        : velKS.get() * Math.signum(targetRpm) + velKV.get() * targetRpm;
    ffVolts = MathUtil.clamp(ffVolts, -kNominalVolts, kNominalVolts);
    commandedVolts = ffVolts;

    sparkPid.setSetpoint(targetRpm * kGearRatio, ControlType.kVelocity, ClosedLoopSlot.kSlot1,
        ffVolts, SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  private void updateGainsIfChanged() {
    // kP/kI/kD/IZone são lidos direto a cada passo da malha (na RIO: barato)
    if (velKP.hasChanged() | velOutMax.hasChanged()) {
      SparkMaxConfig cfg = new SparkMaxConfig();
      applyVelGains(cfg);
      motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    if (absZeroOffsetDeg.hasChanged()) {
      contInit = false; // o ângulo saltou: recomeça o ângulo contínuo
    }
  }

  /** Atualiza o ângulo contínuo e a velocidade a partir do Through Bore. */
  private void updateContinuous() {
    double now = Timer.getFPGATimestamp();
    double wrapped = getDeg();
    if (!contInit || !absEncoder.isConnected()) {
      contInit = absEncoder.isConnected();
      lastWrappedDeg = wrapped;
      lastTime = now;
      contDeg = wrapped;
      velFilter.reset();
      velRpm = 0.0;
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
  }

  /** Calcula o offset para que o ângulo ATUAL passe a valer ZeroRefDeg. */
  private void calibrateAbsZero() {
    if (!absEncoder.isConnected()) {
      System.out.println("[Dial] Zero NÃO calibrado: Through Bore desconectado");
      return;
    }
    double offset = wrap360(signedRawDeg() - zeroRefDeg.get());
    absZeroOffsetDeg.set(offset);
    contInit = false;
    System.out.printf(
        "[Dial] Zero calibrado: ponteiro = %.1f°. Copie para DialSubsystem.java: "
            + "kAbsZeroOffsetDeg = %.2f;%n", zeroRefDeg.get(), offset);
  }

  // ===================== API do autoajuste (usada pelo DialAutoTuner) =====================

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
    return absEncoder.isConnected() && Math.abs(wrap180(getDeg() - zeroRefDeg.get())) <= 15.0;
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
    return run(() -> setVolts(MathUtil.applyDeadband(axis.getAsDouble(), 0.08) * kManualMaxVolts))
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
    ponteiro.setAngle(getDeg());
    double t = getTargetDeg();
    alvo.setLength(Double.isNaN(t) ? 0.0 : 1.1);
    alvo.setAngle(Double.isNaN(t) ? 0.0 : t);
    updateGainsIfChanged();
    updateContinuous();
  }
}
