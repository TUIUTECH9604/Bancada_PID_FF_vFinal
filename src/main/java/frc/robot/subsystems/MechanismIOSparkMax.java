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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Hardware da bancada (2026-11): NEO + Spark MAX (CAN 2) com redução, Through Bore V1 no eixo do
 * mecanismo ligado na roboRIO — absoluto por PWM no DIO 0, quadratura A/B nos DIO 1 e 2.
 * Na simulação, a física vem do {@link MechanismSim} (mesmos objetos, com os simuladores da REV/WPILib).
 */
public class MechanismIOSparkMax implements MechanismIO {

  // ---- Fiação (CONFIRA ao mudar o hardware) ----
  private static final int kCanId = 2;
  /** DIO do sinal absoluto (PWM, fio branco) do Through Bore. */
  private static final int kAbsDioPort = 0;
  /** Quadratura do MESMO Through Bore: 2048 ciclos/volta = 8192 contagens (0,044°). */
  private static final int kQuadDioA = 1;
  private static final int kQuadDioB = 2;
  /** true se a quadratura contar ao contrário do absoluto (conferido à mão). */
  private static final boolean kQuadInverted = true;
  /** true se, com tensão positiva, o mecanismo girar no sentido negativo. */
  private static final boolean kInverted = false;
  private static final double kNominalVolts = 12.0;

  private final SparkMax motor = new SparkMax(kCanId, MotorType.kBrushless);
  private final RelativeEncoder motorEncoder = motor.getEncoder();
  private final SparkClosedLoopController sparkPid = motor.getClosedLoopController();
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(kAbsDioPort);
  private final Encoder quadEncoder =
      new Encoder(kQuadDioA, kQuadDioB, kQuadInverted, Encoder.EncodingType.k4X);
  private final MechanismSim sim;

  /**
   * @param currentLimitA limite de corrente do Spark
   * @param controlFramePeriodMs período do quadro de controle (a tensão pedida). A REVLib manda a
   *     cada 20 ms por padrão: com a malha a 5 ms precisa ser 5 (foi a causa do "tremor").
   * @param velKP ganho da malha de velocidade (slot 1) e {@code velOutMax} a saída máxima dela
   * @param absInverted e {@code absZeroOffsetDeg}: só para a simulação gerar a leitura bruta certa
   */
  public MechanismIOSparkMax(int currentLimitA, int controlFramePeriodMs, double velKP,
      double velOutMax, boolean absInverted, double absZeroOffsetDeg) {
    System.out.println("[MechanismIO] Firmware do Spark MAX: " + motor.getFirmwareString());

    // Through Bore V1: PWM de ~975,6 Hz, pulso de 1 a 1024 µs num período de 1025 µs
    absEncoder.setAssumedFrequency(975.6);
    absEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    // 4X: getDistance/getRate em graus e graus/s (2048 ciclos por volta)
    quadEncoder.setDistancePerPulse(360.0 / 2048.0);

    // Motor (essencial). Realimentação do Spark = encoder do NEO (padrão), usada só na velocidade.
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.inverted(kInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(currentLimitA)
        .voltageCompensation(kNominalVolts);
    applyVelGains(cfg, velKP, velOutMax);
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    System.out.println("[MechanismIO] configure motor: ok");

    // Filtro de velocidade do encoder do NEO (opcional: se recusado, fica o padrão)
    SparkMaxConfig neo = new SparkMaxConfig();
    neo.encoder.uvwMeasurementPeriod(32).uvwAverageDepth(8);
    tryConfigure("filtro do NEO", neo);

    motorEncoder.setPosition(0.0);
    motor.setControlFramePeriodMs(controlFramePeriodMs);

    sim = RobotBase.isSimulation()
        ? new MechanismSim(motor, absEncoder, quadEncoder, absInverted, absZeroOffsetDeg) : null;
  }

  @Override
  public void updateInputs(Inputs in) {
    in.absConnected = absEncoder.isConnected();
    in.absRaw = absEncoder.get();
    in.absFreqHz = absEncoder.getFrequency();
    in.quadDeg = quadEncoder.getDistance();
    in.quadRateDegps = quadEncoder.getRate();
    in.motorRot = motorEncoder.getPosition();
    in.motorRpm = motorEncoder.getVelocity();
    in.busVolts = motor.getBusVoltage();
    in.appliedVolts = motor.getAppliedOutput() * in.busVolts;
    in.currentA = motor.getOutputCurrent();
    in.tempC = motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setVelocity(double motorRpm, double ffVolts) {
    sparkPid.setSetpoint(motorRpm, ControlType.kVelocity, ClosedLoopSlot.kSlot1, ffVolts,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void configureVelocityGains(double kP, double outMax) {
    SparkMaxConfig cfg = new SparkMaxConfig();
    applyVelGains(cfg, kP, outMax);
    motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setBrake(boolean brake) {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    tryConfigure(brake ? "brake" : "coast", cfg);
  }

  @Override
  public void setCurrentLimit(int amps) {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.smartCurrentLimit(amps);
    tryConfigure("limite de corrente " + amps + " A", cfg);
  }

  @Override
  public void simulationStep(double dt, boolean coasting) {
    if (sim != null) sim.step(dt, coasting);
  }

  @Override
  public String description() {
    return "Spark MAX CAN " + kCanId + " + Through Bore (abs DIO " + kAbsDioPort + ", quad DIO "
        + kQuadDioA + "/" + kQuadDioB + ")";
  }

  /** Slot 1 = velocidade, em RPM do motor. */
  private static void applyVelGains(SparkMaxConfig cfg, double kP, double outMax) {
    cfg.closedLoop
        .pid(kP, 0.0, 0.0, ClosedLoopSlot.kSlot1)
        .outputRange(-outMax, outMax, ClosedLoopSlot.kSlot1);
  }

  private void tryConfigure(String name, SparkMaxConfig c) {
    try {
      motor.configure(c, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      System.out.println("[MechanismIO] configure " + name + ": ok");
    } catch (RuntimeException e) {
      System.out.println("[MechanismIO] configure " + name + " RECUSADO (usando padrão): "
          + e.getMessage());
    }
  }
}
