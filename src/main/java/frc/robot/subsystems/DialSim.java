package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Física da bancada para a simulação (só existe quando o código roda no computador).
 *
 * <p>Todos os números vêm de MEDIÇÕES na bancada real, não de chute:
 *   - kG = 0,362 V (autoajuste) e kS = 0,077 V, kV = 0,00247 V/RPM (SysId, 2026-11-07)
 *   - inércia e atritos do eixo: ajustados para reproduzir o PÊNDULO real (ponteiro solto em 0°,
 *     2026-11-09). Viradas reais 191°, 337°, 210°, 320°, 226°, 305°, 241°, 292°; o modelo dá
 *     192°, 338°, 211°, 321°, 227°, 306°, 240°, 294° (erro de 1–2° em cada).
 *
 * <p>Daí:
 *   - torque por volt do NEO parado: 2,6 N·m / 12 V = 0,217 N·m/V
 *   - inércia J = 0,000722 kg·m² pelo pêndulo — confere com o kA do SysId pelo método direto
 *     (0,0000565 V/(°/s²) → J ≈ 0,00070)
 *   - o SingleJointedArmSim modela a gravidade como uma barra uniforme: aceleração =
 *     1,5·g·cos θ / L. Para bater com o kG, L = 1,5·g·J / (kG·0,217) ≈ 0,144 m.
 *     (Não é o comprimento real do ponteiro: é o "comprimento equivalente" que dá o mesmo torque.)
 *   - motor: NEO com velocidade livre de 12 V / kV ≈ 4858 RPM
 *   - perdas: kS (0,077 V) só com o motor acionado; solto, atrito do eixo 0,015 V + viscoso
 *     0,0001 V/RPM
 *   - motor solto (COAST) não freia; só no BRAKE (SysId) o motor parado freia
 *   - atraso de 10 ms entre o controle e o motor (CAN + Spark + leitura do sensor): sem ele a
 *     simulação quase não passava do alvo, e o real passa ~2°
 *   - Through Bore: leitura quantizada em 1024 passos (0,35°), como o sensor real
 */
class DialSim {

  private static final double kTorquePerVolt = 2.6 / 12.0;
  private static final double kJ = 0.000722;
  private static final double kGVolts = 0.362;
  private static final double kEquivLengthM = 1.5 * 9.81 * kJ / (kGVolts * kTorquePerVolt);
  private static final double kSVolts = 0.077;
  /** Atrito para começar a mover parado (V). Solto, o ponteiro real parava a ±1° de 270°. */
  private static final double kStaticVolts = 0.01;
  /** Atrito mecânico em movimento com o motor solto (V equivalentes). */
  private static final double kCoastFrictionVolts = 0.015;
  /** Atrito viscoso com o motor solto (V por RPM), pelo pêndulo real. Acionado, já está no kV. */
  private static final double kViscousVoltsPerRpm = 0.0001;
  /** Atraso entre o controle mandar a tensão e o motor aplicar (s). */
  private static final double kActuatorDelaySec = 0.010;
  private static final double kFreeSpeedRpm = 12.0 / 0.00247;

  private final DCMotor neo = new DCMotor(
      12.0, 2.6, 105.0, 1.8, Units.rotationsPerMinuteToRadiansPerSecond(kFreeSpeedRpm), 1);
  private final SingleJointedArmSim arm;
  private final SparkMaxSim spark;
  private final DutyCycleEncoderSim absSim;
  private final boolean absInverted;
  private final double absOffsetDeg;
  /** Fila de tensões pedidas (V) para aplicar com atraso; o tamanho vem de kActuatorDelaySec. */
  private final java.util.ArrayDeque<Double> delayLine = new java.util.ArrayDeque<>();

  DialSim(SparkMax motor, DutyCycleEncoder abs, boolean absInverted, double absOffsetDeg) {
    // Sem limites de curso: o ponteiro dá voltas completas. Começa pendurado (270° = −90°).
    arm = new SingleJointedArmSim(neo, 1.0, kJ, kEquivLengthM, -1e6, 1e6, true, -Math.PI / 2);
    spark = new SparkMaxSim(motor, neo);
    absSim = new DutyCycleEncoderSim(abs);
    absSim.setConnected(true);
    this.absInverted = absInverted;
    this.absOffsetDeg = absOffsetDeg;
    writeSensors();
  }

  /**
   * Avança a física dt segundos.
   *
   * @param coasting motor parado em COAST: os fios ficam "abertos" e o motor não freia. Sem isso
   *     o simulador trata 0 V como fios em curto (= BRAKE) e o ponteiro solto cai "no mel".
   */
  void step(double dt, boolean coasting) {
    double omega = arm.getVelocityRadPerSec();
    double rpm = Units.radiansPerSecondToRotationsPerMinute(omega);
    spark.iterate(rpm, 12.0, dt);

    double volts;
    if (coasting) {
      // Tensão que zera a corrente: só a força contraeletromotriz, o motor não freia
      volts = omega / neo.KvRadPerSecPerVolt - kViscousVoltsPerRpm * rpm;
    } else {
      delayLine.addLast(spark.getAppliedOutput() * 12.0);
      int atraso = (int) Math.round(kActuatorDelaySec / dt);
      while (delayLine.size() > atraso + 1) {
        delayLine.removeFirst();
      }
      volts = delayLine.peekFirst();
    }

    // Perdas contra o movimento. O kS medido (0,077 V) NÃO é só atrito do eixo: se fosse, o
    // ponteiro solto pararia em qualquer lugar até 12° de 270°, e o real sempre parou a ±1°.
    // A maior parte é perda do acionamento (zona morta do Spark, comutação do NEO), que só
    // existe com o motor acionado. Solto, sobra só o atrito mecânico pequeno.
    double friction = coasting ? kCoastFrictionVolts : kSVolts;
    double theta = arm.getAngleRads();
    double gravityVolts = kGVolts * Math.cos(theta);
    double netVolts = volts - gravityVolts;
    boolean stuck = false;
    if (Math.abs(omega) > 0.05) {
      volts -= friction * Math.signum(omega);
    } else if (Math.abs(netVolts) <= kStaticVolts) {
      volts = gravityVolts; // equilibra a gravidade: fica parado
      stuck = true;
    } else {
      volts -= friction * Math.signum(netVolts);
    }

    arm.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    arm.update(dt);
    if (stuck) {
      arm.setState(theta, 0.0);
    }
    writeSensors();
  }

  private void writeSensors() {
    double deg = Math.toDegrees(arm.getAngleRads());
    // getDeg() do subsistema = (±bruto·360 − offset): aqui é o inverso
    double rawDeg = (absInverted ? -1.0 : 1.0) * (deg + absOffsetDeg);
    double raw = MathUtil.inputModulus(rawDeg / 360.0, 0.0, 1.0);
    raw = Math.floor(raw * 1024.0) / 1024.0; // resolução real do Through Bore
    absSim.set(raw);
  }

  /** Ângulo verdadeiro da simulação (sem quantização), em graus 0–360. */
  double getTrueDeg() {
    return MathUtil.inputModulus(Math.toDegrees(arm.getAngleRads()), 0.0, 360.0);
  }
}
