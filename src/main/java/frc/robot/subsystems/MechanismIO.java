package frc.robot.subsystems;

/**
 * Camada de hardware (IO) do mecanismo da bancada: TUDO o que fala com motor e sensores passa por
 * aqui. O {@link MechanismSubsystem} (controle, perfil, testes) só enxerga esta interface, então trocar o
 * hardware (Kraken/TalonFX, encoder ligado no controlador, outro mecanismo) é escrever outra
 * implementação, sem mexer no controle.
 *
 * <p>Mesmo formato do AdvantageKit (interface + {@link Inputs} lidos uma vez por ciclo), para
 * migrar para ele no futuro se o replay fizer falta.
 *
 * <p>Unidades: ângulos do MECANISMO em graus, motor em rotações/RPM do eixo do MOTOR (antes da
 * redução), tensões em volts.
 */
public interface MechanismIO {

  /** Tudo o que o hardware mede, lido de uma vez no início de cada ciclo de 5 ms. */
  class Inputs {
    /** Sensor absoluto (Through Bore): conectado? */
    public boolean absConnected = false;
    /** Leitura bruta do absoluto, 0 a 1 (sem inversão nem zero). */
    public double absRaw = 0.0;
    /** Frequência do PWM do absoluto (~975 Hz no Through Bore V1). */
    public double absFreqHz = 0.0;
    /** Quadratura no eixo do mecanismo: ângulo contínuo (graus; só diferenças importam). */
    public double quadDeg = 0.0;
    /** Velocidade pela quadratura (graus/s; repete o último valor parado: não usar no kD). */
    public double quadRateDegps = 0.0;
    /** Encoder do motor: rotações e RPM do eixo do MOTOR. */
    public double motorRot = 0.0;
    public double motorRpm = 0.0;
    public double appliedVolts = 0.0;
    public double busVolts = 0.0;
    public double currentA = 0.0;
    public double tempC = 0.0;
  }

  /** Lê todos os sensores para {@code in}. */
  void updateInputs(Inputs in);

  /** Tensão no motor (positiva = mecanismo no sentido positivo). */
  void setVoltage(double volts);

  /** Motor desligado (segue o modo coast/brake configurado). */
  void stop();

  /** Malha de velocidade no controlador do motor: {@code motorRpm} no eixo do motor + FF em volts. */
  void setVelocity(double motorRpm, double ffVolts);

  /** Ganhos da malha de velocidade do controlador ({@code outMax} em fração de 0 a 1). */
  void configureVelocityGains(double kP, double outMax);

  /** true = BRAKE (motor freia parado), false = COAST. */
  void setBrake(boolean brake);

  /** Limite de corrente do controlador (A). */
  void setCurrentLimit(int amps);

  /** Só na simulação: avança a física {@code dt} segundos. {@code coasting} = motor solto. */
  default void simulationStep(double dt, boolean coasting) {}

  /** Texto curto do hardware, para o Riolog. */
  default String description() {
    return getClass().getSimpleName();
  }
}
