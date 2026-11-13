package frc.robot.subsystems;

/**
 * TUDO o que muda de um mecanismo para outro, num lugar só. Para testar um mecanismo novo na
 * bancada: escreva uma configuração (copie {@link #BANCADA} e ajuste) e uma implementação de
 * {@link MechanismIO} para o hardware dele. O controle, o perfil e os testes não mudam.
 *
 * <p>Os valores de ganho são só os PADRÕES: todos continuam ajustáveis ao vivo em
 * /Tuning/{name}/... (e voltam a estes ao reiniciar o robô).
 *
 * <p>Unidades: graus do MECANISMO (depois da redução) e volts. Mecanismos lineares (elevador)
 * ainda não são suportados: precisariam de unidades em metros (próxima etapa).
 */
public final class MechanismConfig {

  /** Como a gravidade age no mecanismo (termo kG do feedforward). */
  public enum Gravity {
    /** Sem gravidade: giratório na horizontal, torre, roda. */
    NONE,
    /** Braço/ponteiro: kG·cos(θ), com θ = 0° na horizontal. */
    COSINE,
    /** Sempre a mesma carga (ex.: elevador, ou braço com mola que compensa só parte). */
    CONSTANT;

    /** Fator que multiplica o kG no ângulo dado (graus). */
    public double factor(double deg) {
      return switch (this) {
        case NONE -> 0.0;
        case COSINE -> Math.cos(Math.toRadians(deg));
        case CONSTANT -> 1.0;
      };
    }
  }

  // ---- Identidade ----
  /** Nome curto: pasta no NetworkTables (/Tuning/{name}/...) e nos logs. */
  public final String name;

  // ---- Mecânica e sensor ----
  /** Voltas do MOTOR por volta do MECANISMO (15 = redução 15:1). */
  public final double gearRatio;
  public final Gravity gravity;
  /** true se o ângulo do sensor absoluto DIMINUI quando o mecanismo gira no sentido positivo. */
  public final boolean absInverted;
  /** Zero do sensor absoluto (graus): calibre com o botão Y / ZeroNow e copie do Riolog. */
  public final double absZeroOffsetDeg;
  /** Ângulo que a calibração do zero grava (posição de repouso do mecanismo). */
  public final double zeroRefDeg;
  /** Quadratura descartada se discordar do absoluto mais que isto (graus). */
  public final double quadMaxDriftDeg;

  // ---- Segurança ----
  public final int currentLimitA;
  /** Teto do feedforward (V): ele sozinho nunca passa disso. */
  public final double maxFFVolts;
  /** Tensão máxima no modo manual (LB + analógico). */
  public final double manualMaxVolts;
  /** Velocidade máxima aceita no modo velocidade (RPM do mecanismo). */
  public final double maxVelRpm;
  /**
   * Motor em BRAKE quando parado (braço, elevador: não cai sozinho) ou COAST (shooter, rolo).
   * Com BRAKE, o botão X / {@code release()} solta (COAST) até o próximo movimento.
   */
  public final boolean brakeWhenStopped;
  /** Limites de curso: padrão ao ligar (ver MechanismSubsystem.setLimits). */
  public final boolean limitsOn;
  public final double limitLowDeg;
  public final double limitHighDeg;
  public final double limitMarginDeg;
  /**
   * Anti-folga: com o alvo onde a gravidade é fraca (|fator| < antiBacklashFactorMax, ex.: perto da
   * vertical) e o movimento chegando CONTRA ela, o perfil passa antiBacklashDeg do alvo e volta a
   * favor da gravidade. Assim o motor freia e segura pelo mesmo lado do dente (sem atravessar a
   * folga no fim). Desligado = false. Ajustável ao vivo (/Tuning/{nome}/AntiFolga...).
   */
  public final boolean antiBacklashOn;
  /**
   * Fator do D com o mecanismo PARADO perto do alvo (perfil terminado e |erro| < IZone): 1 = D
   * normal; menor = menos D parado (o D reage ao mecanismo solto na folga e pode oscilar).
   */
  public final double dHoldFactor;
  public final double antiBacklashDeg;
  public final double antiBacklashFactorMax;

  // ---- Ganhos padrão: posição (fração de 12 V por grau, como sempre) ----
  public final double kP;
  public final double kI;
  public final double kD;
  public final double iZoneDeg;
  public final double iDeadbandDeg;
  public final double dFilterSec;
  public final double outMax;

  // ---- Feedforward padrão ----
  public final double kS;
  public final double kG;
  /** V por RPM do mecanismo. */
  public final double kV;
  /** V por °/s². */
  public final double kA;
  public final double velKS;

  // ---- Velocidade (malha no controlador do motor) ----
  public final double velKP;
  public final double velOutMax;

  // ---- Perfil padrão ----
  public final double profMaxVelDegps;
  public final double profMaxAccelDegps2;

  /**
   * A bancada da TUIUTECH em 2026-11-11: NEO + Spark MAX com redução 15:1 (3:1 × 5:1 + 90°),
   * ponteiro de ~620 g, Through Bore na roboRIO. Ganhos validados no robô (ver CLAUDE.md).
   */
  public static final MechanismConfig BANCADA = builder("Dial")
      .gearRatio(15.0)
      .gravity(Gravity.COSINE)
      .absInverted(false)
      .absZeroOffsetDeg(301.34) // 2026-11-12: ponteiro remontado (+parafusos); era 270,91
      .zeroRefDeg(270.0)
      .quadMaxDriftDeg(5.0)
      .currentLimitA(20)
      .maxFFVolts(4.0)
      .manualMaxVolts(3.0)
      .maxVelRpm(150.0)
      // 2026-11-12: em COAST o ponteiro escorregou de 0° a 330° sozinho (a caixa não segura na
      // horizontal)
      .brakeWhenStopped(true)
      .limits(false, 0.0, 0.0, 10.0)
      // 2026-11-12: folga ~5,7°; perto de 90° o ponteiro atravessava a folga e passava 3-5° do
      // alvo. Desligado até validar no robô (ligar ao vivo: /Tuning/Dial/AntiFolga = 1)
      .antiBacklash(false, 8.0, 0.7)
      // D × 0,3 parado no alvo (2026-11-12): com D cheio, 9 de 10 paradas em 270° viravam
      // ciclo-limite de 12,5 Hz na folga; com 0,3, 0 de 10 e as chegadas iguais (bat_20261112dp)
      .dHoldFactor(0.3)
      // kD 0,00015 (2026-11-12; era 0,0003): com o ponteiro mais pesado o D batia junto com o
      // ponteiro na folga durante o movimento (~10 Hz); pela metade, grandes 0,9–1,0° em vez de
      // 1,9–2,7° (bat_20261112kd, log a 5 ms)
      .pid(0.006, 0.02, 0.00015)
      .integral(3.0, 0.5)
      .dFilterSec(0.005)
      .outMax(0.2)
      // kG 0,09 (2026-11-12, ponteiro com mais parafusos; era 0,063). Medidas: rampas 0,073,
      // segurando em 0°/180° 0,123. Bateria bat_20261112kg + robusto do código: 0,123 passava
      // 3,4-4,6° nos passos de subida perto de 90° (cresce com o kG); 0,09 = meio-termo
      .feedforward(0.0, 0.09, 0.0318, 0.00005)
      .velKS(0.0)
      .velocityLoop(0.0005, 0.5)
      .profile(720.0, 4000.0)
      .build();

  public static Builder builder(String name) {
    return new Builder(name);
  }

  /** Cópia para alterar alguns valores: {@code BANCADA.toBuilder().gravity(NONE).build()}. */
  public Builder toBuilder() {
    return toBuilder(name);
  }

  /** Cópia com outro nome (outra pasta /Tuning/{nome}/): ponto de partida de um mecanismo novo. */
  public Builder toBuilder(String newName) {
    Builder b = new Builder(newName);
    b.gearRatio = gearRatio;
    b.gravity = gravity;
    b.absInverted = absInverted;
    b.absZeroOffsetDeg = absZeroOffsetDeg;
    b.zeroRefDeg = zeroRefDeg;
    b.quadMaxDriftDeg = quadMaxDriftDeg;
    b.currentLimitA = currentLimitA;
    b.maxFFVolts = maxFFVolts;
    b.manualMaxVolts = manualMaxVolts;
    b.maxVelRpm = maxVelRpm;
    b.brakeWhenStopped = brakeWhenStopped;
    b.limitsOn = limitsOn;
    b.limitLowDeg = limitLowDeg;
    b.limitHighDeg = limitHighDeg;
    b.limitMarginDeg = limitMarginDeg;
    b.antiBacklashOn = antiBacklashOn;
    b.dHoldFactor = dHoldFactor;
    b.antiBacklashDeg = antiBacklashDeg;
    b.antiBacklashFactorMax = antiBacklashFactorMax;
    b.kP = kP;
    b.kI = kI;
    b.kD = kD;
    b.iZoneDeg = iZoneDeg;
    b.iDeadbandDeg = iDeadbandDeg;
    b.dFilterSec = dFilterSec;
    b.outMax = outMax;
    b.kS = kS;
    b.kG = kG;
    b.kV = kV;
    b.kA = kA;
    b.velKS = velKS;
    b.velKP = velKP;
    b.velOutMax = velOutMax;
    b.profMaxVelDegps = profMaxVelDegps;
    b.profMaxAccelDegps2 = profMaxAccelDegps2;
    return b;
  }

  private MechanismConfig(Builder b) {
    name = b.name;
    gearRatio = b.gearRatio;
    gravity = b.gravity;
    absInverted = b.absInverted;
    absZeroOffsetDeg = b.absZeroOffsetDeg;
    zeroRefDeg = b.zeroRefDeg;
    quadMaxDriftDeg = b.quadMaxDriftDeg;
    currentLimitA = b.currentLimitA;
    maxFFVolts = b.maxFFVolts;
    manualMaxVolts = b.manualMaxVolts;
    maxVelRpm = b.maxVelRpm;
    brakeWhenStopped = b.brakeWhenStopped;
    limitsOn = b.limitsOn;
    limitLowDeg = b.limitLowDeg;
    limitHighDeg = b.limitHighDeg;
    limitMarginDeg = b.limitMarginDeg;
    antiBacklashOn = b.antiBacklashOn;
    dHoldFactor = b.dHoldFactor;
    antiBacklashDeg = b.antiBacklashDeg;
    antiBacklashFactorMax = b.antiBacklashFactorMax;
    kP = b.kP;
    kI = b.kI;
    kD = b.kD;
    iZoneDeg = b.iZoneDeg;
    iDeadbandDeg = b.iDeadbandDeg;
    dFilterSec = b.dFilterSec;
    outMax = b.outMax;
    kS = b.kS;
    kG = b.kG;
    kV = b.kV;
    kA = b.kA;
    velKS = b.velKS;
    velKP = b.velKP;
    velOutMax = b.velOutMax;
    profMaxVelDegps = b.profMaxVelDegps;
    profMaxAccelDegps2 = b.profMaxAccelDegps2;
  }

  /** Monta uma configuração. Valores não informados ficam nos padrões seguros abaixo. */
  public static final class Builder {
    private final String name;
    private double gearRatio = 1.0;
    private Gravity gravity = Gravity.COSINE;
    private boolean absInverted = false;
    private double absZeroOffsetDeg = 0.0;
    private double zeroRefDeg = 270.0;
    private double quadMaxDriftDeg = 5.0;
    private int currentLimitA = 20;
    private double maxFFVolts = 4.0;
    private double manualMaxVolts = 3.0;
    private double maxVelRpm = 150.0;
    private boolean brakeWhenStopped = true;
    private boolean limitsOn = false;
    private double limitLowDeg = 0.0;
    private double limitHighDeg = 0.0;
    private double limitMarginDeg = 10.0;
    private boolean antiBacklashOn = false;
    private double dHoldFactor = 1.0;
    private double antiBacklashDeg = 8.0;
    private double antiBacklashFactorMax = 0.7;
    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double iZoneDeg = 3.0;
    private double iDeadbandDeg = 0.5;
    private double dFilterSec = 0.005;
    private double outMax = 0.2;
    private double kS = 0.0;
    private double kG = 0.0;
    private double kV = 0.0;
    private double kA = 0.0;
    private double velKS = 0.0;
    private double velKP = 0.0;
    private double velOutMax = 0.5;
    private double profMaxVelDegps = 90.0;
    private double profMaxAccelDegps2 = 360.0;

    private Builder(String name) {
      this.name = name;
    }

    public Builder gearRatio(double v) { gearRatio = v; return this; }
    public Builder gravity(Gravity v) { gravity = v; return this; }
    public Builder absInverted(boolean v) { absInverted = v; return this; }
    public Builder absZeroOffsetDeg(double v) { absZeroOffsetDeg = v; return this; }
    public Builder zeroRefDeg(double v) { zeroRefDeg = v; return this; }
    public Builder quadMaxDriftDeg(double v) { quadMaxDriftDeg = v; return this; }
    public Builder currentLimitA(int v) { currentLimitA = v; return this; }
    public Builder maxFFVolts(double v) { maxFFVolts = v; return this; }
    public Builder manualMaxVolts(double v) { manualMaxVolts = v; return this; }
    public Builder maxVelRpm(double v) { maxVelRpm = v; return this; }

    public Builder brakeWhenStopped(boolean v) { brakeWhenStopped = v; return this; }
    /** Limites de curso: batentes (graus, no sentido negativo e positivo) e folga antes deles. */
    public Builder limits(boolean on, double lowDeg, double highDeg, double marginDeg) {
      limitsOn = on;
      limitLowDeg = lowDeg;
      limitHighDeg = highDeg;
      limitMarginDeg = marginDeg;
      return this;
    }

    /** Anti-folga (ver o campo antiBacklashOn): passa {@code deg} e volta a favor da gravidade. */
    public Builder antiBacklash(boolean on, double deg, double factorMax) {
      antiBacklashOn = on;
      antiBacklashDeg = deg;
      antiBacklashFactorMax = factorMax;
      return this;
    }

    /** Fator do D parado perto do alvo (ver o campo dHoldFactor). */
    public Builder dHoldFactor(double v) { dHoldFactor = v; return this; }

    public Builder pid(double p, double i, double d) { kP = p; kI = i; kD = d; return this; }

    /** Integral: só a menos de {@code zoneDeg} do alvo; congela a menos de {@code deadbandDeg}. */
    public Builder integral(double zoneDeg, double deadbandDeg) {
      iZoneDeg = zoneDeg;
      iDeadbandDeg = deadbandDeg;
      return this;
    }

    public Builder dFilterSec(double v) { dFilterSec = v; return this; }
    public Builder outMax(double v) { outMax = v; return this; }

    /** kS (V), kG (V), kV (V por RPM do mecanismo), kA (V por °/s²). */
    public Builder feedforward(double s, double g, double v, double a) {
      kS = s;
      kG = g;
      kV = v;
      kA = a;
      return this;
    }

    public Builder velKS(double v) { velKS = v; return this; }

    public Builder velocityLoop(double p, double outMaxFrac) {
      velKP = p;
      velOutMax = outMaxFrac;
      return this;
    }

    public Builder profile(double maxVelDegps, double maxAccelDegps2) {
      profMaxVelDegps = maxVelDegps;
      profMaxAccelDegps2 = maxAccelDegps2;
      return this;
    }

    public MechanismConfig build() {
      if (gearRatio <= 0) throw new IllegalArgumentException("gearRatio tem que ser > 0");
      return new MechanismConfig(this);
    }
  }
}
