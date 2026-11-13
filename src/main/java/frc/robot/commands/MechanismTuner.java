package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.MechanismConfig;
import frc.robot.util.TunableNumber;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Kit de ajuste automático GENÉRICO (qualquer mecanismo da {@link MechanismConfig}), com as lições
 * da bancada embutidas. Comando remoto 11. Disable para tudo na hora.
 *
 * <p>Etapas, todas numa faixa de ±{@link #kRangeDeg} em volta de onde o mecanismo está (e dentro
 * dos limites de curso, se ligados) — nunca procura batente nem exige posição de repouso:
 * <ol>
 *   <li>SENTIDO: anda devagar para o lado positivo; se o ângulo for para o outro lado, aborta.
 *   <li>FEEDFORWARD: anda devagar (duas velocidades, os dois sentidos) com uma malha de velocidade
 *       simples e ajusta V = kS·sinal(v) + kG·fator(θ) + kV·v por mínimos quadrados, com o fator
 *       de gravidade da configuração (cos θ, constante ou nenhum).
 *   <li>GANHOS: kP/kI/kD da configuração × degraus crescentes; em cada degrau, movimentos com
 *       perfil entre dois pontos da faixa. Oscilou (4+ inversões de sentido > 0,7° em 1 s) →
 *       motor parado na hora e não sobe mais (lições 2 e 15).
 *   <li>ESCOLHA: o melhor degrau estável que fique ≥ 1,5× abaixo do primeiro que oscilou.
 *   <li>RELATÓRIO em /MechTune/... e no Riolog, com a linha para a configuração. Os valores
 *       escolhidos ficam aplicados AO VIVO (somem ao reiniciar).
 * </ol>
 *
 * <p><b>Só feedforward</b> ({@code /Tuning/<nome>/MechTuneSoFF = 1}, usado pelo assistente
 * {@code tools/ntbridge/autoajuste.sh}): para depois das passadas lentas, publica kS/kG/kV em
 * /MechTune/Resultado/... e NÃO aplica nada ao vivo (quem decide é a bateria + uma pessoa).
 *
 * <p>Depois das passadas lentas, faz {@link #kPulses} pulsos curtos de aceleração (etapa ACELERA)
 * para o PC medir o kA no log a 5 ms (identifica.py --aberta; o kit em si não calcula kA). O kA
 * medido serve ao MODELO (LQR); como feedforward ele é decidido na bateria (o do SysId atrapalhou 3
 * vezes, lição 15). Não mexe no perfil. O kS medido NÃO vai para o
 * feedforward de posição (lição 4): só aparece no relatório.
 */
public class MechanismTuner extends Command {

  // ---- Faixa e segurança ----
  private static final double kRangeDeg = 40.0;      // ± em volta do ponto inicial
  private static final double kMinRangeDeg = 10.0;   // menos que isso: aborta
  private static final double kMaxVolts = 3.0;       // teto nas etapas de malha aberta
  private static final double kStallSec = 1.0;       // tensão no teto e sem andar = travado
  private static final double kSlowDegps = 10.0;
  private static final double kFastDegps = 25.0;
  // Malha de velocidade das etapas lentas: quem faz o grosso é o FF (kV da configuração). Com
  // 15:1, 1 V ≈ 190°/s no ponteiro: ganho de 0,03 V/(°/s) dava ganho de malha ~6 e oscilava a
  // ±150°/s (simulação). Escala: ganho de malha ~0,6.
  private static final double kVelKp = 0.003;        // V por °/s de erro de velocidade
  private static final double kVelKi = 0.05;         // V por ° de erro acumulado (vence o atrito)
  private static final double kBrakeSec = 0.6;       // freia em malha fechada antes de virar
  private static final double kEndMarginDeg = 6.0;   // vira antes da ponta da faixa

  // ---- Degraus de ganho ----
  private static final double[] kMultipliers = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0};
  private static final double kSafetyMargin = 1.5;
  private static final double kMoveSec = 2.0;
  private static final int kMovesPerStage = 4;        // A→B→A→B
  private static final double kOscDeg = 0.7;
  private static final int kOscInversions = 4;

  /** Mensagens de depuração das passadas lentas no Riolog. */
  static boolean DEBUG = false;

  // ---- Pulsos de aceleração (kA, medido no PC pelo log a 5 ms: identifica.py --aberta) ----
  private static final int kPulses = 4;
  private static final double kPulseFracVolts = 0.8;   // pulso = 80% de kMaxVolts
  private static final double kPulseMaxDegps = 300.0;  // corta a aceleração ao chegar nisto
  private static final double kPulseTravelFrac = 0.3;  // ... ou ao andar 30% da faixa
  private static final double kPulseMaxSec = 0.6;      // segurança de cada metade do pulso

  private enum Phase { SENTIDO, VOLTA, CRAWL, ACELERA, GANHOS, CONCLUIDO, ABORTADO }

  private final MechanismSubsystem mech;
  private final MechanismConfig cfg;
  private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("MechTune");
  private final StringPublisher statusPub = nt.getStringTopic("Status").publish();
  private final StringPublisher etapaPub = nt.getStringTopic("Etapa").publish();
  private final StringPublisher relatorioPub = nt.getStringTopic("Relatorio").publish();
  private final DoublePublisher resKsPub = nt.getDoubleTopic("Resultado/kS").publish();
  private final DoublePublisher resKgPub = nt.getDoubleTopic("Resultado/kG").publish();
  private final DoublePublisher resKvPub = nt.getDoubleTopic("Resultado/kV").publish();
  private final DoublePublisher resRmsPub = nt.getDoubleTopic("Resultado/erroV").publish();
  private final DoublePublisher resNPub = nt.getDoubleTopic("Resultado/amostras").publish();
  /** 1 = só mede o feedforward (não testa ganhos nem aplica nada ao vivo). */
  private final TunableNumber soFF;
  private boolean soFeedforward;

  private Phase phase;
  private double phaseStart;
  private double startQuad;     // quadratura no início (referência da faixa)
  private double lowRel;        // faixa permitida, relativa ao início (graus, lowRel < 0 < highRel)
  private double highRel;
  private double startDeg;

  // histórico curto da quadratura (velocidade e oscilação)
  private final double[] histT = new double[128];
  private final double[] histQ = new double[128];
  private int histN;

  // malha de velocidade das etapas lentas
  private double velIntegral;
  private double stallSince;
  /** Freando (velocidade 0 em malha fechada) até este instante; NaN = não está freando. */
  private double brakeUntil = Double.NaN;
  private int crawlPass;
  private boolean sentidoOk;
  private double voltaFim = Double.NaN;
  private int pulse;
  private int pulseSign;
  private boolean pulseDecel;
  private double pulseStart;
  private double pulseFrom;
  /** Início da passada atual (depois de frear), para descartar a aceleração. */
  private double passStart;        // 0..3: +lento, −lento, +rápido, −rápido
  private final List<double[]> samples = new ArrayList<>(); // {sinal, fatorGravidade, rpm, volts}

  // degraus de ganho
  private double baseP;
  private double baseI;
  private double baseD;
  private int stage;
  private int move;
  private double moveStart;
  private double[][] stageScore;  // {passa, tremor, erro, chega, estável(1/0)}
  private int firstUnstable;
  private final List<double[]> moveStats = new ArrayList<>();
  private double maxOverThisMove;
  private double moveFrom;

  // resultados
  private double resKs = Double.NaN;
  private double resKg = Double.NaN;
  private double resKv = Double.NaN;
  private double resRms = Double.NaN;
  private int chosen = -1;

  public MechanismTuner(MechanismSubsystem mech) {
    this.mech = mech;
    this.cfg = mech.getConfig();
    this.soFF = new TunableNumber(cfg.name + "/MechTuneSoFF", 0.0);
    addRequirements(mech);
    setName("Ajuste automático (" + cfg.name + ")");
  }

  // ===================== Ciclo do comando =====================

  @Override
  public void initialize() {
    histN = 0;
    sentidoOk = false;
    brakeUntil = Double.NaN;
    samples.clear();
    moveStats.clear();
    resKs = resKg = resKv = resRms = Double.NaN;
    for (DoublePublisher p : new DoublePublisher[] {resKsPub, resKgPub, resKvPub, resRmsPub, resNPub}) {
      p.set(Double.NaN); // o assistente espera números NOVOS
    }
    soFeedforward = soFF.get() > 0.5;
    chosen = -1;
    firstUnstable = -1;
    baseP = mech.getKP();
    baseI = mech.getKI();
    baseD = mech.getKD();
    if (mech.isQuadFault()) {
      abort("quadratura descartada: sem ela não dá para medir velocidade/oscilação");
      return;
    }
    startQuad = mech.getQuadDeg();
    startDeg = mech.getDeg();
    double[] room = mech.roomDeg();
    lowRel = -Math.min(kRangeDeg, room[0]);
    highRel = Math.min(kRangeDeg, room[1]);
    if (highRel - lowRel < kMinRangeDeg) {
      abort(String.format("faixa livre pequena demais (%.0f°): afaste dos limites", highRel - lowRel));
      return;
    }
    log(String.format("Mecanismo %s, gravidade %s, faixa %.0f°..%.0f° em volta de %.1f°",
        cfg.name, cfg.gravity, lowRel, highRel, startDeg));
    goTo(Phase.SENTIDO);
  }

  @Override
  public void execute() {
    if (phase == Phase.CONCLUIDO || phase == Phase.ABORTADO) return;
    double now = Timer.getFPGATimestamp();
    double rel = mech.getQuadDeg() - startQuad;
    record(now, rel);
    if (mech.isQuadFault()) {
      abort("quadratura descartada no meio do ajuste");
      return;
    }
    // segurança geral: nunca sair da faixa (com 10° de tolerância para frear)
    if (rel < lowRel - 10.0 || rel > highRel + 10.0) {
      abort(String.format("saiu da faixa (%.1f°)", rel));
      return;
    }
    switch (phase) {
      case SENTIDO -> sentido(now, rel);
      case VOLTA -> volta(now, rel);
      case CRAWL -> crawl(now, rel);
      case ACELERA -> acelera(now, rel);
      case GANHOS -> ganhos(now, rel);
      default -> mech.tunerStop();
    }
  }

  // ===================== Etapas =====================

  /** Anda devagar no sentido positivo até 3°: confere que tensão positiva = ângulo subindo. */
  private void sentido(double now, double rel) {
    double t = now - phaseStart;
    if (braking(now)) return;
    if (sentidoOk) {
      goTo(Phase.VOLTA);
      return;
    }
    if (Math.abs(rel) > 3.0) {
      if (rel < 0) {
        mech.tunerStop();
        abort("tensão POSITIVA levou o ângulo para o lado NEGATIVO: troque a inversão do motor "
            + "ou do sensor na camada IO / configuração");
        return;
      }
      log("Sentido conferido (tensão positiva = ângulo positivo)");
      sentidoOk = true;
      startBrake(now);
      return;
    }
    if (t > 6.0) {
      abort("não se moveu com até " + kMaxVolts + " V");
      return;
    }
    driveVelocity(+1, kSlowDegps, now);
  }

  /** Volta para o início da faixa negativa antes de medir. */
  private void volta(double now, double rel) {
    if (braking(now)) return;
    if (!Double.isNaN(voltaFim)) { // já freou na ponta: começa a medir
      voltaFim = Double.NaN;
      crawlPass = 0;
      goTo(Phase.CRAWL);
      passStart = now;
      return;
    }
    if (rel <= lowRel + kEndMarginDeg) {
      voltaFim = now;
      startBrake(now);
      return;
    }
    if (stalled(now)) return;
    driveVelocity(-1, kFastDegps, now);
  }

  /** 4 passadas pela faixa (lento e rápido, nos dois sentidos), gravando tensão × estado. */
  private void crawl(double now, double rel) {
    if (braking(now)) return;
    if (crawlPass >= 4) {
      mech.tunerStop();
      if (!ajustaFeedforward()) return;
      iniciaAcelera(now);
      return;
    }
    int sentido = (crawlPass % 2 == 0) ? +1 : -1;
    double vel = crawlPass < 2 ? kSlowDegps : kFastDegps;
    boolean fim = sentido > 0 ? rel >= highRel - kEndMarginDeg : rel <= lowRel + kEndMarginDeg;
    if (fim) {
      crawlPass++;
      passStart = now;
      startBrake(now);
      return;
    }
    if (stalled(now)) return;
    double v = driveVelocity(sentido, vel, now);
    double medida = velocity();
    if (DEBUG && ((int) ((now - phaseStart) * 50)) % 10 == 0) {
      System.out.printf("[dbg] passada %d t %.2f rel %.1f vel %.1f (pede %.0f) V %.3f amostras %d%n",
          crawlPass, now - passStart, rel, medida, sentido * vel, v, samples.size());
    }
    // só amostras andando perto da velocidade pedida, depois de acelerar
    if (now - passStart > 0.4 && Math.abs(medida - sentido * vel) < 0.4 * vel) {
      double deg = mech.getDeg();
      samples.add(new double[] {Math.signum(medida), cfg.gravity.factor(deg), medida / 6.0, v});
    }
  }

  /**
   * Pulsos curtos de aceleração e frenagem dentro da faixa (para o PC medir o kA no log a 5 ms; as
   * passadas lentas não têm aceleração). Cada pulso: kPulseFracVolts·kMaxVolts (+ gravidade
   * medida) até kPulseMaxDegps ou kPulseTravelFrac da faixa, depois a mesma tensão ao contrário até
   * parar, e freia. Sempre na direção do lado com mais espaço. Num mecanismo muito rápido (bancada
   * 15:1: chega na velocidade final em ~6 ms, mais rápido que o laço) o kA quase não aparece: o
   * assistente usa então um limite conservador e avisa.
   */
  private void iniciaAcelera(double now) {
    pulse = 0;
    pulseSign = 0;
    goTo(Phase.ACELERA);
    status("Pulsos de aceleração (kA)");
  }

  private void acelera(double now, double rel) {
    if (braking(now)) return;
    if (pulse >= kPulses) {
      mech.tunerStop();
      if (soFeedforward) {
        concluirSoFF();
      } else {
        iniciaGanhos(now);
      }
      return;
    }
    double grav = resKg * cfg.gravity.factor(mech.getDeg());
    double vp = kPulseFracVolts * kMaxVolts;
    if (pulseSign == 0) { // começa um pulso, para o lado com mais espaço
      pulseSign = (highRel - rel) >= (rel - lowRel) ? +1 : -1;
      pulseDecel = false;
      pulseStart = now;
      pulseFrom = rel;
    }
    double t = now - pulseStart;
    double vel = velocity();
    if (!pulseDecel) {
      double andou = Math.abs(rel - pulseFrom);
      if (pulseSign * vel >= kPulseMaxDegps || andou >= kPulseTravelFrac * (highRel - lowRel)
          || t > kPulseMaxSec) {
        pulseDecel = true;
        pulseStart = now;
      } else {
        mech.tunerVolts(MathUtil.clamp(grav + pulseSign * vp, -kMaxVolts, kMaxVolts));
        return;
      }
    }
    if (pulseSign * vel <= 0.0 || now - pulseStart > kPulseMaxSec) { // parou: freia e próximo
      pulse++;
      pulseSign = 0;
      startBrake(now);
      return;
    }
    mech.tunerVolts(MathUtil.clamp(grav - pulseSign * vp, -kMaxVolts, kMaxVolts));
  }

  /** Mínimos quadrados: V = kS·s + kG·f + kV·rpm (sem a coluna kG se não há gravidade). */
  private boolean ajustaFeedforward() {
    boolean comG = cfg.gravity != MechanismConfig.Gravity.NONE;
    int m = comG ? 3 : 2;
    if (samples.size() < 30) {
      abort("poucas amostras no feedforward (" + samples.size() + ")");
      return false;
    }
    double[][] ata = new double[m][m];
    double[] aty = new double[m];
    for (double[] r : samples) {
      double[] x = comG ? new double[] {r[0], r[1], r[2]} : new double[] {r[0], r[2]};
      for (int i = 0; i < m; i++) {
        aty[i] += x[i] * r[3];
        for (int j = 0; j < m; j++) ata[i][j] += x[i] * x[j];
      }
    }
    double[] c = solve(ata, aty);
    if (c == null) {
      abort("feedforward mal condicionado (faixa de ângulo ou de velocidade pequena demais)");
      return false;
    }
    resKs = c[0];
    resKg = comG ? c[1] : 0.0;
    resKv = comG ? c[2] : c[1];
    double se = 0.0;
    for (double[] r : samples) {
      double pred = resKs * r[0] + (comG ? resKg * r[1] : 0.0) + resKv * r[2];
      se += (r[3] - pred) * (r[3] - pred);
    }
    resRms = Math.sqrt(se / samples.size());
    if (resKv <= 0.0 || resKs < -0.05 || (comG && resKg < -0.05)) {
      abort(String.format("feedforward sem sentido físico (kS %.3f, kG %.3f, kV %.4f)",
          resKs, resKg, resKv));
      return false;
    }
    log(String.format("Feedforward: kS %.3f V, kG %.3f V, kV %.4f V/RPM (erro %.3f V, %d amostras)",
        resKs, resKg, resKv, resRms, samples.size()));
    resKsPub.set(resKs);
    resKgPub.set(resKg);
    resKvPub.set(resKv);
    resRmsPub.set(resRms);
    resNPub.set(samples.size());
    if (soFeedforward) return true; // só mede: não aplica nada ao vivo
    if (comG) mech.setKG(Math.max(0.0, resKg));
    mech.setVelKV(resKv);
    mech.setKS(0.0); // lição 4: kS como feedforward de posição faz vibrar
    return true;
  }

  private void iniciaGanhos(double now) {
    stageScore = new double[kMultipliers.length][];
    stage = 0;
    aplicaDegrau(stage);
    move = 0;
    moveFrom = mech.getDeg();
    moveStart = now;
    maxOverThisMove = 0.0;
    mech.tunerProfileStart();
    goTo(Phase.GANHOS);
    status(String.format("Ganhos ×%.1f", kMultipliers[stage]));
  }

  private void aplicaDegrau(int i) {
    double k = kMultipliers[i];
    mech.setKP(baseP * k);
    mech.setKI(baseI * k);
    mech.setKD(baseD * k);
  }

  /** Alvos A e B dentro da faixa (a 25% e 75%), como ângulos absolutos. */
  private double alvo(int mv) {
    double rel = (mv % 2 == 0) ? lowRel + 0.25 * (highRel - lowRel)
        : lowRel + 0.75 * (highRel - lowRel);
    return mech.clampTargetDeg(startDeg + rel);
  }

  private void ganhos(double now, double rel) {
    double alvo = alvo(move);
    mech.tunerProfiled(alvo);
    double t = now - moveStart;
    double e = wrap180(mech.getDeg() - alvo);
    double sentido = Math.signum(wrap180(alvo - moveFrom));
    if (mech.tunerProfileDone()) {
      maxOverThisMove = Math.max(maxOverThisMove, sentido * e);
    }
    if (t > 0.3 && oscillating()) {
      mech.tunerStop();
      log(String.format("Ganhos ×%.1f: OSCILOU — motor parado", kMultipliers[stage]));
      stageScore[stage] = new double[] {0, 0, 0, 0, 0};
      firstUnstable = stage;
      escolhe();
      return;
    }
    if (t < kMoveSec) return;
    // fim do movimento: mede pelo histórico do último segundo
    double[] w = window(1.0);
    double pp = w[1] - w[0];
    double erro = Math.abs(e);
    moveStats.add(new double[] {Math.max(0.0, maxOverThisMove), pp, erro});
    move++;
    if (move < kMovesPerStage) {
      moveFrom = mech.getDeg();
      moveStart = now;
      maxOverThisMove = 0.0;
      return;
    }
    // fim do degrau
    double passa = 0, tremor = 0, erroM = 0;
    for (double[] s : moveStats) {
      passa = Math.max(passa, s[0]);
      tremor = Math.max(tremor, s[1]);
      erroM += s[2] / moveStats.size();
    }
    moveStats.clear();
    stageScore[stage] = new double[] {passa, tremor, erroM, 0, 1};
    log(String.format("Ganhos ×%.1f (kP %.4f): passa %.2f°, tremor %.2f°, erro %.2f°",
        kMultipliers[stage], mech.getKP(), passa, tremor, erroM));
    stage++;
    if (stage >= kMultipliers.length) {
      escolhe();
      return;
    }
    aplicaDegrau(stage);
    move = 0;
    moveFrom = mech.getDeg();
    moveStart = now;
    maxOverThisMove = 0.0;
    status(String.format("Ganhos ×%.1f", kMultipliers[stage]));
  }

  /** Melhor degrau estável, com margem do primeiro instável. Aplica e conclui. */
  private void escolhe() {
    double teto = firstUnstable >= 0 ? kMultipliers[firstUnstable] / kSafetyMargin
        : Double.POSITIVE_INFINITY;
    double melhor = Double.POSITIVE_INFINITY;
    for (int i = 0; i < kMultipliers.length; i++) {
      double[] s = stageScore[i];
      if (s == null || s[4] < 0.5 || kMultipliers[i] > teto + 1e-9) continue;
      // nota: passar do alvo e tremer pesam mais que o erro final (o kI corrige devagar)
      double nota = s[0] + 2.0 * s[1] + s[2];
      if (nota < melhor - 0.05) { // empate (±0,05°): fica com o menor ganho
        melhor = nota;
        chosen = i;
      }
    }
    if (chosen < 0) {
      mech.setKP(baseP);
      mech.setKI(baseI);
      mech.setKD(baseD);
      abort("nenhum degrau estável dentro da margem: ganhos originais mantidos");
      return;
    }
    aplicaDegrau(chosen);
    mech.tunerStop();
    concluir();
  }

  /** Fim do modo só feedforward: relatório, nada aplicado. */
  private void concluirSoFF() {
    phase = Phase.CONCLUIDO;
    etapaPub.set(phase.name());
    String rel = String.format(Locale.ROOT, "%s (só feedforward, nada aplicado): kS %.3f V, kG %.3f V, "
        + "kV %.4f V/RPM (erro do ajuste %.3f V, %d amostras)", cfg.name, resKs, resKg, resKv,
        resRms, samples.size());
    relatorioPub.set(rel);
    log("RELATÓRIO " + rel);
    status("Concluído (só feedforward).");
  }

  private void concluir() {
    phase = Phase.CONCLUIDO;
    etapaPub.set(phase.name());
    double k = kMultipliers[chosen];
    String instavel = firstUnstable >= 0
        ? String.format("oscilou em ×%.1f", kMultipliers[firstUnstable]) : "nenhum degrau oscilou";
    String rel = String.format(Locale.ROOT, "%s: kG %.3f V, kV %.4f V/RPM (kS medido %.3f V, erro do ajuste "
        + "%.3f V) | ganhos ×%.1f → kP %.4f, kI %.4f, kD %.6f (%s; margem %.1f×) | config: "
        + ".pid(%.4f, %.4f, %.6f).feedforward(0.0, %.3f, %.4f, kA)",
        cfg.name, resKg, resKv, resKs, resRms, k, baseP * k, baseI * k, baseD * k, instavel,
        kSafetyMargin, baseP * k, baseI * k, baseD * k, resKg, resKv);
    relatorioPub.set(rel);
    log("RELATÓRIO " + rel);
    status("Concluído — valores aplicados ao vivo. Valide com o robusto.sh antes de gravar.");
  }

  // ===================== Ferramentas =====================

  /**
   * Malha de velocidade simples (PI + FF atual), tensão limitada a kMaxVolts.
   * @return a tensão aplicada
   */
  private double driveVelocity(int sentido, double degps, double now) {
    double alvo = sentido * degps;
    double medida = velocity();
    double erro = alvo - medida;
    velIntegral = MathUtil.clamp(velIntegral + erro * 0.02, -kMaxVolts / kVelKi, kMaxVolts / kVelKi);
    double ff = mech.gravityFFVolts(mech.getDeg()) + mech.getVelKV() * alvo / 6.0;
    double v = MathUtil.clamp(ff + kVelKp * erro + kVelKi * velIntegral, -kMaxVolts, kMaxVolts);
    mech.tunerVolts(v);
    return v;
  }

  /**
   * Freia em malha fechada (velocidade 0) por kBrakeSec. Desligar o motor não basta: um mecanismo
   * leve e com pouco atrito segue girando por inércia (visto na simulação da torre).
   * @return true enquanto ainda está freando
   */
  private boolean braking(double now) {
    if (Double.isNaN(brakeUntil)) return false;
    if (now < brakeUntil) {
      driveVelocity(+1, 0.0, now);
      return true;
    }
    brakeUntil = Double.NaN;
    velIntegral = 0.0;
    return false;
  }

  private void startBrake(double now) {
    brakeUntil = now + kBrakeSec;
    velIntegral = 0.0;
  }

  /** Tensão no teto e sem andar por kStallSec = travado (sem usar a corrente: lição 6). */
  private boolean stalled(double now) {
    if (Math.abs(velocity()) < 1.0 && Math.abs(mech.getCommandedVolts()) >= kMaxVolts - 1e-6) {
      if (stallSince < 0) stallSince = now;
      if (now - stallSince > kStallSec) {
        abort("travado: tensão no máximo e o mecanismo não anda (obstáculo ou falta de força)");
        return true;
      }
    } else {
      stallSince = -1;
    }
    return false;
  }

  private void record(double t, double q) {
    histT[histN % histT.length] = t;
    histQ[histN % histT.length] = q;
    histN++;
  }

  /** Velocidade pela quadratura nos últimos ~0,06 s (°/s). */
  private double velocity() {
    if (histN < 2) return 0.0;
    int last = (histN - 1) % histT.length;
    int i = histN - 2;
    while (i > 0 && i > histN - histT.length && histT[last] - histT[i % histT.length] < 0.06) i--;
    int k = i % histT.length;
    double dt = histT[last] - histT[k];
    return dt > 1e-3 ? (histQ[last] - histQ[k]) / dt : 0.0;
  }

  /** {mínimo, máximo} da quadratura no último {@code sec} segundos. */
  private double[] window(double sec) {
    double lo = Double.POSITIVE_INFINITY;
    double hi = Double.NEGATIVE_INFINITY;
    int last = (histN - 1) % histT.length;
    for (int i = histN - 1; i >= 0 && i > histN - histT.length; i--) {
      int k = i % histT.length;
      if (histT[last] - histT[k] > sec) break;
      lo = Math.min(lo, histQ[k]);
      hi = Math.max(hi, histQ[k]);
    }
    return new double[] {lo, hi};
  }

  /** 4+ inversões de sentido de mais de kOscDeg no último segundo (o mesmo critério do robusto). */
  private boolean oscillating() {
    int last = (histN - 1) % histT.length;
    int inv = 0;
    double ext = Double.NaN;
    int dir = 0;
    int start = Math.max(0, histN - histT.length + 1);
    for (int i = start; i < histN; i++) {
      int k = i % histT.length;
      if (histT[last] - histT[k] > 1.0) continue;
      double q = histQ[k];
      if (Double.isNaN(ext)) {
        ext = q;
        continue;
      }
      if (dir >= 0 && q < ext - kOscDeg) {
        if (dir > 0) inv++;
        dir = -1;
        ext = q;
      } else if (dir <= 0 && q > ext + kOscDeg) {
        if (dir < 0) inv++;
        dir = +1;
        ext = q;
      } else if ((dir > 0 && q > ext) || (dir < 0 && q < ext)) {
        ext = q;
      }
    }
    return inv >= kOscInversions;
  }

  /** Resolve A·x = b (Gauss com pivô). null se singular. */
  private static double[] solve(double[][] a, double[] b) {
    int n = b.length;
    double[][] m = new double[n][n + 1];
    for (int i = 0; i < n; i++) {
      System.arraycopy(a[i], 0, m[i], 0, n);
      m[i][n] = b[i];
    }
    for (int c = 0; c < n; c++) {
      int p = c;
      for (int r = c + 1; r < n; r++) if (Math.abs(m[r][c]) > Math.abs(m[p][c])) p = r;
      if (Math.abs(m[p][c]) < 1e-9) return null;
      double[] tmp = m[c];
      m[c] = m[p];
      m[p] = tmp;
      for (int r = 0; r < n; r++) {
        if (r == c) continue;
        double f = m[r][c] / m[c][c];
        for (int k = c; k <= n; k++) m[r][k] -= f * m[c][k];
      }
    }
    double[] x = new double[n];
    for (int i = 0; i < n; i++) x[i] = m[i][n] / m[i][i];
    return x;
  }

  private static double wrap180(double d) {
    return MathUtil.inputModulus(d, -180.0, 180.0);
  }

  @Override
  public boolean isFinished() {
    return phase == Phase.CONCLUIDO || phase == Phase.ABORTADO;
  }

  @Override
  public void end(boolean interrupted) {
    mech.tunerStop();
    if (phase != Phase.CONCLUIDO) {
      // não terminou: volta os ganhos de antes (o feedforward medido, se houver, fica)
      mech.setKP(baseP);
      mech.setKI(baseI);
      mech.setKD(baseD);
      if (interrupted) status("Interrompido na etapa " + phase + " — ganhos de antes restaurados");
    }
  }

  private void goTo(Phase p) {
    phase = p;
    phaseStart = Timer.getFPGATimestamp();
    stallSince = -1;
    velIntegral = 0.0;
    etapaPub.set(p.name());
  }

  private void abort(String motivo) {
    mech.tunerStop();
    phase = Phase.ABORTADO;
    etapaPub.set(phase.name());
    log("ABORTADO: " + motivo);
    status("ABORTADO: " + motivo);
  }

  private void status(String s) {
    statusPub.set(s);
  }

  private void log(String s) {
    System.out.println("[MechTune] " + s);
    statusPub.set(s);
  }

  // para testes
  public double getResultKs() {
    return resKs;
  }

  public double getResultKg() {
    return resKg;
  }

  public double getResultKv() {
    return resKv;
  }

  public double getChosenMultiplier() {
    return chosen >= 0 ? kMultipliers[chosen] : Double.NaN;
  }

  public boolean succeeded() {
    return phase == Phase.CONCLUIDO;
  }
}
