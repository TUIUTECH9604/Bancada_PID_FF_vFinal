package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.util.TunableNumber;
import java.util.ArrayList;
import java.util.List;

/**
 * Autoajuste da bancada. Roda sozinho ao habilitar o robô no modo TEST — nenhum botão.
 * Disable a qualquer momento para tudo na hora.
 *
 * <p>Pré-requisito: zero do Through Bore calibrado (botão Y com o ponteiro pendurado e parado).
 *
 * <p>Etapas:
 *  1. ASSENTAR   — motor solto, espera o ponteiro pendurar parado e confere se o encoder lê ~270°.
 *  2. RAMPA +/−  — sobe a tensão devagar para os dois lados; com o ponteiro quase parado,
 *                  V = kS·sinal(movimento) + kG·cos(θ). Mínimos quadrados → kG e kS.
 *                  Depois desce a tensão em rampa (com peso, soltar de uma vez faz o ponteiro
 *                  despencar e balançar).
 *  3. VELOCIDADE — malha aberta em duas tensões → VelkV e VelkS (dois pontos), refino a 120 RPM
 *                  e busca do VelkP. PULADA se o ponteiro for pesado demais para girar com
 *                  segurança (precisaria de tensão alta só para passar do topo).
 *  4. kP/kD      — degraus de posição COM PERFIL entre 0° e 60°, kP crescente e kD = 0,04·kP,
 *                  kI desligado. Fica com o MAIOR par cujo overshoot ≤ 3° e tremida ao segurar
 *                  ≤ 0,6°. Depois religa o kI (com zona morta) e confere segurando 60°.
 *                  O kS medido NÃO é usado no modo posição (faz vibrar): fica 0.
 *
 * <p>Com /Tuning/Dial/AutoTuneSoKgKs = 1 (padrão), para depois da etapa 2: mede só kG e kS
 * (ex.: depois de mudar o peso do ponteiro). As etapas 3 e 4 só rodam com 0.
 *
 * <p>Ao terminar, os ganhos são aplicados na hora (aparecem em /Tuning/Dial/ no Elastic) e o
 * resumo fica em /AutoTune/... e no Riolog. Para guardar de vez, copie para os padrões do código.
 * Requer TunableNumber.TUNING_ENABLED = true.
 */
public class ArmAutoTuner extends Command {

  // ===================== Limites de segurança =====================
  private static final double kRampMinVps = 0.04;      // V/s nas rampas: mínimo (ponteiro leve)
  private static final double kRampMaxVps = 0.15;      // V/s nas rampas: máximo (ponteiro pesado)
  private static final double kRampDownVps = 0.3;      // V/s ao soltar o ponteiro de volta
  private static final double kMaxRampVolts = 3.0;     // se não subir até aqui, aborta
  private static final double kMaxSafeRpm = 400.0;     // acima disso, aborta
  private static final double kMaxCurrentA = 18.0;     // por mais de 0,3 s, aborta (logo abaixo dos 20 A do Spark: cravado nele = preso)
  private static final double kQuasiStaticRpm = 15.0;  // "quase parado" nas rampas
  private static final double kMaxSlipRpm = 30.0;      // motor × ponteiro por mais de 0,3 s, aborta
  private static final double kMaxSpinRpm = 250.0;     // teste de velocidade só se a previsão ficar abaixo

  private static final double kVelTargetRpm = 120.0;
  private static final double[] kVelKpCandidates = {0.00025, 0.0005, 0.001, 0.002};
  private static final double kVelSearchOutMax = 0.3;

  // Busca de posição: kP crescente, kD proporcional (kD = kKdRatio·kP), sempre COM perfil.
  // Medido na bancada: kP 0,003 / kD 0,00012 ótimo; kD 0,0002 já vibra a 8 Hz (atraso do loop).
  private static final double[] kPosKpCandidates = {0.0015, 0.002, 0.003, 0.004, 0.006};
  private static final double kKdRatio = 0.04;         // s
  private static final double kPosTargetADeg = 0.0;    // horizontal: gravidade máxima
  private static final double kPosTargetBDeg = 60.0;
  private static final double kStageSec = 4.0;         // cada degrau: perfil (~0,8 s) + segurar
  private static final double kHoldWindowSec = 2.0;    // janela final para medir a tremida
  private static final double kMaxOvershootDeg = 3.0;
  private static final double kMaxHoldPpDeg = 0.6;     // ~2 passos do Through Bore
  private static final double kMaxFinalErrDeg = 1.0;   // teste final (com kI)
  private static final double kMaxTrackErrDeg = 25.0;  // longe do setpoint do perfil = instável

  private enum Phase {
    ASSENTAR, RAMPA_POS, DESCANSO_1, RAMPA_NEG, DESCANSO_2,
    VEL_V1, VEL_V2, VEL_REFINO, VEL_KP, PARAR_GIRO, POS_KP, CONCLUIDO, ABORTADO
  }

  private final MechanismSubsystem mech;

  /**
   * 1 = para depois de medir kG e kS (as rampas). A busca de kP/kD daqui está desatualizada
   * (kD = 0,04·kP, sem o teste robusto) e trocaria os ganhos validados; use o robusto.sh para eles.
   * 0 = autoajuste completo, como antes.
   */
  private final TunableNumber soKgKs = new TunableNumber("Dial/AutoTuneSoKgKs", 1.0);

  private Phase phase;
  private double phaseStart;
  private double overCurrentSince;
  private double slipSince;

  // Resultados
  private double resKG, resKS, resVelKS, resVelKV, resVelKP, resKP, resKD;

  // Rampas quase-estáticas: {sinal do movimento, cos θ, tensão}
  private final List<double[]> samples = new ArrayList<>();
  private double rampVolts;
  /** Quanto o ponteiro andou desde o início da rampa atual (contínuo, para o sentido errado). */
  private double rampTravelDeg;
  private double rampLastDeg;

  // Velocidade: média pela distância percorrida (encoder do NEO, contínuo)
  private double v1, v2, rpm1, rpm2;
  private double windowStartDeg, windowStartTime;
  private boolean windowStarted;

  // Estatística do erro de velocidade
  private double errSum, errSumSq;
  private int errN;
  private int velKpIndex;
  private double bestVelStd, bestVelKp, origVelOutMax;
  private boolean velOutMaxChanged;

  // Busca do kP de posição (tudo em erro com wrap: −180° a +180°)
  private int posKpIndex, posStage;
  private double stageStart;
  private double errMin, errMax, ppMin, ppMax, absErrSum;
  private int absErrN;
  private double overUp, overDown, ppUp, ppDown, ssUp, ssDown;
  private double lastAcceptedPosKp;
  private double origKP, origKD, origKI;
  private boolean kiChanged;

  // Telemetria (/AutoTune/...)
  private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("AutoTune");
  private final StringPublisher phasePub = nt.getStringTopic("Etapa").publish();
  private final StringPublisher statusPub = nt.getStringTopic("Status").publish();
  private final StringPublisher resumoPub = nt.getStringTopic("Resumo").publish();
  private final DoublePublisher kGPub = nt.getDoubleTopic("Resultado/kG").publish();
  private final DoublePublisher kSPub = nt.getDoubleTopic("Resultado/kS").publish();
  private final DoublePublisher velKSPub = nt.getDoubleTopic("Resultado/VelkS").publish();
  private final DoublePublisher velKVPub = nt.getDoubleTopic("Resultado/VelkV").publish();
  private final DoublePublisher velKPPub = nt.getDoubleTopic("Resultado/VelkP").publish();
  private final DoublePublisher kPPub = nt.getDoubleTopic("Resultado/kP").publish();
  private final DoublePublisher kDPub = nt.getDoubleTopic("Resultado/kD").publish();

  public ArmAutoTuner(MechanismSubsystem mech) {
    this.mech = mech;
    addRequirements(mech);
    setName("Autoajuste");
  }

  // ===================== Ciclo do comando =====================

  @Override
  public void initialize() {
    resKG = resKS = resVelKS = resVelKV = resVelKP = resKP = resKD = Double.NaN;
    kiChanged = false;
    samples.clear();
    overCurrentSince = -1;
    slipSince = -1;
    velOutMaxChanged = false;
    rampVolts = 0.0;
    status("Iniciando. Disable para parar a qualquer momento.");
    goTo(Phase.ASSENTAR);
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double t = now - phaseStart;
    double deg = mech.getDeg();
    double rpm = mech.getVelRpm();

    if (safetyTripped(now, rpm)) return;

    switch (phase) {
      case ASSENTAR -> assentar(t, deg, rpm);
      case RAMPA_POS -> rampa(+1, deg, rpm);
      case DESCANSO_1 -> descanso(t, rpm, Phase.RAMPA_NEG);
      case RAMPA_NEG -> rampa(-1, deg, rpm);
      case DESCANSO_2 -> descanso(t, rpm, Phase.VEL_V1);
      case VEL_V1 -> malhaAberta(t, now, true);
      case VEL_V2 -> malhaAberta(t, now, false);
      case VEL_REFINO -> refinoVelocidade(t, now);
      case VEL_KP -> buscaVelKp(t, rpm);
      case PARAR_GIRO -> pararGiro(t, rpm);
      case POS_KP -> buscaPosKp(now, deg);
      default -> mech.tunerStop();
    }
  }

  @Override
  public boolean isFinished() {
    return phase == Phase.CONCLUIDO || phase == Phase.ABORTADO;
  }

  @Override
  public void end(boolean interrupted) {
    mech.tunerStop();
    if (velOutMaxChanged) {
      mech.setVelOutMax(origVelOutMax);
    }
    if (kiChanged) {
      // interrompido no meio da busca: volta aos ganhos de antes, não ao candidato da vez
      mech.setKP(origKP);
      mech.setKD(origKD);
      mech.setKI(origKI);
    }
    if (interrupted) {
      status("Interrompido (Disable ou outro comando) na etapa " + phase.name());
    }
    publishResults();
  }

  // ===================== Etapas =====================

  private void assentar(double t, double deg, double rpm) {
    mech.tunerStop();
    if (t > 2.5 && Math.abs(rpm) < 3.0) {
      if (!mech.tunerCheckHanging()) {
        abort(String.format("ponteiro pendurado lendo %.1f° (esperado 270°). Calibre o zero com o "
            + "botão Y (pendurado e parado) e copie kAbsZeroOffsetDeg para o código", deg));
        return;
      }
      rampVolts = 0.0;
      goTo(Phase.RAMPA_POS);
    } else if (t > 15.0) {
      abort("o ponteiro não parou pendurado em 15 s");
    }
  }

  /**
   * Rampa quase-estática: +1 sobe pelo lado direito (270°→345°), −1 pelo esquerdo (270°→195°).
   * A velocidade da rampa se adapta ao peso: kG estimado ≈ V / |cos θ|.
   */
  private void rampa(int sentido, double deg, double rpm) {
    double a = MathUtil.inputModulus(deg, 0.0, 360.0);

    // Foi para o lado errado? Compara com onde a rampa COMEÇOU (tensão positiva = ângulo subindo).
    // Antes comparava com faixas fixas em volta de 270°, o que supunha o ponteiro caindo sozinho de
    // volta a 270° entre as rampas; com a redução 15:1 ele pode ficar parado lá em cima (~345°)
    // e a rampa negativa abortaria logo no início.
    // O deslocamento é SOMADO passo a passo: a diferença direta com o início "pelo caminho mais
    // curto" vira o lado contrário depois de meia volta (abortou assim em 2026-11-11, 15° → 193°).
    if (rampVolts == 0.0) {
      rampTravelDeg = 0.0; // 1º passo da rampa
    } else {
      rampTravelDeg += MathUtil.inputModulus(a - rampLastDeg, -180.0, 180.0);
    }
    rampLastDeg = a;
    boolean ladoErrado = sentido * rampTravelDeg < -5.0;
    if (ladoErrado) {
      abort("o ângulo andou para o lado contrário da tensão. Se o ponteiro girou ANTI-HORÁRIO "
          + "com tensão positiva, troque kAbsInverted; se girou HORÁRIO, troque kInverted");
      return;
    }

    double kgEst = Math.abs(rampVolts) / Math.max(0.17, Math.abs(Math.cos(Math.toRadians(a))));
    double rate = MathUtil.clamp(0.25 * kgEst, kRampMinVps, kRampMaxVps);
    rampVolts += sentido * rate * 0.02;
    mech.tunerVolts(rampVolts);

    boolean faixa = sentido > 0 ? (a > 280.0 && a < 350.0) : (a > 190.0 && a < 260.0);
    if (faixa && Math.abs(rpm) < kQuasiStaticRpm) {
      // Desconta a parte da tensão que vai para a velocidade (kV·v): com a redução 15:1 o kV é 15×
      // maior e, a 10 RPM do ponteiro, isso já são ~0,3 V. Sem descontar, o kG saía 0,234 V em vez
      // de 0,063 V (2026-11-11).
      samples.add(new double[] {sentido, Math.cos(Math.toRadians(a)),
          rampVolts - mech.getVelKV() * rpm});
    }

    boolean chegou = sentido > 0 ? (a >= 345.0 && a < 359.0) : (a <= 195.0 && a > 181.0);
    if (chegou) {
      // Não solta de uma vez: o descanso desce a tensão em rampa
      if (sentido > 0) {
        goTo(Phase.DESCANSO_1);
      } else {
        ajustarKgKs();
        goTo(Phase.DESCANSO_2);
      }
    } else if (Math.abs(rampVolts) > kMaxRampVolts) {
      abort("o ponteiro não subiu nem com " + kMaxRampVolts + " V");
    }
  }

  private void descanso(double t, double rpm, Phase proxima) {
    // 1º: desce a tensão até zero, devagar (o ponteiro volta para baixo sem despencar)
    if (rampVolts != 0.0) {
      double passo = kRampDownVps * 0.02;
      rampVolts = Math.abs(rampVolts) <= passo ? 0.0 : rampVolts - Math.signum(rampVolts) * passo;
      mech.tunerVolts(rampVolts);
      phaseStart = Timer.getFPGATimestamp(); // o tempo de descanso só conta depois
      return;
    }
    mech.tunerStop();
    if ((t > 3.0 && Math.abs(rpm) < 3.0) || t > 10.0) {
      if (proxima == Phase.VEL_V1 && soKgKs.get() > 0.5) {
        log("Só kG/kS (/Tuning/Dial/AutoTuneSoKgKs = 1): velocidade e kP/kD não foram mexidos");
        concluir();
        return;
      }
      if (proxima == Phase.VEL_V1) {
        iniciarVelocidadeOuPular();
        return;
      }
      goTo(proxima);
    }
  }

  /** V = kS·s + kG·cos θ  →  mínimos quadrados com duas incógnitas. */
  private void ajustarKgKs() {
    long nPos = samples.stream().filter(s -> s[0] > 0).count();
    long nNeg = samples.size() - nPos;
    if (nPos < 10 || nNeg < 10) {
      log("kG/kS: poucas amostras (" + nPos + " / " + nNeg + "), mantendo os valores atuais");
      return;
    }
    double sss = 0, ssc = 0, scc = 0, ssv = 0, scv = 0;
    for (double[] p : samples) {
      sss += p[0] * p[0];
      ssc += p[0] * p[1];
      scc += p[1] * p[1];
      ssv += p[0] * p[2];
      scv += p[1] * p[2];
    }
    double det = sss * scc - ssc * ssc;
    if (Math.abs(det) < 1e-9) {
      log("kG/kS: sistema mal condicionado, mantendo os valores atuais");
      return;
    }
    double ks = (ssv * scc - scv * ssc) / det;
    double kg = (sss * scv - ssc * ssv) / det;

    if (kg < 0.02 || kg > kMaxRampVolts) {
      log(String.format("kG/kS: kG=%.3f fora do esperado, mantendo os valores atuais", kg));
      return;
    }
    resKG = kg;
    resKS = MathUtil.clamp(ks, 0.0, 0.5);
    mech.setKG(resKG);
    // kS no modo posição troca de sinal a cada cruzamento do alvo e faz o ponteiro vibrar
    // (medido na bancada). Fica 0; o kI com zona morta vence o atrito parado.
    mech.setKS(0.0);
    log(String.format("kG = %.3f V, kS medido = %.3f V (%d amostras; kS de posição = 0)",
        resKG, resKS, samples.size()));
  }

  /**
   * Para girar em malha aberta o ponteiro precisa passar do topo: V > kG + kS. Com muito peso,
   * isso dá centenas de RPM — nesse caso a etapa de velocidade é pulada (VelkV de um NEO direto
   * no eixo quase não muda com o peso; os valores atuais continuam valendo).
   */
  private void iniciarVelocidadeOuPular() {
    double kg = Double.isNaN(resKG) ? mech.getKG() : resKG;
    v1 = Math.max(0.3, 2.0 * kg);
    v2 = 1.6 * v1;
    double kv = Math.max(1e-4, mech.getVelKV());
    double rpmPrevisto = (v2 - mech.getVelKS()) / kv;
    if (rpmPrevisto > kMaxSpinRpm) {
      log(String.format("Velocidade PULADA: com kG = %.2f V precisaria de ~%.0f RPM em malha aberta "
          + "(limite %.0f). Mantendo VelkS/VelkV/VelkP atuais.", kg, rpmPrevisto, kMaxSpinRpm));
      iniciarBuscaPos();
      return;
    }
    windowStarted = false;
    goTo(Phase.VEL_V1);
  }

  /** Malha aberta: mede o RPM médio pela distância percorrida entre 1,5 s e 5 s. */
  private void malhaAberta(double t, double now, boolean primeira) {
    double volts = primeira ? v1 : v2;
    mech.tunerVolts(volts);
    double cont = mech.getContinuousDeg();

    if (!windowStarted && t >= 1.5) {
      windowStarted = true;
      windowStartDeg = cont;
      windowStartTime = now;
    }
    if (t >= 5.0) {
      double rpmMedio = (cont - windowStartDeg) / 360.0 / (now - windowStartTime) * 60.0;
      log(String.format("Malha aberta %.2f V → %.1f RPM médios", volts, rpmMedio));
      if (rpmMedio < 20.0) {
        abort(String.format("o ponteiro não girou com %.2f V", volts));
        return;
      }
      windowStarted = false;
      if (primeira) {
        rpm1 = rpmMedio;
        goTo(Phase.VEL_V2);
      } else {
        rpm2 = rpmMedio;
        if (rpm2 < rpm1 + 20.0) {
          abort("as duas velocidades ficaram parecidas demais para calcular o kV");
          return;
        }
        resVelKV = (v2 - v1) / (rpm2 - rpm1);
        resVelKS = MathUtil.clamp(v1 - resVelKV * rpm1, 0.0, 0.3);
        mech.setVelKV(resVelKV);
        mech.setVelKS(resVelKS);
        mech.setVelKP(0.0);
        log(String.format("Dois pontos: VelkV = %.5f V/RPM, VelkS = %.3f V", resVelKV, resVelKS));
        goTo(Phase.VEL_REFINO);
      }
    }
  }

  /** Só FF a 120 RPM: o erro médio que sobrar é corrigido no VelkS. */
  private void refinoVelocidade(double t, double now) {
    mech.tunerVelocity(kVelTargetRpm);
    double cont = mech.getContinuousDeg();
    if (!windowStarted && t >= 1.5) {
      windowStarted = true;
      windowStartDeg = cont;
      windowStartTime = now;
    }
    if (t >= 5.0) {
      double media = (cont - windowStartDeg) / 360.0 / (now - windowStartTime) * 60.0;
      double erro = kVelTargetRpm - media;
      resVelKS = MathUtil.clamp(resVelKS + resVelKV * erro, 0.0, 0.3);
      mech.setVelKS(resVelKS);
      log(String.format("Refino: média %.1f RPM (alvo %.0f) → VelkS = %.3f V", media, kVelTargetRpm, resVelKS));

      origVelOutMax = mech.getVelOutMax();
      mech.setVelOutMax(kVelSearchOutMax);
      velOutMaxChanged = true;
      velKpIndex = 0;
      bestVelStd = Double.POSITIVE_INFINITY;
      bestVelKp = 0.0;
      startVelTrial();
      goTo(Phase.VEL_KP);
    }
  }

  private void startVelTrial() {
    mech.setVelKP(kVelKpCandidates[velKpIndex]);
    errSum = errSumSq = 0.0;
    errN = 0;
  }

  private void buscaVelKp(double t, double rpm) {
    mech.tunerVelocity(kVelTargetRpm);
    double kp = kVelKpCandidates[velKpIndex];

    if (t > 0.5 && (rpm < -10.0 || rpm > 2.2 * kVelTargetRpm)) {
      log(String.format("VelkP %.5f: INSTÁVEL (%.0f RPM)", kp, rpm));
      encerrarBuscaVel();
      return;
    }
    if (t >= 1.5) {
      double e = kVelTargetRpm - rpm;
      errSum += e;
      errSumSq += e * e;
      errN++;
    }
    if (t >= 5.0) {
      double media = errSum / errN;
      double desvio = Math.sqrt(Math.max(0.0, errSumSq / errN - media * media));
      log(String.format("VelkP %.5f: desvio padrão %.1f RPM", kp, desvio));

      if (desvio < bestVelStd) {
        bestVelStd = desvio;
        bestVelKp = kp;
      } else {
        encerrarBuscaVel(); // piorou: o anterior era o melhor
        return;
      }
      velKpIndex++;
      if (velKpIndex >= kVelKpCandidates.length) {
        encerrarBuscaVel();
        return;
      }
      startVelTrial();
      phaseStart = Timer.getFPGATimestamp();
    }
  }

  private void encerrarBuscaVel() {
    resVelKP = bestVelKp;
    mech.setVelKP(resVelKP);
    mech.setVelOutMax(origVelOutMax);
    velOutMaxChanged = false;
    log(String.format("VelkP escolhido = %.5f (desvio %.1f RPM)", resVelKP, bestVelStd));
    goTo(Phase.PARAR_GIRO);
  }

  private void pararGiro(double t, double rpm) {
    mech.tunerStop();
    if (t > 2.0 && Math.abs(rpm) < 3.0) {
      iniciarBuscaPos();
    } else if (t > 15.0) {
      abort("o ponteiro não parou de girar em 15 s");
    }
  }

  // ---------- Busca de kP/kD de posição (com perfil) ----------

  private void iniciarBuscaPos() {
    log(String.format("Degraus COM perfil entre %.0f° e %.0f° (kD = %.2f·kP, kI desligado)",
        kPosTargetADeg, kPosTargetBDeg, kKdRatio));
    origKP = mech.getKP();
    origKD = mech.getKD();
    origKI = mech.getKI();
    mech.setKI(0.0); // a integral mascararia o amortecimento; volta no teste final
    kiChanged = true;
    posKpIndex = 0;
    lastAcceptedPosKp = Double.NaN;
    aplicarCandidato(0);
    posStage = 0;
    mech.tunerProfileStart();
    goTo(Phase.POS_KP);
    stageStart = phaseStart;
    resetStageStats();
  }

  private void aplicarCandidato(int i) {
    mech.setKP(kPosKpCandidates[i]);
    mech.setKD(kKdRatio * kPosKpCandidates[i]);
  }

  private void resetStageStats() {
    errMin = Double.POSITIVE_INFINITY;
    errMax = Double.NEGATIVE_INFINITY;
    ppMin = Double.POSITIVE_INFINITY;
    ppMax = Double.NEGATIVE_INFINITY;
    absErrSum = 0.0;
    absErrN = 0;
  }

  /**
   * Estágio 0: perfil até A e assenta. Estágio 1: perfil A→B. Estágio 2: perfil B→A.
   * Cada estágio dura kStageSec. e = ângulo − destino (com wrap). O overshoot só conta depois que
   * o perfil chegou; a tremida é o pico a pico nos últimos kHoldWindowSec.
   * Estágio 3 (final): com o kI de volta, segura B e confere.
   */
  private void buscaPosKp(double now, double deg) {
    double ts = now - stageStart;
    double alvo = (posStage == 1 || posStage == 3) ? kPosTargetBDeg : kPosTargetADeg;
    mech.tunerProfiled(alvo);

    // Instável: longe do setpoint do perfil (não do destino, que pode estar a 60°)
    if (ts > 0.5 && Math.abs(mech.getErrorDeg()) > kMaxTrackErrDeg) {
      log(String.format("kP %.4f / kD %.5f: INSTÁVEL (%.0f° do setpoint)",
          mech.getKP(), mech.getKD(), Math.abs(mech.getErrorDeg())));
      if (posStage == 3) {
        abort("ficou instável no teste final com kI");
        return;
      }
      encerrarBuscaPos();
      return;
    }

    double e = MathUtil.inputModulus(deg - alvo, -180.0, 180.0);
    if (mech.tunerProfileDone()) {
      errMin = Math.min(errMin, e);
      errMax = Math.max(errMax, e);
    }
    if (ts >= kStageSec - kHoldWindowSec) {
      ppMin = Math.min(ppMin, e);
      ppMax = Math.max(ppMax, e);
      absErrSum += Math.abs(e);
      absErrN++;
    }
    if (ts < kStageSec) return;

    double pp = ppMax - ppMin;
    double ss = absErrSum / Math.max(1, absErrN);

    if (posStage == 3) {
      String veredito = pp > kMaxHoldPpDeg ? "ATENÇÃO: tremendo (reduza kI ou kD)"
          : ss > kMaxFinalErrDeg ? "ATENÇÃO: erro alto (kP fraco demais?)" : "ok";
      log(String.format("Teste final com kI %.4f: erro %.2f°, tremida %.2f° → %s",
          origKI, ss, pp, veredito));
      concluir();
      return;
    }
    if (posStage == 1) {
      overUp = Math.max(0.0, errMax);
      ppUp = pp;
      ssUp = ss;
    } else if (posStage == 2) {
      overDown = Math.max(0.0, -errMin);
      ppDown = pp;
      ssDown = ss;

      double kp = kPosKpCandidates[posKpIndex];
      boolean aprovado = overUp <= kMaxOvershootDeg && overDown <= kMaxOvershootDeg
          && ppUp <= kMaxHoldPpDeg && ppDown <= kMaxHoldPpDeg;
      log(String.format("kP %.4f / kD %.5f: overshoot %.1f°/%.1f°, tremida %.2f°/%.2f°, "
              + "erro final %.2f°/%.2f° → %s",
          kp, kKdRatio * kp, overUp, overDown, ppUp, ppDown, ssUp, ssDown,
          aprovado ? "aprovado" : "reprovado"));
      // O kP falha nos DOIS extremos: fraco demais cede à gravidade (overshoot/erro grande),
      // forte demais vibra. Só a vibração encerra a busca (kP maior só pioraria).
      boolean vibrou = ppUp > kMaxHoldPpDeg || ppDown > kMaxHoldPpDeg;
      if (aprovado) {
        lastAcceptedPosKp = kp; // candidatos em ordem crescente: fica o maior aprovado
      } else if (vibrou) {
        encerrarBuscaPos();
        return;
      }
      posKpIndex++;
      if (posKpIndex >= kPosKpCandidates.length) {
        encerrarBuscaPos();
        return;
      }
      aplicarCandidato(posKpIndex);
      posStage = 1; // já está em A: o próximo teste começa no degrau
      stageStart = now;
      resetStageStats();
      return;
    }
    posStage++;
    stageStart = now;
    resetStageStats();
  }

  private void encerrarBuscaPos() {
    if (Double.isNaN(lastAcceptedPosKp)) {
      resKP = origKP;
      resKD = origKD;
      log(String.format("Nenhum par aprovado; mantendo os ganhos anteriores (kP %.4f / kD %.5f)",
          resKP, resKD));
    } else {
      resKP = lastAcceptedPosKp;
      resKD = kKdRatio * resKP;
      log(String.format("Escolhido: kP %.4f / kD %.5f", resKP, resKD));
    }
    mech.setKP(resKP);
    mech.setKD(resKD);

    // Teste final: kI (com zona morta) de volta, segurando B
    mech.setKI(origKI);
    kiChanged = false;
    posStage = 3;
    stageStart = Timer.getFPGATimestamp();
    resetStageStats();
  }

  private void concluir() {
    mech.tunerStop();
    status("Concluído. Ganhos aplicados — copie para o código para guardar.");
    phase = Phase.CONCLUIDO;
    phasePub.set(phase.name());
  }

  // ===================== Segurança e utilidades =====================

  private boolean safetyTripped(double now, double rpm) {
    // Motor e ponteiro têm que girar juntos (NEO direto no eixo). Se discordarem, o acoplamento
    // está patinando: o motor gira solto e o ponteiro fica para trás (visto na bancada).
    double slip = Math.abs(mech.getMotorVelRpm() - rpm);
    if (slip > kMaxSlipRpm) {
      if (slipSince < 0) slipSince = now;
      if (now - slipSince > 0.3) {
        abort(String.format("acoplamento patinando: motor %.0f RPM, ponteiro %.0f RPM. "
            + "Confira o parafuso/eixo entre o NEO e o ponteiro", mech.getMotorVelRpm(), rpm));
        return true;
      }
    } else {
      slipSince = -1;
    }
    if (Math.abs(rpm) > kMaxSafeRpm) {
      abort(String.format("velocidade %.0f RPM acima do limite", rpm));
      return true;
    }
    if (mech.isCurrentAbove(kMaxCurrentA)) {
      if (overCurrentSince < 0) overCurrentSince = now;
      if (now - overCurrentSince > 0.3) {
        abort(String.format("corrente %.1f A acima do limite", mech.getCurrentA()));
        return true;
      }
    } else {
      overCurrentSince = -1;
    }
    return false;
  }

  private void goTo(Phase next) {
    phase = next;
    phaseStart = Timer.getFPGATimestamp();
    phasePub.set(next.name());
  }

  private void abort(String motivo) {
    mech.tunerStop();
    status("ABORTADO: " + motivo);
    log("ABORTADO: " + motivo);
    phase = Phase.ABORTADO;
    phasePub.set(phase.name());
  }

  private void status(String s) {
    statusPub.set(s);
  }

  private void log(String s) {
    System.out.println("[Autoajuste] " + s);
    statusPub.set(s);
  }

  private void publishResults() {
    kGPub.set(resKG);
    kSPub.set(resKS);
    velKSPub.set(resVelKS);
    velKVPub.set(resVelKV);
    velKPPub.set(resVelKP);
    kPPub.set(resKP);
    kDPub.set(resKD);
    String resumo = String.format(
        "kG=%.3f  kS(medido)=%.3f  kP=%.4f  kD=%.5f  |  VelkS=%.3f  VelkV=%.5f  VelkP=%.5f",
        resKG, resKS, resKP, resKD, resVelKS, resVelKV, resVelKP);
    resumoPub.set(resumo);
    System.out.println("[Autoajuste] RESUMO: " + resumo);
  }
}
