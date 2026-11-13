package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MechanismSubsystem;

/**
 * Descobre os batentes físicos (limites de curso) SEM FORÇAR: anda devagar para o lado positivo
 * (anti-horário) até encostar, recua, e repete para o lado negativo (horário). No fim grava os
 * ângulos de contato e liga os limites de software, com folga (/Tuning/Dial/LimitMarginDeg).
 *
 * <p>Para não quebrar nada:
 *  - velocidade de busca baixa (kSeekDegps) em malha fechada de VELOCIDADE feita aqui, com tensão
 *    limitada a kMaxVolts e o limite de corrente do Spark reduzido (kSeekCurrentA) durante o teste;
 *  - contato = velocidade (pela quadratura, 0,044°) abaixo de kStallDegps por kStallSec depois de
 *    ter começado a andar → motor desligado na hora, depois recua kBackoffDeg;
 *  - aborta se não andar em kNoMoveSec, se passar de kMaxDegps, se a quadratura falhar, ou se
 *    andar kMaxTravelDeg sem achar batente (esse lado fica sem limite);
 *  - Disable para tudo; os limites antigos voltam se o teste não terminar.
 *
 * <p>NÃO usar com batente fraco: em 2026-11-11 a versão anterior arrancou um parafuso.
 *
 * <p>Comando remoto 10. Resultado em /LimitFinder/... e no Riolog (com a linha para o código).
 */
public class MechanismLimitFinder extends Command {

  // 2026-11-12, depois de arrancar um parafuso (lição 17): tudo bem mais suave. O ganho de
  // velocidade antigo (0,02 V por °/s) era ~10× alto demais para a redução 15:1 (1 V ≈ 190°/s no
  // ponteiro) e a malha chegava ao batente oscilando; a detecção esperava 0,25 s empurrando.
  // Mesmo assim: NÃO usar com batente fraco (parafuso solto). Marcar os limites à mão é o padrão.
  private static final double kSeekDegps = 8.0;       // velocidade de busca do ponteiro
  private static final double kMaxVolts = 0.5;        // tensão máxima na busca
  private static final int kSeekCurrentA = 5;         // limite de corrente do Spark na busca
  private static final double kKpVoltsPerDegps = 0.003;
  private static final double kFrictionVolts = 0.1;   // kS medido (2026-11-11, 15:1)
  private static final double kStartTravelDeg = 2.0;  // andou isso = começou a se mover
  private static final double kStallDegps = 2.0;      // abaixo disso (depois de andar) = encostou
  private static final double kStallSec = 0.12;
  private static final double kBackoffDeg = 5.0;      // recuo depois de encostar
  private static final double kNoMoveSec = 4.0;
  private static final double kMaxDegps = 48.0;       // 6× a busca: algo errado
  private static final double kMaxTravelDeg = 350.0;  // sem batente desse lado
  private static final double kVelWindowSec = 0.06;

  private enum Phase { INICIO, BUSCA_POS, RECUA_POS, BUSCA_NEG, RECUA_NEG, CONCLUIDO, ABORTADO }

  private final MechanismSubsystem mech;
  private Phase phase;
  private double phaseStart;
  private double startQuad;
  private boolean moving;
  private double stallSince;
  private double contactPos = Double.NaN;  // ângulo (0–360) do batente positivo
  private double contactNeg = Double.NaN;  // ângulo (0–360) do batente negativo
  private boolean limitsWereOn;
  private final double[] histT = new double[64];
  private final double[] histQ = new double[64];
  private int histN;

  private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("LimitFinder");
  private final StringPublisher statusPub = nt.getStringTopic("Status").publish();
  private final StringPublisher phasePub = nt.getStringTopic("Etapa").publish();
  private final DoublePublisher posPub = nt.getDoubleTopic("BatentePositivoDeg").publish();
  private final DoublePublisher negPub = nt.getDoubleTopic("BatenteNegativoDeg").publish();

  public MechanismLimitFinder(MechanismSubsystem mech) {
    this.mech = mech;
    addRequirements(mech);
    setName("Busca de limites");
  }

  @Override
  public void initialize() {
    contactPos = Double.NaN;
    contactNeg = Double.NaN;
    limitsWereOn = mech.isLimitsOn();
    mech.setLimitsOn(false); // precisa passar dos limites antigos para achar os batentes
    mech.setCurrentLimitA(kSeekCurrentA);
    goTo(Phase.INICIO);
    status("Iniciando (devagar, sem forçar). Disable para parar.");
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double t = now - phaseStart;
    if (mech.isQuadFault()) {
      abort("quadratura descartada: sem ela não dá para detectar o contato com segurança");
      return;
    }
    double q = mech.getQuadDeg();
    record(now, q);

    switch (phase) {
      case INICIO -> {
        mech.tunerStop();
        if (t > 0.5) startSeek(Phase.BUSCA_POS);
      }
      case BUSCA_POS -> seek(+1, now, t, q);
      case BUSCA_NEG -> seek(-1, now, t, q);
      case RECUA_POS -> backoff(-1, t, q, Phase.BUSCA_NEG);
      case RECUA_NEG -> backoff(+1, t, q, Phase.CONCLUIDO);
      default -> mech.tunerStop();
    }
  }

  /** Anda devagar no sentido dado até encostar (velocidade some depois de ter andado). */
  private void seek(int sentido, double now, double t, double q) {
    double traveled = sentido * (q - startQuad);
    double vel = sentido * velocity();
    if (vel > kMaxDegps) {
      abort(String.format("rápido demais (%.0f°/s)", vel));
      return;
    }
    if (!moving && traveled > kStartTravelDeg) {
      moving = true;
    }
    if (!moving && t > kNoMoveSec) {
      abort(String.format("não se moveu com até %.1f V (sentido %+d)", kMaxVolts, sentido));
      return;
    }
    if (traveled > kMaxTravelDeg) {
      log(String.format("Sentido %+d: andou %.0f° sem batente — esse lado fica sem limite",
          sentido, traveled));
      mech.tunerStop();
      if (sentido > 0) {
        startSeek(Phase.BUSCA_NEG); // sem recuo: não encostou em nada
      } else {
        concluir();
      }
      return;
    }
    if (moving && vel < kStallDegps) {
      if (stallSince < 0) stallSince = now;
      if (now - stallSince >= kStallSec) {
        mech.tunerStop(); // encostou: desliga na hora
        double deg = mech.getDeg();
        if (sentido > 0) {
          contactPos = deg;
          posPub.set(deg);
        } else {
          contactNeg = deg;
          negPub.set(deg);
        }
        log(String.format("Batente %s encontrado em %.1f° (andou %.0f°)",
            sentido > 0 ? "POSITIVO (anti-horário)" : "NEGATIVO (horário)", deg, traveled));
        goTo(sentido > 0 ? Phase.RECUA_POS : Phase.RECUA_NEG);
        startQuad = q;
        moving = false;
        return;
      }
    } else {
      stallSince = -1;
    }
    drive(sentido, vel);
  }

  /** Recua kBackoffDeg no sentido dado (afasta do batente) e passa para a próxima etapa. */
  private void backoff(int sentido, double t, double q, Phase next) {
    double traveled = sentido * (q - startQuad);
    if (traveled >= kBackoffDeg) {
      mech.tunerStop();
      if (next == Phase.CONCLUIDO) {
        concluir();
      } else {
        startSeek(next);
      }
      return;
    }
    if (t > kNoMoveSec + 2.0) { // a folga das engrenagens (~5,5°) passa antes de o ponteiro andar
      abort("não conseguiu recuar do batente");
      return;
    }
    drive(sentido, sentido * velocity());
  }

  /** Malha de velocidade simples: FF (gravidade + atrito + kV) + P, tensão limitada. */
  private void drive(int sentido, double vel) {
    double ff = mech.gravityFFVolts(mech.getDeg()) + sentido * kFrictionVolts
        + mech.getVelKV() * sentido * kSeekDegps / 6.0;
    double v = ff + sentido * kKpVoltsPerDegps * (kSeekDegps - vel);
    mech.tunerVolts(MathUtil.clamp(v, -kMaxVolts, kMaxVolts));
  }

  private void startSeek(Phase p) {
    goTo(p);
    startQuad = mech.getQuadDeg();
    moving = false;
    stallSince = -1;
    status(p == Phase.BUSCA_POS ? "Procurando o batente POSITIVO (anti-horário)..."
        : "Procurando o batente NEGATIVO (horário)...");
  }

  /** Velocidade pela quadratura na janela kVelWindowSec (°/s, com sinal). */
  private double velocity() {
    if (histN < 2) return 0.0;
    int last = (histN - 1) % histT.length;
    double tl = histT[last];
    int i = histN - 2;
    while (i > 0 && i > histN - histT.length && tl - histT[i % histT.length] < kVelWindowSec) i--;
    int k = i % histT.length;
    double dt = tl - histT[k];
    return dt > 1e-3 ? (histQ[last] - histQ[k]) / dt : 0.0;
  }

  private void record(double t, double q) {
    histT[histN % histT.length] = t;
    histQ[histN % histT.length] = q;
    histN++;
  }

  private void concluir() {
    mech.tunerStop();
    phase = Phase.CONCLUIDO;
    phasePub.set(phase.name());
    if (Double.isNaN(contactPos) || Double.isNaN(contactNeg)) {
      mech.setLimitsOn(false);
      status(String.format("Concluído SEM limites (batente + %.1f°, − %.1f°: falta um lado)",
          contactPos, contactNeg));
      return;
    }
    mech.setLimits(contactNeg, contactPos, true);
    String msg = String.format("Batentes: − %.1f°, + %.1f°. Limites ligados com %.0f° de folga. "
        + "Para o código: LimitLowDeg = %.1f, LimitHighDeg = %.1f, LimitsOn = 1",
        contactNeg, contactPos, mech.getLimitMarginDeg(), contactNeg, contactPos);
    log(msg);
    status(msg);
  }

  @Override
  public boolean isFinished() {
    return phase == Phase.CONCLUIDO || phase == Phase.ABORTADO;
  }

  @Override
  public void end(boolean interrupted) {
    mech.tunerStop();
    mech.restoreCurrentLimit();
    if (phase != Phase.CONCLUIDO) {
      mech.setLimitsOn(limitsWereOn); // não terminou: volta como estava
      if (interrupted) status("Interrompido na etapa " + phase.name());
    }
  }

  private void goTo(Phase p) {
    phase = p;
    phaseStart = Timer.getFPGATimestamp();
    phasePub.set(p.name());
  }

  private void abort(String motivo) {
    mech.tunerStop();
    phase = Phase.ABORTADO;
    phasePub.set(phase.name());
    log("ABORTADO: " + motivo);
    status("ABORTADO: " + motivo);
  }

  private void status(String s) {
    statusPub.set(s);
  }

  private void log(String s) {
    System.out.println("[Limites] " + s);
  }
}
