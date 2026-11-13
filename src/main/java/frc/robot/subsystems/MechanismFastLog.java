package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.TunableNumber;

/**
 * Gravação do laço de controle a CADA 5 ms no .wpilog do robô (pasta /Mech5ms/ no AdvantageScope).
 * O Epilogue grava a cada 20 ms e o NetworkTables pelo Wi-Fi perde amostras (Lições 8 e 13):
 * aqui fica tudo, direto no arquivo, sem passar pela rede.
 *
 * <p>Só grava com o robô habilitado ou o mecanismo em movimento (parado e desabilitado não gera
 * nada). ~3000 valores/s ≈ 200 MB por hora de teste: por isso, por padrão, só liga se os logs
 * estiverem num PENDRIVE (/u). /Tuning/<nome>/FastLog: −1 = automático (só no pendrive), 0 = nunca,
 * 1 = sempre (cuidado com o disco interno da roboRIO, que tem só ~3 GB).
 */
public class MechanismFastLog {

  private final boolean onUsb;
  private final TunableNumber enable;
  private final String nome;
  private boolean warned = false;

  private final DoubleLogEntry deg;
  private final DoubleLogEntry quadDeg;
  private final DoubleLogEntry targetDeg;
  private final DoubleLogEntry goalDeg;
  private final DoubleLogEntry errorDeg;
  private final DoubleLogEntry profVel;
  private final DoubleLogEntry profAccel;
  private final DoubleLogEntry ffVolts;
  private final DoubleLogEntry pVolts;
  private final DoubleLogEntry iVolts;
  private final DoubleLogEntry dVolts;
  private final DoubleLogEntry pidVolts;
  private final DoubleLogEntry cmdVolts;
  private final DoubleLogEntry appliedVolts;
  private final DoubleLogEntry currentA;
  private final DoubleLogEntry busVolts;
  private final DoubleLogEntry rioVolts;
  private final DoubleLogEntry dtMs;
  private final IntegerLogEntry mode;

  /**
   * @param log o log do DataLogManager
   * @param logDir a pasta dos logs ({@code DataLogManager.getLogDir()}): /u... = pendrive
   * @param nome nome do mecanismo (MechanismConfig.name): grava em /Mech5ms/<nome>/ — um robô com
   *     vários mecanismos tem um log a 5 ms para cada (desde 2026-11-13; antes era /Mech5ms/)
   */
  public MechanismFastLog(DataLog log, String logDir, String nome) {
    this.nome = nome;
    enable = new TunableNumber(nome + "/FastLog", -1.0);
    onUsb = logDir != null && logDir.startsWith("/u");
    String p = "/Mech5ms/" + nome + "/";
    deg = new DoubleLogEntry(log, p + "deg");
    quadDeg = new DoubleLogEntry(log, p + "quadDeg");
    targetDeg = new DoubleLogEntry(log, p + "targetDeg");
    goalDeg = new DoubleLogEntry(log, p + "goalDeg");
    errorDeg = new DoubleLogEntry(log, p + "errorDeg");
    profVel = new DoubleLogEntry(log, p + "profVelDegps");
    profAccel = new DoubleLogEntry(log, p + "profAccelDegps2");
    ffVolts = new DoubleLogEntry(log, p + "ffVolts");
    pVolts = new DoubleLogEntry(log, p + "pVolts");
    iVolts = new DoubleLogEntry(log, p + "iVolts");
    dVolts = new DoubleLogEntry(log, p + "dVolts");
    pidVolts = new DoubleLogEntry(log, p + "pidVolts");
    cmdVolts = new DoubleLogEntry(log, p + "cmdVolts");
    appliedVolts = new DoubleLogEntry(log, p + "appliedVolts");
    currentA = new DoubleLogEntry(log, p + "currentA");
    busVolts = new DoubleLogEntry(log, p + "busVolts");
    rioVolts = new DoubleLogEntry(log, p + "rioVolts");
    dtMs = new DoubleLogEntry(log, p + "dtMs");
    mode = new IntegerLogEntry(log, p + "mode");
    System.out.println("[Mech5ms] logs em " + logDir + (onUsb ? " (pendrive): gravação a 5 ms LIGADA"
        : " (disco interno): gravação a 5 ms DESLIGADA (/Tuning/" + nome + "/FastLog = 1 para forçar)"));
  }

  private boolean active() {
    double e = enable.get();
    return e > 0.5 || (e > -1.5 && e < -0.5 && onUsb);
  }

  /** Chamado no fim de cada ciclo de 5 ms. */
  void record(MechanismSubsystem d, double dt) {
    if (!active()) return;
    if (!DriverStation.isEnabled() && d.isStopped()) return; // parado e desabilitado: nada a gravar
    if (!warned && !onUsb) {
      System.out.println("[Mech5ms] ATENÇÃO: gravando a 5 ms no disco INTERNO (~200 MB/h)");
      warned = true;
    }
    deg.append(d.getDeg());
    quadDeg.append(d.getQuadDeg());
    targetDeg.append(d.getTargetDeg());
    goalDeg.append(d.getGoalDeg());
    errorDeg.append(d.getErrorDeg());
    profVel.append(d.getProfileVelDegps());
    profAccel.append(d.getProfileAccelDegps2());
    ffVolts.append(d.getFFVolts());
    pVolts.append(d.getPTermVolts());
    iVolts.append(d.getIntegralVolts());
    dVolts.append(d.getDTermVolts());
    pidVolts.append(d.getPidVolts());
    cmdVolts.append(d.getCommandedVolts());
    appliedVolts.append(d.getAppliedVolts());
    currentA.append(d.getCurrentA());
    busVolts.append(d.getBusVolts());
    rioVolts.append(d.getRioVolts());
    dtMs.append(dt * 1000.0);
    mode.append(d.getModeCode());
  }
}
