package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DialSubsystem;

/**
 * Caracterização (estilo SysId do WPILib) para achar kV e kA do ponteiro.
 *
 * <p>Sequência de testes em MALHA ABERTA, sempre partindo do ponteiro pendurado e parado:
 * degraus de tensão (±0,7 V e ±1,0 V: aceleração → kA) e rampas lentas (±0,25 V/s: velocidade →
 * kV). Cada teste para ao passar de kMaxRpm, ao chegar no tempo/tensão máximos, ou por segurança.
 * Entre testes o motor fica em BRAKE para frear e assentar.
 *
 * <p>Tudo é gravado a 200 Hz no callback rápido e salvo em /home/lvuser/sysid.csv, analisado
 * depois com o modelo do braço: V = kS·sinal(v) + kG·cos θ + kV·v + kA·a.
 *
 * <p>Disable a qualquer momento para tudo na hora.
 */
public class DialSysId extends Command {

  private static final double kMaxRpm = 200.0;     // para o teste ao passar disto (a leitura tem ~50 ms de atraso)
  private static final double kAbortRpm = 450.0;   // segurança absoluta
  private static final double kMaxCurrentA = 18.0;
  private static final String kCsvPath = "/home/lvuser/sysid.csv";

  /** {tipo (0 = degrau, 1 = rampa), tensão do degrau ou V/s da rampa, duração máxima (s)}. */
  private static final double[][] kTests = {
    {0, +0.7, 3.0},
    {0, -0.7, 3.0},
    {0, +1.0, 3.0},
    {0, -1.0, 3.0},
    {1, +0.25, 8.0},
    {1, -0.25, 8.0},
  };
  private static final double kMaxRampVolts = 2.0;

  private enum Phase { ASSENTAR, TESTE, CONCLUIDO, ABORTADO }

  private final DialSubsystem dial;
  private Phase phase;
  private int test;
  private double phaseStart;
  private double stillSince;
  private double overCurrentSince;

  private final StringPublisher statusPub =
      NetworkTableInstance.getDefault().getTable("SysId").getStringTopic("Status").publish();

  public DialSysId(DialSubsystem dial) {
    this.dial = dial;
    addRequirements(dial);
    setName("SysId");
  }

  @Override
  public void initialize() {
    dial.sysIdBegin();
    test = 0;
    overCurrentSince = -1;
    goTo(Phase.ASSENTAR);
    status("Assentando antes do teste 1/" + kTests.length);
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double t = now - phaseStart;
    double rpm = dial.getVelRpm();

    // Segurança
    if (Math.abs(rpm) > kAbortRpm) {
      abort(String.format("velocidade %.0f RPM", rpm));
      return;
    }
    if (dial.getCurrentA() > kMaxCurrentA) {
      if (overCurrentSince < 0) overCurrentSince = now;
      if (now - overCurrentSince > 0.3) {
        abort(String.format("corrente %.1f A", dial.getCurrentA()));
        return;
      }
    } else {
      overCurrentSince = -1;
    }

    switch (phase) {
      case ASSENTAR -> {
        dial.tunerStop(); // brake: freia
        if (Math.abs(rpm) < 2.0) {
          if (stillSince < 0) stillSince = now;
        } else {
          stillSince = -1;
        }
        boolean hanging = dial.tunerCheckHanging();
        if (stillSince > 0 && now - stillSince > 1.0 && hanging) {
          goTo(Phase.TESTE);
          status(String.format("Teste %d/%d: %s %+.2f", test + 1, kTests.length,
              kTests[test][0] == 0 ? "degrau" : "rampa (V/s)", kTests[test][1]));
        } else if (t > 20.0) {
          abort("o ponteiro não assentou pendurado em 20 s");
        }
      }
      case TESTE -> {
        double[] c = kTests[test];
        double volts = c[0] == 0 ? c[1] : c[1] * t;
        dial.sysIdRun(test, volts);
        boolean fim = t > c[2]
            || (t > 0.1 && Math.abs(rpm) > kMaxRpm)
            || Math.abs(volts) > kMaxRampVolts;
        if (fim) {
          dial.tunerStop();
          test++;
          if (test >= kTests.length) {
            phase = Phase.CONCLUIDO;
            status("Concluído. " + dial.getSysIdSamples() + " amostras em " + kCsvPath);
          } else {
            goTo(Phase.ASSENTAR);
            status("Assentando antes do teste " + (test + 1) + "/" + kTests.length);
          }
        }
      }
      default -> dial.tunerStop();
    }
  }

  @Override
  public boolean isFinished() {
    return phase == Phase.CONCLUIDO || phase == Phase.ABORTADO;
  }

  @Override
  public void end(boolean interrupted) {
    dial.sysIdEnd(kCsvPath); // grava o que tiver, mesmo interrompido
    if (interrupted) {
      status("Interrompido no teste " + (test + 1) + ". Dados parciais em " + kCsvPath);
    }
  }

  private void goTo(Phase p) {
    phase = p;
    phaseStart = Timer.getFPGATimestamp();
    stillSince = -1;
  }

  private void abort(String motivo) {
    dial.tunerStop();
    phase = Phase.ABORTADO;
    status("ABORTADO: " + motivo);
  }

  private void status(String s) {
    System.out.println("[SysId] " + s);
    statusPub.set(s);
  }
}
