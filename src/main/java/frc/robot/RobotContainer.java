package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DialAutoTuner;
import frc.robot.commands.DialSysId;
import frc.robot.subsystems.DialSubsystem;
import frc.robot.util.TunableNumber;

/**
 * Bancada PID/FF — controle Xbox na porta 0.
 *
 *   A        → 1º toque: vai para 0°. Depois: +15° a cada toque
 *   B        → 1º toque: vai para 0°. Depois: −15° a cada toque
 *   RB + A/B → passo fino de 5°
 *   D-pad    → ↑ 90° · → 0° · ← 180° · ↓ 270° (para baixo)
 *   Analógico direito (X) → ajuste fino contínuo do alvo (até 60°/s)
 *   X        → solta o ponteiro (motor desligado)
 *   Y (segurar 1 s, ponteiro SOLTO, pendurado e parado) → calibra o zero (pendurado = 270°)
 *   LB + analógico esquerdo (Y) → manual em malha aberta (até ±3 V)
 *
 *   O controle vibra rápido quando o ponteiro chega ao alvo, e mais longo ao calibrar o zero.
 *   O alvo atual aparece em Robot/m_robotContainer/getAlvoControleDeg (Elastic).
 *
 *   MODO TEST + Enable → AUTOAJUSTE automático (kG, kS, kP, kD). Sem botões.
 *
 *   COMANDO REMOTO (/Tuning/Dial/RemoteCmd, pelo Elastic ou pela ponte NetworkTables):
 *   o comando roda enquanto o valor estiver ativo E o robô HABILITADO no Driver Station.
 *   Ao desabilitar, volta sozinho para 0 — reabilitar nunca religa um teste antigo.
 *     0 nada · 1 setpoint · 2 degraus · 3 malha aberta · 4 velocidade
 *     5 degrau de velocidade · 6 perfil degrau · 7 perfil setpoint · 8 autoajuste
 *     9 SysId (kV/kA; grava /home/lvuser/sysid.csv)
 */
@Logged
public class RobotContainer {

  private static final double kStepDeg = 15.0;
  private static final double kFineStepDeg = 5.0;
  private static final double kStickDegps = 60.0;
  private static final double kAtGoalTolDeg = 1.0;

  private final DialSubsystem dial = new DialSubsystem();
  @NotLogged
  private final CommandXboxController controle = new CommandXboxController(0);
  @NotLogged
  private final TunableNumber remoteCmd = new TunableNumber("Dial/RemoteCmd", 0.0);
  /** 1 = calibrar o zero agora (ponteiro pendurado e parado). Funciona desabilitado; volta a 0. */
  @NotLogged
  private final TunableNumber zeroNow = new TunableNumber("Dial/ZeroNow", 0.0);

  /** Alvo comandado pelo controle (0–360°). */
  @NotLogged
  private double alvoDeg = 0.0;
  /** Segue alvoDeg com perfil, somando o ajuste fino do analógico direito. */
  @NotLogged
  private final Command seguirAlvo = dial.profiledHoldCmd(this::alvoComAjusteFino);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    dial.setDefaultCommand(dial.stopCmd());

    // Modo TEST + Enable = autoajuste automático (nenhum botão). Disable para na hora.
    RobotModeTriggers.test().whileTrue(new DialAutoTuner(dial));

    // ---- Controle ----
    controle.a().onTrue(Commands.runOnce(() -> passo(+1)));
    controle.b().onTrue(Commands.runOnce(() -> passo(-1)));
    controle.povUp().onTrue(Commands.runOnce(() -> irPara(90.0)));
    controle.povRight().onTrue(Commands.runOnce(() -> irPara(0.0)));
    controle.povLeft().onTrue(Commands.runOnce(() -> irPara(180.0)));
    controle.povDown().onTrue(Commands.runOnce(() -> irPara(270.0)));
    controle.x().onTrue(Commands.runOnce(seguirAlvo::cancel)); // volta ao padrão: motor parado
    controle.leftBumper().whileTrue(dial.manualCmd(() -> -controle.getLeftY()));

    // Zero: segurar Y por 1 s, só com o motor parado (calibrar com o ponteiro sendo segurado
    // pelo motor gravaria um zero errado)
    controle.y().debounce(1.0).and(dial::isStopped)
        .onTrue(dial.zeroCmd().andThen(vibrar(0.6)).ignoringDisable(true));

    // Vibra quando o ponteiro chega ao alvo
    new Trigger(() -> seguirAlvo.isScheduled() && dial.isAtGoal(kAtGoalTolDeg))
        .onTrue(vibrar(0.15));

    // ---- Comando remoto ----
    RobotModeTriggers.disabled().onTrue(
        Commands.runOnce(() -> remoteCmd.set(0.0)).ignoringDisable(true));
    remote(1).whileTrue(dial.holdTunableSetpointCmd());
    remote(2).whileTrue(dial.stepTestCmd());
    remote(3).whileTrue(dial.openLoopTunableCmd());
    remote(4).whileTrue(dial.velocityHoldCmd());
    remote(5).whileTrue(dial.velocityStepCmd());
    remote(6).whileTrue(dial.profiledStepCmd());
    remote(7).whileTrue(dial.profiledSetpointCmd());
    remote(8).whileTrue(new DialAutoTuner(dial));
    remote(9).whileTrue(new DialSysId(dial));

    // Zero remoto: não move o motor, por isso vale também desabilitado
    new Trigger(() -> zeroNow.get() > 0.5).onTrue(
        dial.zeroCmd().andThen(Commands.runOnce(() -> zeroNow.set(0.0)).ignoringDisable(true)));
  }

  /** A (+1) ou B (−1): o 1º toque vai para 0°; os seguintes andam um passo. */
  private void passo(int sentido) {
    if (!seguirAlvo.isScheduled()) {
      irPara(0.0);
      return;
    }
    double passo = controle.getHID().getRightBumperButton() ? kFineStepDeg : kStepDeg;
    irPara(alvoDeg + sentido * passo);
  }

  private void irPara(double deg) {
    alvoDeg = MathUtil.inputModulus(deg, 0.0, 360.0);
    if (!seguirAlvo.isScheduled()) {
      CommandScheduler.getInstance().schedule(seguirAlvo);
    }
  }

  /** Chamado a cada ciclo pelo comando de perfil: aplica o ajuste fino do analógico direito. */
  private double alvoComAjusteFino() {
    double stick = MathUtil.applyDeadband(controle.getRightX(), 0.1);
    if (stick != 0.0) {
      alvoDeg = MathUtil.inputModulus(alvoDeg - stick * kStickDegps * 0.02, 0.0, 360.0);
    }
    return alvoDeg;
  }

  /** Vibração curta no controle (não usa o subsistema; funciona desabilitado). */
  private Command vibrar(double segundos) {
    return Commands.startEnd(
            () -> controle.setRumble(RumbleType.kBothRumble, 0.6),
            () -> controle.setRumble(RumbleType.kBothRumble, 0.0))
        .withTimeout(segundos)
        .ignoringDisable(true);
  }

  /** Alvo do controle (NaN se o controle não estiver comandando o ponteiro). */
  public double getAlvoControleDeg() {
    return seguirAlvo.isScheduled() ? alvoDeg : Double.NaN;
  }

  private Trigger remote(int code) {
    return new Trigger(() -> DriverStation.isEnabled() && Math.round(remoteCmd.get()) == code);
  }

  /** Malha rápida de posição (chamada pelo Robot a cada DialSubsystem.kFastPeriodSec). */
  public void fastLoop() {
    dial.fastLoop();
  }

  /** Código do comando remoto ativo (0 = nenhum). */
  public double getRemoteCmd() {
    return remoteCmd.get();
  }

  @NotLogged
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
