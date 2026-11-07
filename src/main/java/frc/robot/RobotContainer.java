package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DialAutoTuner;
import frc.robot.commands.DialSysId;
import frc.robot.subsystems.DialSubsystem;
import frc.robot.util.TunableNumber;

/**
 * Bancada PID/FF. Todos os botões funcionam "enquanto apertado": soltou, o motor para.
 *
 *   MODO TEST + Enable → AUTOAJUSTE automático (kG, kS, VelkV, VelkS, VelkP, kP). Sem botões.
 *
 *   A        → segue TestSetpointDeg (mude no Elastic)
 *   B        → sequência de degraus: StepA → B → C → D e volta
 *   X        → malha aberta com OpenLoopVolts (medir kS e kG)
 *   LB + LY  → manual pelo analógico esquerdo (até ±3 V)
 *   Y        → zerar o Through Bore (ponteiro pendurado e parado = 270°; funciona desabilitado)
 *
 *   Velocidade:
 *   RB       → velocidade constante VelTargetRPM
 *   RT       → degrau de velocidade: alterna VelStepA ↔ VelStepB
 *
 *   Perfil de movimento (compare com A e B):
 *   D-pad ↑  → mesma sequência COM perfil
 *   D-pad ↓  → perfil até TestSetpointDeg
 *
 *   COMANDO REMOTO (/Tuning/Dial/RemoteCmd, pelo Elastic ou pela ponte NetworkTables):
 *   o comando roda enquanto o valor estiver ativo E o robô HABILITADO no Driver Station.
 *   Ao desabilitar, volta sozinho para 0 — reabilitar nunca religa um teste antigo.
 *     0 nada · 1 setpoint (A) · 2 degraus (B) · 3 malha aberta (X) · 4 velocidade (RB)
 *     5 degrau de velocidade (RT) · 6 perfil degrau (↑) · 7 perfil setpoint (↓) · 8 autoajuste
 *     9 SysId (kV/kA; grava /home/lvuser/sysid.csv)
 */
@Logged
public class RobotContainer {

  private final DialSubsystem dial = new DialSubsystem();
  @NotLogged
  private final CommandXboxController controle = new CommandXboxController(0);
  @NotLogged
  private final TunableNumber remoteCmd = new TunableNumber("Dial/RemoteCmd", 0.0);
  /** 1 = calibrar o zero agora (ponteiro pendurado e parado). Funciona desabilitado; volta a 0. */
  @NotLogged
  private final TunableNumber zeroNow = new TunableNumber("Dial/ZeroNow", 0.0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    dial.setDefaultCommand(dial.stopCmd());

    // Modo TEST + Enable = autoajuste automático (nenhum botão). Disable para na hora.
    RobotModeTriggers.test().whileTrue(new DialAutoTuner(dial));

    controle.a().whileTrue(dial.holdTunableSetpointCmd());
    controle.b().whileTrue(dial.stepTestCmd());
    controle.x().whileTrue(dial.openLoopTunableCmd());
    controle.leftBumper().whileTrue(dial.manualCmd(() -> -controle.getLeftY()));
    controle.y().onTrue(dial.zeroCmd());

    controle.rightBumper().whileTrue(dial.velocityHoldCmd());
    controle.rightTrigger().whileTrue(dial.velocityStepCmd());

    controle.povUp().whileTrue(dial.profiledStepCmd());
    controle.povDown().whileTrue(dial.profiledSetpointCmd());

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
