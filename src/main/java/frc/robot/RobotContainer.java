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
import frc.robot.commands.ArmAutoTuner;
import frc.robot.commands.MechanismLimitFinder;
import frc.robot.commands.MechanismTuner;
import frc.robot.commands.MechanismSysId;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.util.TunableNumber;

/**
 * Bancada PID/FF — controle Xbox na porta 0.
 *
 *   A        → 1º toque: vai para 0°. Depois: +15° a cada toque
 *   B        → 1º toque: vai para 0°. Depois: −15° a cada toque
 *   RB + A/B → passo fino de 5°
 *   D-pad    → ↑ 90° · → 0° · ← 180° · ↓ 270° (para baixo)
 *   Analógico direito (X) → ajuste fino contínuo do alvo (até 60°/s)
 *   X        → solta o ponteiro: motor desligado em COAST (gira com a mão) até o próximo
 *              movimento; funciona também desabilitado. Sem o X, parado = BRAKE (não escorrega)
 *   Y (segurar 1 s, ponteiro SOLTO com X, pendurado e parado) → calibra o zero (pendurado = 270°)
 *   LB + analógico esquerdo (Y) → manual em malha aberta (até ±3 V)
 *
 *   O controle vibra rápido quando o ponteiro chega ao alvo, e mais longo ao calibrar o zero.
 *   O alvo atual aparece em Robot/m_robotContainer/getAlvoControleDeg (Elastic).
 *
 *   MODO TEST + Enable → AUTOAJUSTE automático (kG, kS, kP, kD). Sem botões.
 *
 *   COMANDO REMOTO (/Tuning/<nome do mecanismo>/RemoteCmd — na bancada /Tuning/Dial/RemoteCmd —,
 *   pelo Elastic ou pela ponte NetworkTables):
 *   o comando roda enquanto o valor estiver ativo E o robô HABILITADO no Driver Station.
 *   Ao desabilitar, volta sozinho para 0 — reabilitar nunca religa um teste antigo.
 *     0 nada · 1 setpoint · 2 degraus · 3 malha aberta · 4 velocidade
 *     5 degrau de velocidade · 6 perfil degrau · 7 perfil setpoint · 8 autoajuste
 *     9 SysId (kV/kA; grava /home/lvuser/sysid.csv)
 *     10 busca dos batentes (limites de curso) — NÃO usar com batente fraco (ver CLAUDE.md)
 *     11 ajuste automático genérico (sentido, kG/kV, ganhos em degraus; MechanismTuner)
 */
@Logged
public class RobotContainer {

  private static final double kStepDeg = 15.0;
  private static final double kFineStepDeg = 5.0;
  private static final double kStickDegps = 60.0;
  private static final double kAtGoalTolDeg = 1.0;

  private final MechanismSubsystem mechanism = new MechanismSubsystem();
  @NotLogged
  private final CommandXboxController controle = new CommandXboxController(0);
  @NotLogged
  // Na pasta do mecanismo (/Tuning/<nome>/): os scripts de teste (tools/ntbridge, MEC=...) acham
  // tudo pelo NOME do mecanismos/<MEC>.conf
  private final TunableNumber remoteCmd =
      new TunableNumber(mechanism.getConfig().name + "/RemoteCmd", 0.0);
  /** 1 = calibrar o zero agora (ponteiro pendurado e parado). Funciona desabilitado; volta a 0. */
  @NotLogged
  private final TunableNumber zeroNow =
      new TunableNumber(mechanism.getConfig().name + "/ZeroNow", 0.0);

  /** Alvo comandado pelo controle (0–360°). */
  @NotLogged
  private double alvoDeg = 0.0;
  /** Segue alvoDeg com perfil, somando o ajuste fino do analógico direito. */
  @NotLogged
  private final Command seguirAlvo = mechanism.profiledHoldCmd(this::alvoComAjusteFino);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    mechanism.setDefaultCommand(mechanism.stopCmd());

    // Modo TEST + Enable = autoajuste automático (nenhum botão). Disable para na hora.
    RobotModeTriggers.test().whileTrue(new ArmAutoTuner(mechanism));

    // ---- Controle ----
    controle.a().onTrue(Commands.runOnce(() -> passo(+1)));
    controle.b().onTrue(Commands.runOnce(() -> passo(-1)));
    controle.povUp().onTrue(Commands.runOnce(() -> irPara(90.0)));
    controle.povRight().onTrue(Commands.runOnce(() -> irPara(0.0)));
    controle.povLeft().onTrue(Commands.runOnce(() -> irPara(180.0)));
    controle.povDown().onTrue(Commands.runOnce(() -> irPara(270.0)));
    // Solta: cancela o alvo e põe o motor em COAST até o próximo movimento (vale desabilitado,
    // para mover o ponteiro com a mão; sem requisito, o comando padrão segue parando o motor)
    controle.x().onTrue(Commands.runOnce(() -> {
      seguirAlvo.cancel();
      mechanism.release();
    }).ignoringDisable(true));
    controle.leftBumper().whileTrue(mechanism.manualCmd(() -> -controle.getLeftY()));

    // Zero: segurar Y por 1 s, só com o motor parado (calibrar com o ponteiro sendo segurado
    // pelo motor gravaria um zero errado)
    controle.y().debounce(1.0).and(mechanism::isStopped)
        .onTrue(mechanism.zeroCmd().andThen(vibrar(0.6)).ignoringDisable(true));

    // Vibra quando o ponteiro chega ao alvo
    new Trigger(() -> seguirAlvo.isScheduled() && mechanism.isAtGoal(kAtGoalTolDeg))
        .onTrue(vibrar(0.15));

    // ---- Comando remoto ----
    RobotModeTriggers.disabled().onTrue(
        Commands.runOnce(() -> remoteCmd.set(0.0)).ignoringDisable(true));
    // Proteção contra oscilação: pode parar o motor só em teste automático (ganhos ruins de
    // propósito); com o operador, no máximo desliga o D e ele volta sozinho
    new Trigger(() -> Math.round(remoteCmd.get()) != 0)
        .onTrue(Commands.runOnce(() -> mechanism.setOscStrict(true)).ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> mechanism.setOscStrict(false)).ignoringDisable(true));
    remote(1).whileTrue(mechanism.holdTunableSetpointCmd());
    remote(2).whileTrue(mechanism.stepTestCmd());
    remote(3).whileTrue(mechanism.openLoopTunableCmd());
    remote(4).whileTrue(mechanism.velocityHoldCmd());
    remote(5).whileTrue(mechanism.velocityStepCmd());
    remote(6).whileTrue(mechanism.profiledStepCmd());
    remote(7).whileTrue(mechanism.profiledSetpointCmd());
    remote(8).whileTrue(new ArmAutoTuner(mechanism));
    remote(9).whileTrue(new MechanismSysId(mechanism));
    remote(10).whileTrue(new MechanismLimitFinder(mechanism));
    remote(11).whileTrue(new MechanismTuner(mechanism));

    // Zero remoto: não move o motor, por isso vale também desabilitado. Como no botão Y, só com o
    // motor parado (calibrar com o ponteiro segurado pelo motor gravaria um zero errado).
    Trigger zeroPedido = new Trigger(() -> zeroNow.get() > 0.5);
    zeroPedido.and(mechanism::isStopped).onTrue(
        mechanism.zeroCmd().andThen(Commands.runOnce(() -> zeroNow.set(0.0)).ignoringDisable(true)));
    zeroPedido.and(() -> !mechanism.isStopped()).onTrue(Commands.runOnce(() -> {
      System.out.println("[Mecanismo] Zero NÃO calibrado: solte o ponteiro (motor parado) e tente de novo");
      zeroNow.set(0.0);
    }).ignoringDisable(true));
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

  /** Liga a gravação a 5 ms do mecanismo, em /Mech5ms/<nome do mecanismo>/. */
  public void enableFastLog(edu.wpi.first.util.datalog.DataLog log, String logDir) {
    mechanism.enableFastLog(new frc.robot.subsystems.MechanismFastLog(
        log, logDir, mechanism.getConfig().name));
  }

  /** Malha rápida de posição (chamada pelo Robot a cada MechanismSubsystem.kFastPeriodSec). */
  public void fastLoop() {
    mechanism.fastLoop();
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
