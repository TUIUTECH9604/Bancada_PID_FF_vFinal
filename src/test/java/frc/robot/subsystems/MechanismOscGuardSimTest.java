package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Proteção contra oscilação (oscGuardStep), no estilo das equipes de ponta: numa partida o
 * mecanismo fica PIOR, nunca morto. Operador: desliga o D, que volta sozinho depois de 2 s calmo
 * (até 3 disparos); nunca para o motor. Teste automático (RemoteCmd ≠ 0): oscilar sem o D para o
 * motor até pedir OUTRO alvo. Desabilitar zera.
 */
class MechanismOscGuardSimTest {

  private static final MechanismConfig CFG = MechanismConfig.BANCADA.toBuilder("OscGuard").build();

  private MechanismSubsystem mec;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    habilitar(true);
    mec = new MechanismSubsystem(CFG);
  }

  @AfterEach
  void tearDown() {
    SimHooks.resumeTiming();
  }

  private static void habilitar(boolean on) {
    DriverStationSim.setEnabled(on);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private static void ajustar(String chave, double v) {
    NetworkTableInstance.getDefault().getEntry("/Tuning/OscGuard/" + chave).setDouble(v);
  }

  /** Segura {@code alvo} por {@code s} segundos; devolve o maior |tensão| do último 0,5 s. */
  private double segurar(double alvo, double s) {
    double vmax = 0.0;
    int n = (int) Math.round(s / 0.005);
    for (int i = 0; i < n; i++) {
      if (i % 4 == 0) {
        mec.tunerProfiled(alvo);
        mec.periodic();
      }
      SimHooks.stepTiming(0.005);
      mec.fastLoop();
      if (i >= n - 100) vmax = Math.max(vmax, Math.abs(mec.getCommandedVolts()));
    }
    return vmax;
  }

  /** Zera a proteção (como desabilitar) e volta aos ganhos do código. */
  private void zerar() {
    habilitar(false);
    mec.periodic();
    assertEquals(0, mec.getOscLevel(), "desabilitar zera");
    assertFalse(mec.isOscTripped());
    ajustar("kP", CFG.kP);
    ajustar("kD", CFG.kD);
    habilitar(true);
    mec.periodic();
    mec.tunerProfileStart();
  }

  // Um teste só por classe: a REVLib não deixa dois Spark MAX com o mesmo CAN ID no processo
  @Test
  void operadorNuncaPerdeOMecanismoTesteAutomaticoPodeParar() {
    mec.tunerProfileStart();
    segurar(0.0, 2.0);
    assertFalse(mec.isOscTripped(), "com os ganhos normais não pode disparar");
    segurar(90.0, 2.0);
    segurar(0.0, 2.0);
    assertFalse(mec.isOscTripped(), "movimentos normais não podem disparar");

    // ---- OPERADOR (oscStrict false, o padrão) ----
    // D alto demais: dispara, desliga o D; com ganhos normais de volta e 2 s calmo, o D VOLTA
    ajustar("kD", 0.004);
    mec.periodic();
    segurar(30.0, 1.0);
    assertTrue(mec.isOscTripped(), "D alto demais deveria disparar");
    assertEquals(1, mec.getOscLevel(), "1º disparo: D desligado");
    ajustar("kD", CFG.kD);
    mec.periodic();
    segurar(90.0, 2.0);
    assertEquals(90.0, mec.getDeg(), 2.0, "sem o D o mecanismo continua obedecendo");
    segurar(90.0, 2.0);
    assertEquals(0, mec.getOscLevel(), "parado e calmo por 2 s: o D volta");

    // D alto de verdade (o ciclo-limite volta sempre): depois de 3 disparos fica sem D, e
    // NUNCA para o motor, nem com P altíssimo oscilando sem o D
    zerar();
    ajustar("kD", 0.004);
    mec.periodic();
    // na simulação o D alto só oscila andando: vai e volta, com tempo para o D voltar entre eles
    for (int i = 0; i < 4; i++) segurar(i % 2 == 0 ? 30.0 : 0.0, 3.0);
    System.out.printf("[sim osc operador] nível %d, ângulo %.1f°%n", mec.getOscLevel(),
        mec.getDeg());
    assertEquals(1, mec.getOscLevel(), "3 disparos com o D: fica sem D até desabilitar");
    ajustar("kP", 0.5);
    mec.periodic();
    segurar(30.0, 2.0);
    assertEquals(1, mec.getOscLevel(), "operador: nunca para o motor");
    ajustar("kP", CFG.kP);
    mec.periodic();
    segurar(60.0, 2.0);
    assertEquals(60.0, mec.getDeg(), 2.0, "e continua obedecendo");

    // Trancos ESPAÇADOS (> 20 s) numa partida não se somam: o D volta todas as vezes
    zerar();
    for (int i = 0; i < 4; i++) {
      ajustar("kD", 0.004);
      mec.periodic();
      segurar(i % 2 == 0 ? 30.0 : 0.0, 1.0);
      assertEquals(1, mec.getOscLevel(), "disparo " + (i + 1) + " deveria desligar o D");
      ajustar("kD", CFG.kD);
      mec.periodic();
      segurar(i % 2 == 0 ? 30.0 : 0.0, 21.0);
      assertEquals(0, mec.getOscLevel(), "disparo " + (i + 1) + " espaçado: o D tem que voltar");
    }

    // ---- TESTE AUTOMÁTICO (oscStrict, RemoteCmd ≠ 0) ----
    // P e D altíssimos: 1º nível (sem D) → oscila mesmo assim → 2º nível (motor parado)
    zerar();
    mec.setOscStrict(true);
    ajustar("kP", 0.5); // a simulação não tem folga nem quase atraso: precisa de MUITO P
    ajustar("kD", 0.004);
    mec.periodic();
    double v = segurar(30.0, 4.0);
    System.out.printf("[sim osc teste] nível %d, tensão no fim %.3f V, ângulo %.1f°%n",
        mec.getOscLevel(), v, mec.getDeg());
    assertEquals(2, mec.getOscLevel(), "teste automático: oscila sem o D → motor parado");
    assertEquals(0.0, v, 1e-9, "no 2º nível, o mesmo alvo continua sem tensão");

    // Ganhos normais de volta; o MESMO alvo continua parado; OUTRO alvo libera (sem o D)
    ajustar("kP", CFG.kP);
    ajustar("kD", CFG.kD);
    mec.periodic();
    assertEquals(0.0, segurar(30.0, 0.5), 1e-9, "mesmo alvo: continua parado");
    segurar(60.0, 2.0);
    assertEquals(1, mec.getOscLevel(), "outro alvo: volta a controlar, ainda sem o D");
    assertEquals(60.0, mec.getDeg(), 2.0, "e chega no alvo novo");
    segurar(60.0, 3.0);
    assertEquals(1, mec.getOscLevel(), "no teste automático o D não volta sozinho");
    mec.setOscStrict(false);
    zerar();
  }
}
