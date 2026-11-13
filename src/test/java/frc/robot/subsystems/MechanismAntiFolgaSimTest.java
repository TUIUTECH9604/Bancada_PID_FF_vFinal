package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Anti-folga (MechanismConfig.antiBacklash): com o destino onde a gravidade é fraca e o movimento
 * chegando contra ela, o perfil passa do destino e volta a favor da gravidade. A simulação não tem
 * folga: aqui só se confere QUANDO a aproximação acontece e que o ponteiro termina no destino.
 */
class MechanismAntiFolgaSimTest {

  private static final MechanismConfig CFG = MechanismConfig.BANCADA.toBuilder("AntiFolga")
      .antiBacklash(true, 8.0, 0.7)
      .build();

  private MechanismSubsystem mec;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    mec = new MechanismSubsystem(CFG);
  }

  @AfterEach
  void tearDown() {
    SimHooks.resumeTiming();
  }

  private static double erro(double alvo, double deg) {
    return MathUtil.inputModulus(deg - alvo, -180.0, 180.0);
  }

  /** Vai até {@code alvo} por 2,5 s. Devolve {menor, maior} distância do SETPOINT ao alvo. */
  private double[] mover(double alvo, boolean[] aproximou) {
    double maxAlem = Double.NEGATIVE_INFINITY;
    double minAlem = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 500; i++) {
      if (i % 4 == 0) {
        mec.tunerProfiled(alvo);
        mec.periodic();
      }
      SimHooks.stepTiming(0.005);
      mec.fastLoop();
      if (mec.isApproaching()) aproximou[0] = true;
      double e = erro(alvo, mec.getTargetDeg());
      maxAlem = Math.max(maxAlem, e);
      minAlem = Math.min(minAlem, e);
    }
    System.out.printf("[sim anti-folga] alvo %.0f° → %.2f° (setpoint foi de %+.1f° a %+.1f° do alvo,"
        + " aproximação: %s)%n", alvo, mec.getDeg(), minAlem, maxAlem, aproximou[0]);
    assertEquals(0.0, erro(alvo, mec.getDeg()), 1.0, "não chegou em " + alvo);
    assertFalse(mec.isApproaching(), "terminou ainda indo ao ponto intermediário");
    assertTrue(mec.tunerProfileDone(), "o perfil não terminou no destino");
    return new double[] {minAlem, maxAlem};
  }

  @Test
  void passaEVoltaSoContraAGravidadePertoDaVertical() {
    mec.tunerProfileStart();
    boolean[] ap = {false};

    mover(0.0, ap); // de 270° até 0°: |cos 0°| = 1 ≥ 0,7 → direto

    // 0° → 60°: sobe CONTRA a gravidade (lado direito ela puxa para baixo) e |cos 60°| = 0,5 < 0,7
    ap[0] = false;
    double alem = mover(60.0, ap)[1];
    assertTrue(ap[0], "0→60 deveria passar e voltar");
    assertEquals(8.0, alem, 0.5, "0→60: o setpoint deveria ir até 68°");

    // 60° → 45°: desce A FAVOR da gravidade → direto
    ap[0] = false;
    mover(45.0, ap);
    assertFalse(ap[0], "60→45 desce a favor da gravidade: não precisa");

    // 45° → 30°... e 30° → 20°: |cos| ≥ 0,7 → direto (gravidade forte segura o dente)
    ap[0] = false;
    mover(20.0, ap);
    assertFalse(ap[0], "20°: gravidade forte, não precisa");

    // 20° → 90°: na vertical não há lado preferido → direto
    ap[0] = false;
    mover(90.0, ap);
    assertFalse(ap[0], "90°: gravidade nula, não precisa");

    // 90° → 120°: lado esquerdo, a gravidade puxa para 180°/270° (ângulo crescendo) → a favor → direto
    ap[0] = false;
    mover(120.0, ap);
    assertFalse(ap[0], "90→120 desce a favor da gravidade: não precisa");

    // 180° → 120°: sobe pelo lado esquerdo (contra) e |cos 120°| = 0,5 → passa (até 112°) e volta
    ap[0] = false;
    mover(180.0, ap);
    ap[0] = false;
    alem = mover(120.0, ap)[0];
    assertTrue(ap[0], "180→120 deveria passar e voltar");
    assertEquals(-8.0, alem, 0.5, "180→120: o setpoint deveria ir até 112°");

    // Passo pequeno (< 3°, ajuste fino): direto
    ap[0] = false;
    mover(118.0, ap);
    assertFalse(ap[0], "passo de 2° não usa a aproximação");
  }
}
