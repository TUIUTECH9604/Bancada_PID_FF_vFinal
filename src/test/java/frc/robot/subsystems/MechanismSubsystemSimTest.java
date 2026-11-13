package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
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
 * Testa o controle de posição contra a física simulada da bancada (MechanismSim), sem robô.
 * O tempo da simulação é avançado à mão: 5 ms por passo, como o callback rápido do Robot.
 *
 * <p>Rodar: ./gradlew test   (relatório em build/reports/tests/test/index.html)
 */
class MechanismSubsystemSimTest {

  private MechanismSubsystem mech;
  private double maxOvershoot;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    mech = new MechanismSubsystem();
  }

  @AfterEach
  void tearDown() {
    SimHooks.resumeTiming();
  }

  private static double erro(double alvo, double deg) {
    return MathUtil.inputModulus(deg - alvo, -180.0, 180.0);
  }

  /**
   * Roda o controle por {@code segundos}, pedindo o alvo com perfil a cada 20 ms (como um
   * comando) e o callback rápido a cada 5 ms. Mede o maior overshoot depois que o perfil chega.
   */
  private void rodar(double alvo, double segundos, int sentido) {
    maxOvershoot = 0.0;
    int passos = (int) Math.round(segundos / 0.005);
    for (int i = 0; i < passos; i++) {
      if (i % 4 == 0) {
        mech.tunerProfiled(alvo);
        mech.periodic();
      }
      SimHooks.stepTiming(0.005);
      mech.fastLoop();
      if (mech.tunerProfileDone()) {
        maxOvershoot = Math.max(maxOvershoot, sentido * erro(alvo, mech.getDeg()));
      }
    }
  }

  @Test
  void ponteiroVaiAteOAlvoESegura() {
    // Ligou: pendurado
    assertEquals(270.0, mech.getDeg(), 1.0, "deveria começar pendurado (270°)");

    mech.tunerProfileStart();
    rodar(0.0, 2.0, +1); // 270° → 0° subindo pela direita (sentido anti-horário)
    System.out.printf("[sim] 270→0: erro final %.2f°, overshoot %.2f°%n",
        erro(0.0, mech.getDeg()), maxOvershoot);
    assertEquals(0.0, erro(0.0, mech.getDeg()), 1.0, "não chegou em 0°");
    assertTrue(maxOvershoot < 4.0, "overshoot alto: " + maxOvershoot);

    rodar(90.0, 2.0, +1); // 0° → 90° (contra a gravidade)
    System.out.printf("[sim] 0→90: erro final %.2f°, overshoot %.2f°%n",
        erro(90.0, mech.getDeg()), maxOvershoot);
    assertEquals(0.0, erro(90.0, mech.getDeg()), 1.0, "não chegou em 90°");
    assertTrue(maxOvershoot < 4.0, "overshoot alto: " + maxOvershoot);

    rodar(15.0, 2.0, -1); // 90° → 15° (descendo, a gravidade ajuda)
    System.out.printf("[sim] 90→15: erro final %.2f°, overshoot %.2f°%n",
        erro(15.0, mech.getDeg()), maxOvershoot);
    assertEquals(0.0, erro(15.0, mech.getDeg()), 1.0, "não chegou em 15°");
    assertTrue(maxOvershoot < 4.0, "overshoot alto: " + maxOvershoot);

    // Solto na horizontal: tem que cair como pêndulo (passar bem do ponto mais baixo) e, com o
    // atrito pequeno do eixo, terminar parado perto de 270° — como a bancada real
    rodar(0.0, 2.0, +1);
    // Parado = BRAKE (padrão desde 2026-11-12): em 0° (horizontal, gravidade máxima) não escorrega
    mech.tunerStop();
    // O BRAKE do NEO só curto-circuita as bobinas: freia o movimento, mas não trava. A simulação
    // (sem o atrito da caixa calibrado) desce ~2°/s; no robô em COAST foram 30° em 1,5 s.
    for (int i = 1; i <= 400; i++) { // 2 s
      if (i % 4 == 0) mech.periodic();
      SimHooks.stepTiming(0.005);
      mech.fastLoop();
    }
    System.out.printf("[sim] parado em BRAKE na horizontal: 2 s -> %.2f°%n",
        erro(0.0, mech.getDeg()));
    assertTrue(mech.isBrakeMode(), "parado tem que ficar em BRAKE");
    assertEquals(0.0, erro(0.0, mech.getDeg()), 6.0, "parado em BRAKE caiu da horizontal");
    // X (release) = COAST até o próximo movimento
    mech.release();
    assertTrue(!mech.isBrakeMode(), "release() tem que soltar (COAST)");
    double anterior = mech.getDeg();
    double primeiraVirada = Double.NaN;
    double velAnterior = 0.0;
    for (int i = 0; i < 4000; i++) { // 20 s
      if (i % 4 == 0) mech.periodic();
      SimHooks.stepTiming(0.005);
      mech.fastLoop();
      double deg = mech.getDeg();
      double vel = erro(anterior, deg);
      if (Double.isNaN(primeiraVirada) && velAnterior < -0.01 && vel > 0.01) {
        primeiraVirada = deg;
      }
      if (Math.abs(vel) > 0.01) velAnterior = vel;
      anterior = deg;
    }
    System.out.printf("[sim] solto em 0°: primeira virada em %.1f°, parou em %.1f°%n",
        primeiraVirada, mech.getDeg());
    assertTrue(!mech.isBrakeMode(), "solto e parado tem que continuar em COAST");
    // Qualquer movimento volta ao BRAKE
    rodar(270.0, 0.1, +1);
    assertTrue(mech.isBrakeMode(), "ao mover de novo tem que voltar ao BRAKE");
    // Sem verificação desde a redução 15:1 (2026-11-11): os números do pêndulo eram da bancada sem
    // redução (virava em ~192° e parava a ±3° de 270°). Com a caixa, o atrito do motor pesa 15×
    // no ponteiro e a simulação ainda não foi recalibrada (medir o ponteiro real solto primeiro).
  }
}
