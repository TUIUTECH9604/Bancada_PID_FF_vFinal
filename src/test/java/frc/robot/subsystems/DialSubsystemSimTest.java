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
 * Testa o controle de posição contra a física simulada da bancada (DialSim), sem robô.
 * O tempo da simulação é avançado à mão: 5 ms por passo, como o callback rápido do Robot.
 *
 * <p>Rodar: ./gradlew test   (relatório em build/reports/tests/test/index.html)
 */
class DialSubsystemSimTest {

  private DialSubsystem dial;
  private double maxOvershoot;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    dial = new DialSubsystem();
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
        dial.tunerProfiled(alvo);
        dial.periodic();
      }
      SimHooks.stepTiming(0.005);
      dial.fastLoop();
      if (dial.tunerProfileDone()) {
        maxOvershoot = Math.max(maxOvershoot, sentido * erro(alvo, dial.getDeg()));
      }
    }
  }

  @Test
  void ponteiroVaiAteOAlvoESegura() {
    // Ligou: pendurado
    assertEquals(270.0, dial.getDeg(), 1.0, "deveria começar pendurado (270°)");

    dial.tunerProfileStart();
    rodar(0.0, 2.0, +1); // 270° → 0° subindo pela direita (sentido anti-horário)
    System.out.printf("[sim] 270→0: erro final %.2f°, overshoot %.2f°%n",
        erro(0.0, dial.getDeg()), maxOvershoot);
    assertEquals(0.0, erro(0.0, dial.getDeg()), 1.0, "não chegou em 0°");
    assertTrue(maxOvershoot < 4.0, "overshoot alto: " + maxOvershoot);

    rodar(90.0, 2.0, +1); // 0° → 90° (contra a gravidade)
    System.out.printf("[sim] 0→90: erro final %.2f°, overshoot %.2f°%n",
        erro(90.0, dial.getDeg()), maxOvershoot);
    assertEquals(0.0, erro(90.0, dial.getDeg()), 1.0, "não chegou em 90°");
    assertTrue(maxOvershoot < 4.0, "overshoot alto: " + maxOvershoot);

    rodar(15.0, 2.0, -1); // 90° → 15° (descendo, a gravidade ajuda)
    System.out.printf("[sim] 90→15: erro final %.2f°, overshoot %.2f°%n",
        erro(15.0, dial.getDeg()), maxOvershoot);
    assertEquals(0.0, erro(15.0, dial.getDeg()), 1.0, "não chegou em 15°");
    assertTrue(maxOvershoot < 4.0, "overshoot alto: " + maxOvershoot);

    // Solto na horizontal: tem que cair como pêndulo (passar bem do ponto mais baixo) e, com o
    // atrito pequeno do eixo, terminar parado perto de 270° — como a bancada real
    rodar(0.0, 2.0, +1);
    dial.tunerStop();
    double anterior = dial.getDeg();
    double primeiraVirada = Double.NaN;
    double velAnterior = 0.0;
    for (int i = 0; i < 4000; i++) { // 20 s
      if (i % 4 == 0) dial.periodic();
      SimHooks.stepTiming(0.005);
      dial.fastLoop();
      double deg = dial.getDeg();
      double vel = erro(anterior, deg);
      if (Double.isNaN(primeiraVirada) && velAnterior < -0.01 && vel > 0.01) {
        primeiraVirada = deg;
      }
      if (Math.abs(vel) > 0.01) velAnterior = vel;
      anterior = deg;
    }
    System.out.printf("[sim] solto em 0°: primeira virada em %.1f°, parou em %.1f°%n",
        primeiraVirada, dial.getDeg());
    assertTrue(primeiraVirada < 215.0, "o pêndulo perdeu força demais: virou em " + primeiraVirada);
    assertEquals(270.0, dial.getDeg(), 2.0, "solto, deveria parar pendurado");
  }
}
