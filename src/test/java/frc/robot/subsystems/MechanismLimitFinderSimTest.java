package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.commands.MechanismLimitFinder;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Busca de batentes e limites de curso contra a física simulada, com dois "parafusos" virtuais em
 * 60° e 100° (a faixa livre vai de 100° a 60° passando por 270° e 0°). Escolhidos para que o
 * caminho mais curto entre 120° e 40° atravesse os batentes (por 90°): com limites, o perfil tem
 * que dar a volta por dentro da faixa.
 */
class MechanismLimitFinderSimTest {

  private MechanismSubsystem mech;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    // referencial do braço: pendurado = −90°; batente negativo em 100° (= −260°), positivo em 60°
    MechanismSim.testStopsDeg = new double[] {100.0 - 360.0, 60.0};
    mech = new MechanismSubsystem();
  }

  @AfterEach
  void tearDown() {
    MechanismSim.testStopsDeg = null;
    mech.setLimits(0.0, 0.0, false);
    SimHooks.resumeTiming();
  }

  /** 20 ms de robô: periodic + 4 passos de 5 ms do callback rápido. */
  private void ciclo(Runnable comando) {
    mech.periodic();
    comando.run();
    for (int k = 0; k < 4; k++) {
      SimHooks.stepTiming(0.005);
      mech.fastLoop();
    }
  }

  private static double dist(double a, double b) {
    return Math.abs(MathUtil.inputModulus(a - b, -180.0, 180.0));
  }

  @Test
  void achaOsBatentesERespeitaOsLimites() {
    MechanismLimitFinder finder = new MechanismLimitFinder(mech);
    finder.initialize();
    int n = 0;
    while (!finder.isFinished() && n++ < 6000) { // até 120 s
      ciclo(finder::execute);
    }
    finder.end(false);
    System.out.printf("[sim limites] %.1f s, limites suaves %.1f° .. %.1f°%n",
        n * 0.02, mech.getSoftLowDeg(), mech.getSoftHighDeg());
    assertTrue(finder.isFinished(), "a busca não terminou");
    assertTrue(mech.limitsActive(), "limites não foram ligados");
    // batentes em 100° (−) e 60° (+), folga padrão 10°
    assertEquals(0.0, dist(mech.getSoftLowDeg(), 110.0), 2.0, "limite suave negativo");
    assertEquals(0.0, dist(mech.getSoftHighDeg(), 50.0), 2.0, "limite suave positivo");

    // Vai até 120° e depois pede 40°: tem que dar a volta por dentro (nunca entre 60° e 100°)
    double[] alvos = {120.0, 40.0, 90.0};
    double[] esperado = {120.0, 40.0, Double.NaN}; // 90° é proibido: fica num limite suave
    for (int i = 0; i < alvos.length; i++) {
      double alvo = alvos[i];
      mech.tunerProfileStart();
      double piorDentroDoVao = Double.NaN;
      for (int k = 0; k < 250; k++) { // 5 s
        ciclo(() -> mech.tunerProfiled(alvo));
        double d = mech.getDeg();
        if (d > 60.5 && d < 99.5) piorDentroDoVao = d;
      }
      double fim = mech.getDeg();
      System.out.printf("[sim limites] pediu %.0f° → parou em %.1f°%n", alvo, fim);
      assertTrue(Double.isNaN(piorDentroDoVao), "entrou no vão entre os batentes: " + piorDentroDoVao);
      if (!Double.isNaN(esperado[i])) {
        assertEquals(0.0, dist(fim, esperado[i]), 1.5, "não chegou em " + alvo);
      } else {
        assertTrue(dist(fim, 50.0) < 2.0 || dist(fim, 110.0) < 2.0,
            "alvo proibido deveria parar num limite suave, parou em " + fim);
      }
    }
  }
}
