package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Locale;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Calibração da simulação contra a bancada REAL: roda na física simulada os mesmos testes do
 * SysId (MechanismSysId: degraus ±0,7 e ±1,0 V por 3 s, rampas ±0,25 V/s por 8 s) e grava
 * {@code build/sim_sysid.csv} (t, V, quadratura, ângulo) a 5 ms.
 *
 * <p>Compare com o real: {@code python tools/ntbridge/identifica.py --csv build/sim_sysid.csv}
 * contra o SysId de 2026-11-13 (log a 5 ms): kS 0,114 V, kG 0,044 V (kit 0,06), kV 0,0318 V/RPM,
 * kA 0,00002–0,00005 V/(°/s²). Aqui só confere que o ponteiro anda na velocidade do kV real.
 */
class MechanismSimCalibracaoTest {

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @AfterEach
  void tearDown() {
    SimHooks.resumeTiming();
  }

  @Test
  void sysIdNaSimulacao() throws IOException {
    MechanismSubsystem d = new MechanismSubsystem(MechanismConfig.BANCADA.toBuilder("SimCal").build());
    // {tipo (0 degrau, 1 rampa), V ou V/s, duração}
    double[][] testes = {{0, 0.7, 3}, {0, -0.7, 3}, {0, 1.0, 3}, {0, -1.0, 3}, {1, 0.25, 8}, {1, -0.25, 8}};
    double vMaxPassa = 0.0;
    try (PrintWriter w = new PrintWriter(new FileWriter("build/sim_sysid.csv"))) {
      w.println("t,test,V,quadDeg,deg");
      for (int k = 0; k < testes.length; k++) {
        // assenta 1,5 s parado (BRAKE)
        d.tunerStop();
        passos(d, 1.5, null, 0, 0, 0);
        double[] tst = testes[k];
        double t0 = Timer.getFPGATimestamp();
        int n = (int) Math.round(tst[2] / 0.005);
        double q0 = d.getQuadDeg();
        for (int i = 0; i < n; i++) {
          double t = Timer.getFPGATimestamp() - t0;
          double v = tst[0] == 0 ? tst[1] : Math.signum(tst[1]) * Math.min(2.0, Math.abs(tst[1]) * t);
          if (i % 4 == 0) d.periodic();
          d.tunerVolts(v);
          SimHooks.stepTiming(0.005);
          d.fastLoop();
          w.printf(Locale.ROOT, "%.5f,%d,%.4f,%.4f,%.4f%n", Timer.getFPGATimestamp(), k,
              d.getCommandedVolts(), d.getQuadDeg(), d.getDeg());
          if (tst[0] == 0 && t > 2.0) {
            vMaxPassa = Math.max(vMaxPassa, Math.abs(d.getQuadDeg() - q0) / t);
          }
        }
      }
    }
    d.tunerStop();
    // degrau de 1 V: o real chega a ~45 RPM = 270°/s (kV 0,0318 e kS 0,114: (1−0,114)/0,0053)
    System.out.printf(Locale.ROOT, "[sim cal] velocidade média nos degraus %.0f °/s%n", vMaxPassa);
    assertTrue(vMaxPassa > 100 && vMaxPassa < 400, "velocidade fora do esperado: " + vMaxPassa);
  }

  private static void passos(MechanismSubsystem d, double s, double[] unused, int a, int b, int c) {
    int n = (int) Math.round(s / 0.005);
    for (int i = 0; i < n; i++) {
      if (i % 4 == 0) d.periodic();
      SimHooks.stepTiming(0.005);
      d.fastLoop();
    }
  }
}
