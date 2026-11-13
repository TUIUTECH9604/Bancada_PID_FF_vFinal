package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.MechanismConfig;
import frc.robot.subsystems.SimTestHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * O kit de ajuste automático contra a física simulada: a bancada (braço, kG 0,063 V na física) e
 * uma torre sem gravidade, os dois partindo de ganhos e kG ERRADOS na configuração.
 */
class MechanismTunerSimTest {

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
    SimTestHooks.reset();
    SimHooks.resumeTiming();
  }

  static MechanismTuner roda(MechanismSubsystem d) {
    return roda(d, new MechanismTuner(d));
  }

  /** Roda um kit já criado (no robô ele é criado uma vez, ao ligar, antes dos ajustes ao vivo). */
  static MechanismTuner roda(MechanismSubsystem d, MechanismTuner t) {
    t.initialize();
    int n = 0;
    while (!t.isFinished() && n++ < 10000) { // até 200 s
      d.periodic();
      t.execute();
      for (int k = 0; k < 4; k++) {
        SimHooks.stepTiming(0.005);
        d.fastLoop();
      }
    }
    t.end(false);
    System.out.printf("[sim tuner] %.0f s: kG %.3f, kV %.4f, ganhos ×%.1f, sucesso %b%n",
        n * 0.02, t.getResultKg(), t.getResultKv(), t.getChosenMultiplier(), t.succeeded());
    return t;
  }

  @Test
  void bancadaComKgErradoEGanhosBaixos() {
    // começa com kG zero e metade dos ganhos: o kit tem que achar o kG e subir os ganhos
    MechanismConfig c = MechanismConfig.BANCADA.toBuilder("TunerBancada")
        .feedforward(0.0, 0.0, 0.0318, 0.0)
        .pid(0.003, 0.01, 0.00015)
        .build();
    MechanismSubsystem d = new MechanismSubsystem(c);
    MechanismTuner t = roda(d);
    assertTrue(t.succeeded(), "o ajuste não terminou com sucesso");
    assertEquals(0.063, t.getResultKg(), 0.02, "kG medido longe da física (0,063 V)");
    assertTrue(t.getResultKv() > 0.02 && t.getResultKv() < 0.06, "kV fora do esperado");
    assertTrue(t.getChosenMultiplier() >= 1.0, "deveria ter subido os ganhos");
  }
}
