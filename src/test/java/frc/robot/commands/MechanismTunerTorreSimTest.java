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
 * O kit de ajuste automático numa torre sem gravidade (física e configuração sem kG). Classe
 * separada: cada classe de teste roda num processo (dois SparkMax com o mesmo CAN ID não podem).
 */
class MechanismTunerTorreSimTest {

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
    MechanismTuner t = new MechanismTuner(d);
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
  void torreSemGravidade() {
    SimTestHooks.semGravidade();
    MechanismConfig c = MechanismConfig.BANCADA.toBuilder("TunerTorre")
        .gravity(MechanismConfig.Gravity.NONE)
        .feedforward(0.0, 0.0, 0.0318, 0.0)
        .build();
    MechanismSubsystem d = new MechanismSubsystem(c);
    MechanismTuner t = roda(d);
    assertTrue(t.succeeded(), "o ajuste da torre não terminou com sucesso");
    assertEquals(0.0, t.getResultKg(), 1e-12, "sem gravidade, kG não é medido");
  }
}
