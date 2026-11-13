package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * O mesmo código de controle com OUTRO mecanismo, só trocando a configuração: uma "torre"
 * giratória na horizontal (sem gravidade), com nome e ganhos próprios. Prova que o padrão de
 * teste serve para mais de um mecanismo.
 */
class MechanismConfigSimTest {

  private static final MechanismConfig TORRE = MechanismConfig.BANCADA.toBuilder("Torre")
      .gravity(MechanismConfig.Gravity.NONE)
      .feedforward(0.0, 0.0, 0.0318, 0.00005)
      .build();

  private MechanismSubsystem mec;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    MechanismSim.testKgVolts = 0.0; // física sem gravidade
    mec = new MechanismSubsystem(TORRE);
  }

  @AfterEach
  void tearDown() {
    MechanismSim.testKgVolts = Double.NaN;
    SimHooks.resumeTiming();
  }

  private static double erro(double alvo, double deg) {
    return MathUtil.inputModulus(deg - alvo, -180.0, 180.0);
  }

  @Test
  void torreSemGravidadeChegaNosAlvos() {
    // números ajustáveis na pasta do mecanismo, com os padrões da configuração
    double kp = NetworkTableInstance.getDefault().getEntry("/Tuning/Torre/kP").getDouble(-1);
    assertEquals(TORRE.kP, kp, 1e-12, "/Tuning/Torre/kP deveria existir com o padrão da config");
    assertEquals(0.0, mec.gravityFFVolts(0.0), 1e-12, "sem gravidade, kG não entra");

    mec.tunerProfileStart();
    for (double alvo : new double[] {0.0, 120.0, 300.0, 45.0}) {
      for (int i = 0; i < 400; i++) { // 2 s
        if (i % 4 == 0) {
          mec.tunerProfiled(alvo);
          mec.periodic();
        }
        SimHooks.stepTiming(0.005);
        mec.fastLoop();
      }
      System.out.printf("[sim torre] alvo %.0f° → %.2f°%n", alvo, mec.getDeg());
      assertEquals(0.0, erro(alvo, mec.getDeg()), 1.0, "torre não chegou em " + alvo);
    }
  }
}
