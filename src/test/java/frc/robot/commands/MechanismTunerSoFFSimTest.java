package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.MechanismConfig;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.SimTestHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Modo "só feedforward" do kit (MechTuneSoFF = 1), usado pelo assistente autoajuste.sh: mede
 * kS/kG/kV, publica em /MechTune/Resultado/... e NÃO aplica nada ao vivo nem testa ganhos.
 */
class MechanismTunerSoFFSimTest {

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

  @Test
  void soMedeOFeedforwardENaoMexeEmNada() {
    MechanismConfig c = MechanismConfig.BANCADA.toBuilder("SoFF")
        .feedforward(0.0, 0.0, 0.0318, 0.00005) // kG errado de propósito
        .build();
    MechanismSubsystem d = new MechanismSubsystem(c);
    // como no robô: o kit é criado antes (ao ligar) e o modo é ligado depois, ao vivo
    MechanismTuner t = new MechanismTuner(d);
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getEntry("/Tuning/SoFF/MechTuneSoFF").setDouble(1.0);
    d.periodic();

    MechanismTunerSimTest.roda(d, t);
    double kgPub = nt.getEntry("/MechTune/Resultado/kG").getDouble(Double.NaN);
    double kvPub = nt.getEntry("/MechTune/Resultado/kV").getDouble(Double.NaN);
    System.out.printf("[sim tuner só FF] publicado kG %.3f kV %.4f | ao vivo kG %.3f kP %.4f%n",
        kgPub, kvPub, d.gravityFFVolts(0.0), d.getKP());

    assertTrue(t.succeeded(), "o modo só feedforward não terminou com sucesso");
    assertTrue(Double.isNaN(t.getChosenMultiplier()), "não deveria ter testado ganhos");
    assertEquals(0.063, kgPub, 0.02, "kG publicado longe da física (0,063 V)");
    assertTrue(kvPub > 0.02 && kvPub < 0.06, "kV publicado fora do esperado");
    assertEquals(0.0, d.gravityFFVolts(0.0), 1e-12, "não pode aplicar o kG ao vivo");
    assertEquals(c.kP, d.getKP(), 1e-12, "não pode mexer nos ganhos");
  }
}
