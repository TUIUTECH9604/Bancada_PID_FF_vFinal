package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.MechanismConfig;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.SimTestHooks;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Locale;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * MODO DO ZERO: o kit só-feedforward num mecanismo SEM NENHUM valor (kG = kV = kA = kP = kI = kD =
 * 0), na física calibrada pela bancada real (2026-11-13). Tem que terminar e gravar
 * {@code build/sim_kit_zero.csv} (t, V, quadratura, ângulo a 5 ms) com os pulsos de aceleração:
 * o PC tira dali kS/kG/kV/kA ({@code python tools/ntbridge/identifica.py --csv ... --aberta}).
 */
class MechanismTunerZeroSimTest {

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
  void kitDoZeroComPulsos() throws IOException {
    MechanismConfig c = MechanismConfig.BANCADA.toBuilder("Zero")
        .feedforward(0.0, 0.0, 0.0, 0.0)
        .pid(0.0, 0.0, 0.0)
        .build();
    MechanismSubsystem d = new MechanismSubsystem(c);
    MechanismTuner t = new MechanismTuner(d);
    NetworkTableInstance.getDefault().getEntry("/Tuning/Zero/MechTuneSoFF").setDouble(1.0);
    t.initialize();
    int n = 0;
    try (PrintWriter w = new PrintWriter(new FileWriter("build/sim_kit_zero.csv"))) {
      w.println("t,test,V,quadDeg,deg");
      while (!t.isFinished() && n++ < 15000) {
        d.periodic();
        t.execute();
        for (int k = 0; k < 4; k++) {
          SimHooks.stepTiming(0.005);
          d.fastLoop();
          w.printf(Locale.ROOT, "%.5f,0,%.4f,%.4f,%.4f%n", Timer.getFPGATimestamp(),
              d.getCommandedVolts(), d.getQuadDeg(), d.getDeg());
        }
      }
    }
    t.end(false);
    System.out.printf(Locale.ROOT, "[sim zero] %.0f s: kS %.3f, kG %.3f, kV %.4f, sucesso %b%n",
        n * 0.02, t.getResultKs(), t.getResultKg(), t.getResultKv(), t.succeeded());
    assertTrue(t.succeeded(), "o kit do zero não terminou");
    assertEquals(0.052, t.getResultKg(), 0.02, "kG longe da física (0,052 V)");
  }
}
