package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.io.File;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** A gravação a 5 ms grava UMA amostra por ciclo de 5 ms, e só com o robô ativo. */
class MechanismFastLogSimTest {

  private MechanismSubsystem mech;
  private File file;

  @BeforeEach
  void setUp() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    mech = new MechanismSubsystem();
    // Fica em build/ de propósito: serve para testar o leitor Python (tools/ntbridge/wpilog.py)
    file = new File("build/mech5ms_teste.wpilog");
    file.getParentFile().mkdirs();
    file.delete();
  }

  @AfterEach
  void tearDown() {
    SimHooks.resumeTiming();
  }

  @Test
  void gravaCadaCicloDe5ms() throws Exception {
    DataLogWriter log = new DataLogWriter(file.getAbsolutePath());
    mech.enableFastLog(new MechanismFastLog(log, "/u/logs", "Dial")); // "pendrive": liga sozinha
    mech.tunerProfileStart();
    for (int i = 0; i < 400; i++) { // 2 s a 5 ms: 0° no 1º segundo, 90° no 2º
      double alvo = i < 200 ? 0.0 : 90.0;
      if (i % 4 == 0) {
        mech.tunerProfiled(alvo);
        mech.periodic();
      }
      SimHooks.stepTiming(0.005);
      mech.fastLoop();
    }
    log.close();

    DataLogReader reader = new DataLogReader(file.getAbsolutePath());
    assertTrue(reader.isValid(), "arquivo .wpilog inválido");
    Map<Integer, String> nomes = new HashMap<>();
    Map<String, Integer> contagem = new HashMap<>();
    double ultimoDeg = Double.NaN;
    for (DataLogRecord r : reader) {
      if (r.isStart()) {
        DataLogRecord.StartRecordData st = r.getStartData();
        nomes.put(st.entry, st.name);
      } else if (!r.isControl()) {
        String n = nomes.get(r.getEntry());
        contagem.merge(n, 1, Integer::sum);
        if ("/Mech5ms/Dial/deg".equals(n)) ultimoDeg = r.getDouble();
      }
    }
    System.out.printf("[sim 5ms] %d amostras de deg, %d de cmdVolts, último ângulo %.1f°%n",
        contagem.getOrDefault("/Mech5ms/Dial/deg", 0), contagem.getOrDefault("/Mech5ms/Dial/cmdVolts", 0),
        ultimoDeg);
    assertEquals(400, contagem.getOrDefault("/Mech5ms/Dial/deg", 0), "uma amostra por ciclo de 5 ms");
    assertEquals(400, contagem.getOrDefault("/Mech5ms/Dial/pVolts", 0));
    assertEquals(90.0, ultimoDeg, 2.0, "o ponteiro deveria ter chegado em 90°");
  }
}
