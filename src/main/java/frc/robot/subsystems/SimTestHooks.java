package frc.robot.subsystems;

/**
 * Ganchos para testes de simulação em outros pacotes (o MechanismSim é do pacote subsystems).
 * Chamar ANTES de criar o MechanismSubsystem; {@link #reset()} no fim do teste.
 */
public final class SimTestHooks {
  private SimTestHooks() {}

  /** Física sem gravidade (giratório na horizontal). */
  public static void semGravidade() {
    MechanismSim.testKgVolts = 0.0;
  }

  /** Batentes físicos simulados {mínimo, máximo} (graus, referencial do braço: pendurado = −90°). */
  public static void batentes(double minDeg, double maxDeg) {
    MechanismSim.testStopsDeg = new double[] {minDeg, maxDeg};
  }

  public static void reset() {
    MechanismSim.testKgVolts = Double.NaN;
    MechanismSim.testStopsDeg = null;
  }
}
