package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Detector de oscilação: dispara com o ciclo-limite (~11 Hz, medido na bancada em 2026-11-12) e
 * NÃO com a mão / um tranco / o operador mudando de sentido (3–5 Hz, medido em 2026-11-13 — a
 * versão antiga, "8 viradas em 1 s", disparava com a mão e travava o mecanismo).
 */
class MechanismOscDetectorTest {
  private static final double DT = 0.005; // laço de 5 ms
  private static final double SWING = 1.0;
  private static final double MAX_INTERVAL = 0.070;
  private static final int NEEDED = 8;

  /** Primeiro instante (s) em que dispara, ou NaN. Senoide de amplitude a(t) em volta de 270°. */
  private static double primeiroDisparo(double hz, double segundos,
      java.util.function.DoubleUnaryOperator amplitude) {
    MechanismOscDetector d = new MechanismOscDetector();
    for (int i = 0; i * DT <= segundos; i++) {
      double t = i * DT;
      double x = 270.0 + amplitude.applyAsDouble(t) * Math.sin(2 * Math.PI * hz * t);
      if (d.step(t, x, SWING, MAX_INTERVAL, NEEDED)) return t;
    }
    return Double.NaN;
  }

  @Test
  void cicloLimiteDe11HzDisparaRapido() {
    double t = primeiroDisparo(11.0, 3.0, s -> 3.5);
    assertFalse(Double.isNaN(t), "o ciclo-limite (11 Hz, ±3,5°) tem que disparar");
    assertTrue(t < 0.5, "tem que disparar em menos de 0,5 s (disparou em " + t + " s)");
  }

  @Test
  void maoBalancandoNaFolgaNaoDispara() {
    // medido: viradas a cada 95–150 ms (3,3–5,3 Hz), ±4° (folga) ou ±40° (forçando o motor)
    for (double hz : new double[] {2.0, 3.3, 4.3, 5.3, 6.0}) {
      for (double amp : new double[] {4.0, 40.0}) {
        assertTrue(Double.isNaN(primeiroDisparo(hz, 10.0, s -> amp)),
            "mão a " + hz + " Hz ±" + amp + "° não pode disparar");
      }
    }
  }

  @Test
  void trancoQueAmorteceNaoDispara() {
    // uma batida faz o ponteiro balançar rápido e parar: ~3 ciclos a 11 Hz morrendo
    assertTrue(Double.isNaN(primeiroDisparo(11.0, 3.0, s -> 5.0 * Math.exp(-s / 0.1))),
        "um tranco que amortece sozinho não pode disparar");
  }

  @Test
  void oscilacaoPequenaDemaisNaoConta() {
    // tremor de ±0,3° (menor que a histerese de 1°) não é oscilação perigosa
    assertTrue(Double.isNaN(primeiroDisparo(12.0, 3.0, s -> 0.3)));
  }
}
