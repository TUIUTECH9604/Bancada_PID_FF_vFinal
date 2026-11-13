package frc.robot.subsystems;

/**
 * Detector de oscilação RÁPIDA (o ciclo-limite que o controle sustenta), separado de movimento
 * lento de vai-e-vem (a mão, um tranco no robô, o operador mudando de sentido).
 *
 * <p>Conta as viradas de sentido maiores que {@code swing} (histerese: o mecanismo tem que voltar
 * pelo menos isso desde o último extremo). Uma virada só entra na sequência se vier até
 * {@code maxIntervalSec} depois da anterior; se demorar mais, a sequência recomeça. Dispara com
 * {@code needed} viradas rápidas SEGUIDAS.
 *
 * <p>Medido na bancada (log a 5 ms): ciclo-limite em 270° = viradas a cada 43–48 ms (~11 Hz);
 * a mão balançando o ponteiro na folga, ou forçando contra o motor = 95–150 ms (3–5 Hz). Com 70 ms
 * e 8 seguidas, o ciclo-limite dispara em ~0,4 s e a mão nunca.
 */
final class MechanismOscDetector {
  private double extreme = Double.NaN;
  private int dir = 0;
  private double lastReversal = Double.NEGATIVE_INFINITY;
  private int chain = 0;

  void reset() {
    extreme = Double.NaN;
    dir = 0;
    lastReversal = Double.NEGATIVE_INFINITY;
    chain = 0;
  }

  /** Viradas rápidas seguidas até agora. */
  int chain() {
    return chain;
  }

  /** Tempo (s) da última virada de sentido (−∞ se nenhuma desde o reset). */
  double lastReversal() {
    return lastReversal;
  }

  /**
   * @param now tempo (s)
   * @param x posição contínua (sem dar a volta em 360°)
   * @return true quando a sequência de viradas rápidas chega a {@code needed}
   */
  boolean step(double now, double x, double swing, double maxIntervalSec, int needed) {
    if (Double.isNaN(extreme)) {
      extreme = x;
      dir = 0;
      return false;
    }
    if (dir == 0) {
      if (Math.abs(x - extreme) >= swing) {
        dir = x > extreme ? 1 : -1;
        extreme = x;
      }
      return false;
    }
    if (dir * (x - extreme) > 0) {
      extreme = x; // continua no mesmo sentido: novo extremo
      return false;
    }
    if (Math.abs(x - extreme) < swing) return false;
    // virou de sentido mais que swing
    chain = now - lastReversal <= maxIntervalSec ? chain + 1 : 1;
    lastReversal = now;
    dir = -dir;
    extreme = x;
    return chain >= needed;
  }
}
