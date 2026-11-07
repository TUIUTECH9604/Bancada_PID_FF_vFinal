package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.DoubleSupplier;

/**
 * Número ajustável ao vivo pelo Elastic (ou qualquer cliente NetworkTables).
 *
 * <p>Aparece em /Tuning/<chave>. No Elastic, arraste o tópico para a aba e use um widget
 * "Text Display" (editável) para digitar o novo valor com o robô ligado.
 *
 * <p>Com TUNING_ENABLED = false (use em competição), o valor padrão do código é usado
 * e nada é publicado — assim ninguém altera um ganho sem querer no meio da partida.
 */
public class TunableNumber implements DoubleSupplier {

  /** Desligue em competição. */
  public static final boolean TUNING_ENABLED = true;

  private static final String ROOT = "/Tuning/";

  private final double defaultValue;
  private final DoubleEntry entry; // null quando o tuning está desligado
  private double lastValue;

  public TunableNumber(String key, double defaultValue) {
    this.defaultValue = defaultValue;
    this.lastValue = defaultValue;

    if (TUNING_ENABLED) {
      entry = NetworkTableInstance.getDefault()
          .getDoubleTopic(ROOT + key)
          .getEntry(defaultValue);
      entry.set(defaultValue); // publica para o valor aparecer no Elastic
    } else {
      entry = null;
    }
  }

  /** Valor atual (o do dashboard, se o tuning estiver ligado). */
  public double get() {
    return entry != null ? entry.get(defaultValue) : defaultValue;
  }

  /**
   * Muda o valor pelo código (usado pelo autoajuste). Aparece no Elastic na hora e dispara o
   * hasChanged(). Sem efeito com TUNING_ENABLED = false.
   */
  public void set(double value) {
    if (entry != null) {
      entry.set(value);
    }
  }

  @Override
  public double getAsDouble() {
    return get();
  }

  /**
   * True uma vez sempre que o valor muda no dashboard. Útil para reconfigurar o
   * controlador só quando necessário (configurar um Spark a cada 20 ms trava o CAN).
   */
  public boolean hasChanged() {
    double value = get();
    if (value != lastValue) {
      lastValue = value;
      return true;
    }
    return false;
  }
}
