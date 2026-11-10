import edu.wpi.first.networktables.*;
import java.io.*;
import java.util.*;

/**
 * Ponte NT4 de linha de comando para a bancada.
 *   list [prefixo]                 tópicos e valores atuais
 *   get  topico...                 lê valores
 *   set  topico valor              escreve (double, boolean ou string, conforme o tipo existente)
 *   rec  segundos arquivo.csv prefixo...   grava todas as mudanças (t_robô_s,topico,valor)
 * Servidor: -Dnt.server=IP (padrão roborio-9604-frc.local)
 */
public class NtBridge {
  static NetworkTableInstance inst;

  public static void main(String[] args) throws Exception {
    NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
    edu.wpi.first.util.WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
    for (String lib : new String[] {"wpiutil", "wpiutiljni", "wpinet", "wpinetjni", "ntcore", "ntcorejni"}) {
      System.loadLibrary(lib);
    }
    inst = NetworkTableInstance.create();
    inst.startClient4("ntbridge");
    inst.setServer(System.getProperty("nt.server", "roborio-9604-frc.local"));
    long t0 = System.currentTimeMillis();
    while (!inst.isConnected()) {
      if (System.currentTimeMillis() - t0 > 5000) {
        System.out.println("ERRO: não conectou ao robô em 5 s");
        System.exit(2);
      }
      Thread.sleep(50);
    }
    String cmd = args.length > 0 ? args[0] : "list";
    switch (cmd) {
      case "list" -> list(args.length > 1 ? args[1] : "/");
      case "get" -> get(Arrays.copyOfRange(args, 1, args.length));
      case "set" -> set(args[1], args[2]);
      case "rec" -> rec(Double.parseDouble(args[1]), args[2], Arrays.copyOfRange(args, 3, args.length));
      default -> System.out.println("comando desconhecido: " + cmd);
    }
    inst.close();
    System.exit(0);
  }

  static String fmt(NetworkTableValue v) {
    if (v == null || !v.isValid()) return "<sem valor>";
    Object o = v.getValue();
    if (o instanceof double[] d) return Arrays.toString(d);
    if (o instanceof boolean[] b) return Arrays.toString(b);
    if (o instanceof long[] l) return Arrays.toString(l);
    if (o instanceof String[] s) return Arrays.toString(s);
    if (o instanceof byte[] r) return "<" + r.length + " bytes>";
    return String.valueOf(o);
  }

  static void list(String prefix) throws Exception {
    MultiSubscriber ms = new MultiSubscriber(inst, new String[] {prefix});
    Thread.sleep(1500);
    TreeMap<String, String> out = new TreeMap<>();
    for (Topic t : inst.getTopics(prefix)) {
      GenericSubscriber s = t.genericSubscribe();
      Thread.sleep(5);
      out.put(t.getName(), t.getTypeString() + " = " + fmt(s.get()));
    }
    Thread.sleep(300);
    for (Topic t : inst.getTopics(prefix)) {
      GenericSubscriber s = t.genericSubscribe();
      out.put(t.getName(), t.getTypeString() + " = " + fmt(s.get()));
    }
    out.forEach((k, v) -> System.out.println(k + "  " + v));
    ms.close();
  }

  static void get(String[] topics) throws Exception {
    List<GenericSubscriber> subs = new ArrayList<>();
    for (String n : topics) subs.add(inst.getTopic(n).genericSubscribe());
    Thread.sleep(800);
    for (int i = 0; i < topics.length; i++) System.out.println(topics[i] + " = " + fmt(subs.get(i).get()));
  }

  static void set(String name, String value) throws Exception {
    Topic t = inst.getTopic(name);
    GenericSubscriber s = t.genericSubscribe();
    Thread.sleep(600);
    String type = t.getTypeString();
    GenericPublisher p;
    switch (type) {
      case "boolean" -> { p = t.genericPublish("boolean"); p.setBoolean(Boolean.parseBoolean(value)); }
      case "string" -> { p = t.genericPublish("string"); p.setString(value); }
      default -> { p = t.genericPublish("double"); p.setDouble(Double.parseDouble(value)); }
    }
    inst.flush();
    Thread.sleep(400);
    System.out.println(name + " (" + (type.isEmpty() ? "novo" : type) + ") -> " + value
        + "   lido de volta: " + fmt(s.get()));
    p.close();
  }

  static void rec(double secs, String file, String[] prefixes) throws Exception {
    NetworkTableListenerPoller poller = new NetworkTableListenerPoller(inst);
    MultiSubscriber ms = new MultiSubscriber(inst, prefixes, PubSubOption.sendAll(true));
    poller.addListener(prefixes, EnumSet.of(NetworkTableEvent.Kind.kValueAll, NetworkTableEvent.Kind.kImmediate));
    long end = System.currentTimeMillis() + (long) (secs * 1000);
    int n = 0;
    try (PrintWriter w = new PrintWriter(new FileWriter(file))) {
      w.println("t,topic,value");
      // Para antes do tempo se existir o arquivo indicado em -Dnt.stopfile (usado pelos testes
      // com parada automática)
      String stopFile = System.getProperty("nt.stopfile", "");
      while (System.currentTimeMillis() < end) {
        if (!stopFile.isEmpty() && new File(stopFile).exists()) break;
        for (NetworkTableEvent e : poller.readQueue()) {
          if (e.valueData == null) continue;
          NetworkTableValue v = e.valueData.value;
          String name = e.valueData.getTopic().getName();
          w.printf(Locale.ROOT, "%.4f,%s,%s%n", v.getServerTime() / 1e6, name,
              fmt(v).replace(",", ";"));
          n++;
        }
        Thread.sleep(5);
      }
    }
    ms.close();
    poller.close();
    System.out.println("gravadas " + n + " amostras em " + file);
  }
}
