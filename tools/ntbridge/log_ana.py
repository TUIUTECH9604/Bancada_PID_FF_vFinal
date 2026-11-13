"""Análise dos movimentos gravados a 5 ms no .wpilog do robô (/Mech5ms/<NOME>/, ver MechanismFastLog).

Genérico: o mecanismo vem de MEC=<nome> (mecanismos/<nome>.conf, padrão bancada).

Cada mudança do destino do perfil (goalDeg) é um movimento. Para cada um mede, com TODAS as
amostras de 5 ms (sem as perdas do Wi-Fi):
  - tempo até chegar (perfil terminado e |erro| < 1° por 0,25 s);
  - quanto passou do alvo (no sentido do movimento) e erro final (média dos últimos 0,3 s);
  - tremor parado (pico a pico no último 0,5 s) e a maior tensão pedida;
  - P, I e D no fim (quem está segurando o ponteiro).

uso: [MEC=<mecanismo>] python log_ana.py arquivo.wpilog [arquivo2.wpilog ...] [--de SEGUNDOS] [--ate SEGUNDOS]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wpilog import read_wpilog  # noqa: E402
import mec  # noqa: E402

CFG = mec.carrega()

TOL_DEG = CFG.TOL
TOL_SEC = 0.25


def wrap180(x):
    """Diferença no espaço do mecanismo: pelo caminho mais curto se CIRCULAR, senão a própria."""
    x = np.asarray(x, float)
    return (x + 180.0) % 360.0 - 180.0 if CFG.CIRCULAR else x


def analisa(path, t0=None, t1=None):
    s = read_wpilog(path, "Mech5ms/")
    if CFG.canal5ms(s, "deg") is None:
        print(f"== {os.path.basename(path)}: sem dados /Mech5ms/{CFG.NOME}/ (gravação a 5 ms "
              f"desligada? MEC={CFG.arquivo})")
        return []
    t, deg = CFG.canal5ms(s, "deg")

    def col(nome):
        tt, v = CFG.canal5ms(s, nome)
        return v if len(v) == len(t) else np.interp(t, tt, v)

    goal, prof_v, cmd = col("goalDeg"), col("profVelDegps"), col("cmdVolts")
    pv, iv, dv = col("pVolts"), col("iVolts"), col("dVolts")
    sel = np.ones(len(t), bool)
    if t0 is not None:
        sel &= t >= t0
    if t1 is not None:
        sel &= t <= t1
    idx = np.where(sel & ~np.isnan(goal))[0]
    if len(idx) == 0:
        print(f"== {os.path.basename(path)}: nenhum movimento com perfil no trecho")
        return []

    # quebra em movimentos: mudança do destino (> 0,2°), buraco no tempo (> 0,1 s) ou trecho sem
    # alvo (goalDeg NaN = sem malha de posição, ex.: comando terminou e o ponteiro ficou solto).
    # Cada movimento termina na última amostra COM alvo: antes, o trecho solto entrava no
    # movimento anterior (2026-11-12: o ponteiro escorregou 30° solto entre duas rodadas e
    # apareceu como "90>0 passou 30°").
    trechos = []
    ini = idx[0]
    for a, b in zip(idx[:-1], idx[1:]):
        if b != a + 1 or t[b] - t[a] > 0.1 or abs(wrap180(goal[b] - goal[a])) > 0.2:
            trechos.append((ini, a + 1))
            ini = b
    trechos.append((ini, idx[-1] + 1))

    print(f"== {os.path.basename(path)}  ({len(t)} amostras, dt médio "
          f"{np.median(np.diff(t)) * 1000:.1f} ms)")
    print(f" de>para    | chega   | passa  | erro f | tremor | V máx |  P     I     D (V, fim)"
          f"   [{CFG.UNIDADE}]")
    res = []
    anterior = None
    for a, b in trechos:
        if b - a < 40:  # < 0,2 s: pedido intermediário, não é um movimento completo
            continue
        g = goal[a]
        tt = t[a:b] - t[a]
        e = wrap180(deg[a:b] - g)  # positivo = além do alvo no sentido anti-horário
        inicio = wrap180(deg[a] - g)
        sentido = -np.sign(inicio) if abs(inicio) > 0.5 else 0.0
        # "parou" = depois da ÚLTIMA amostra com o perfil andando. Com a aproximação anti-folga o
        # perfil para um instante no ponto intermediário (além do alvo, de propósito) e volta: isso
        # não é passar do alvo. O sentido da chegada é o do fim do perfil, não o do início.
        andando = np.where(np.abs(prof_v[a:b]) >= 1e-6)[0]
        if len(andando):
            parou = np.arange(b - a) > andando[-1]
            sentido = float(np.sign(prof_v[a + andando[-1]]))
        else:
            parou = np.ones(b - a, bool)
        # chegada: perfil terminado e |erro| < TOL por TOL_SEC seguidos
        ok = parou & (np.abs(e) < TOL_DEG)
        chega = np.nan
        n_tol = int(TOL_SEC / 0.005)
        run = 0
        for k in range(len(ok)):
            run = run + 1 if ok[k] else 0
            if run >= n_tol:
                chega = tt[k - n_tol + 1]
                break
        passa = max(0.0, float(np.max(sentido * e[parou]))) if sentido and parou.any() else 0.0
        fim = tt >= tt[-1] - 0.3
        erro_f = float(np.mean(e[fim]))
        # tremor: pelo ERRO (não pelo ângulo, que salta de 359,9° para 0°) e só depois de chegar
        ini_tremor = tt[-1] - 0.5 if np.isnan(chega) else max(tt[-1] - 0.5, chega + TOL_SEC)
        ult = tt >= ini_tremor
        tremor = float(np.ptp(e[ult]))
        vmax = float(np.max(np.abs(cmd[a:b])))
        de = f"{anterior:5.0f}" if anterior is not None else "  ?  "
        print(f"{de}>{CFG.norm(g):5.0f} | {chega:5.2f} s | {passa:5.2f}  | {erro_f:+5.2f}  | "
              f"{tremor:5.2f}  | {vmax:5.2f} | {pv[b - 1]:+5.2f} {iv[b - 1]:+5.2f} {dv[b - 1]:+5.2f}")
        # (chegada s, passou, erro final, tremor, de, para, tamanho do passo)
        passo = abs(float(wrap180(CFG.norm(g) - anterior))) if anterior is not None else np.nan
        res.append((chega, passa, erro_f, tremor,
                    anterior if anterior is not None else np.nan, CFG.norm(g), passo))
        anterior = CFG.norm(g)
    if res:
        r = np.array(res, float)
        print(f"RESUMO: chega {np.nanmean(r[:, 0]):.2f} s (máx {np.nanmax(r[:, 0]):.2f}) | passa "
              f"{r[:, 1].mean():.2f} (máx {r[:, 1].max():.2f}) | erro final "
              f"{np.abs(r[:, 2]).mean():.2f} | tremor {r[:, 3].mean():.2f} (máx {r[:, 3].max():.2f})"
              f" | n={len(r)}" + (f" | NÃO chegou: {int(np.isnan(r[:, 0]).sum())}"
                                    if np.isnan(r[:, 0]).any() else ""))
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("arquivos", nargs="+")
    ap.add_argument("--de", type=float, default=None, help="só a partir deste tempo (s)")
    ap.add_argument("--ate", type=float, default=None, help="só até este tempo (s)")
    a = ap.parse_args()
    for f in a.arquivos:
        analisa(f, a.de, a.ate)
