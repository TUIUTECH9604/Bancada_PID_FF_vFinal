"""Mede o feedforward (kS, kG, kV, kA) a partir do log a 5 ms de movimentos NORMAIS (robusto.sh).

Não precisa de teste especial: usa os movimentos com perfil que o mecanismo já fez dentro do curso
seguro. Para cada amostra com o mecanismo andando:

    V(t − atraso) = kS·sinal(v) + kG·fator(θ) + kV·v + kA·a

com V = tensão pedida (o controlador compensa a bateria), v e a pela QUADRATURA (0,044°)
suavizadas (polinômio local de 2º grau), fator(θ) pelo GRAVIDADE do .conf. O atraso do
acionamento é procurado (0 a 30 ms). A regressão é robusta: descarta as amostras que destoam
(batidas na folga, trancos), em 3 passadas.

Limites: os dados são de malha fechada e a folga mistura o lado do motor com o do mecanismo —
os valores são um PONTO DE PARTIDA (a confirmar por bateria), não a resposta final.

uso: [MEC=<mecanismo>] python identifica.py logs/ARQ.wpilog [...] [--de S --ate S] [--vmin 20]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wpilog import read_wpilog  # noqa: E402
import mec  # noqa: E402

CFG = mec.carrega()


def savgol(x, janela, deriv, dt):
    """Derivada `deriv` de x por polinômio local de 2º grau (janela ímpar de amostras)."""
    m = janela // 2
    k = np.arange(-m, m + 1) * dt
    A = np.vander(k, 3, increasing=True)          # 1, k, k²
    coef = np.linalg.pinv(A)[deriv] * (1 if deriv < 2 else 2)
    return np.convolve(x, coef[::-1], mode="same")


def amostras(path, t0=None, t1=None):
    s = read_wpilog(path, "Mech5ms/")
    tv = CFG.canal5ms(s, "deg")
    if tv is None:
        return None
    t, deg = (np.asarray(x) for x in tv)

    def col(nome):
        tt, v = CFG.canal5ms(s, nome)
        v = np.asarray(v, float)
        return v if len(v) == len(t) else np.interp(t, tt, v)

    quad, volts, goal = col("quadDeg"), col("cmdVolts"), col("goalDeg")
    sel = np.ones(len(t), bool)
    if t0 is not None:
        sel &= t >= t0
    if t1 is not None:
        sel &= t <= t1
    # posição contínua: a quadratura (se estiver viva) ou o absoluto desembrulhado
    cont = np.concatenate([[0.0], np.cumsum(CFG.erro(deg[1:], deg[:-1]))]) + deg[0]
    if np.ptp(quad[sel]) < 0.2 * max(1e-9, np.ptp(cont[sel])):
        print("  (quadratura parada: usando o absoluto, mais ruidoso)")
        pos = cont
    else:
        pos = quad
    return t, pos, deg, volts, goal, sel


def identifica(arquivos, t0=None, t1=None, vmin=20.0, janela=11, atraso_ms=None):
    X, Y, info = [], [], []
    for f in arquivos:
        r = amostras(f, t0, t1)
        if r is None:
            print(f"{f}: sem /Mech5ms/{CFG.NOME}/")
            continue
        t, pos, deg, volts, goal, sel = r
        dt = float(np.median(np.diff(t)))
        # trechos contínuos (sem buracos no tempo) e com malha de posição ativa
        quebra = np.where(np.diff(t) > 3 * dt)[0]
        limites = np.concatenate([[0], quebra + 1, [len(t)]])
        for a, b in zip(limites[:-1], limites[1:]):
            if b - a < 3 * janela:
                continue
            p = pos[a:b]
            v = savgol(p, janela, 1, dt)
            ac = savgol(p, janela, 2, dt)
            ok = np.zeros(b - a, bool)
            m = janela // 2
            ok[m:-m] = True
            ok &= sel[a:b] & ~np.isnan(goal[a:b]) & (np.abs(v) >= vmin)
            info.append((t[a:b], v, ac, volts[a:b], deg[a:b], ok, dt))
    if not info:
        return None

    melhor = None
    dt0 = info[0][6]
    atrasos = range(0, 7) if atraso_ms is None else [int(round(atraso_ms / 1000 / dt0))]
    for atraso in atrasos:               # 0 a 30 ms, ou o dado
        Xs, Ys = [], []
        for tt, v, ac, V, d, ok, dt in info:
            idx = np.where(ok)[0]
            idx = idx[idx >= atraso]
            Vd = V[idx - atraso]          # a tensão de antes é a que move agora
            cols = [np.sign(v[idx])]
            if CFG.GRAVIDADE != "NONE":
                cols.append(CFG.fator_gravidade(d[idx]))
            cols += [v[idx], ac[idx]]
            Xs.append(np.column_stack(cols))
            Ys.append(Vd)
        Xa, Ya = np.vstack(Xs), np.concatenate(Ys)
        usa = np.ones(len(Ya), bool)
        for _ in range(3):               # regressão robusta: tira o que destoa
            coef, *_ = np.linalg.lstsq(Xa[usa], Ya[usa], rcond=None)
            res = Ya - Xa @ coef
            mad = np.median(np.abs(res[usa] - np.median(res[usa]))) * 1.4826
            usa = np.abs(res) <= 3 * max(mad, 1e-6)
        res_u = res[usa]
        r2 = 1 - np.var(res_u) / np.var(Ya[usa])
        # incerteza (erro padrão) de cada coeficiente
        sig2 = np.sum(res_u ** 2) / max(1, usa.sum() - Xa.shape[1])
        cov = sig2 * np.linalg.pinv(Xa[usa].T @ Xa[usa])
        ep = np.sqrt(np.diag(cov))
        cand = dict(atraso_ms=atraso * info[0][6] * 1000, coef=coef, ep=ep, r2=r2,
                    n=int(usa.sum()), descartadas=int((~usa).sum()), rms=float(np.std(res_u)))
        if melhor is None or cand["rms"] < melhor["rms"]:
            melhor = cand
    nomes = ["kS"] + (["kG"] if CFG.GRAVIDADE != "NONE" else []) + ["kV", "kA"]
    melhor["valores"] = dict(zip(nomes, melhor["coef"]))
    melhor["erros"] = dict(zip(nomes, melhor["ep"]))
    melhor.setdefault("valores", {}).setdefault("kG", 0.0)
    return melhor


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("arquivos", nargs="+")
    ap.add_argument("--de", type=float)
    ap.add_argument("--ate", type=float)
    ap.add_argument("--atraso", type=float, default=None,
                    help="atraso do acionamento fixo (ms); sem ele, procura de 0 a 30 ms")
    ap.add_argument("--vmin", type=float, default=20.0,
                    help="só amostras andando mais rápido que isto (unidade/s); padrão 20")
    a = ap.parse_args()
    r = identifica(a.arquivos, a.de, a.ate, a.vmin, atraso_ms=a.atraso)
    if r is None:
        sys.exit(1)
    v, e = r["valores"], r["erros"]
    u = CFG.UNIDADE
    print(f"MEC={CFG.arquivo}  gravidade {CFG.GRAVIDADE}  atraso {r['atraso_ms']:.0f} ms  "
          f"R² {r['r2']:.3f}  resíduo {r['rms']:.3f} V  amostras {r['n']} "
          f"(descartadas {r['descartadas']})")
    print(f"  kS = {v['kS']:.4f} V   (±{e['kS']:.4f})")
    if "kG" in e:
        print(f"  kG = {v['kG']:.4f} V   (±{e['kG']:.4f})")
    print(f"  kV = {v['kV']:.6f} V/({u}/s)  (±{e['kV']:.6f})"
          + (f"  = {v['kV'] * 6:.5f} V/RPM (unidade do código)" if CFG.CIRCULAR else ""))
    print(f"  kA = {v['kA']:.7f} V/({u}/s²) (±{e['kA']:.7f})")
