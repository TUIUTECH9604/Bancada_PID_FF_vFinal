"""Mede cada rodada do robusto.sh dentro de um .wpilog (a 5 ms), para qualquer mecanismo.

Separa as rodadas pelo RemoteCmd 7 do mecanismo (início/fim, > 40 s; trechos só com os alvos do
teste de parada — PARADA_ALVO/PARADA_DE — viram TESTES DE PARADA: paradas e disparos da proteção),
roda o log_ana em cada rodada
e imprime, por rodada:
  - os AJUSTES ao vivo que mudam entre as rodadas do arquivo (descobertos sozinho: tudo em
    /Tuning/<NOME>/ que não é igual em todas — numa bateria, é o que estava sendo testado);
  - passada média/máxima, passada média nos movimentos GRANDES (passo ≥ GRANDE do .conf), erro
    final, tempo de chegada e tremor;
  - os 3 piores movimentos e a bateria (mínima/média, no Spark e na roboRIO).

uso: [MEC=<mecanismo>] python log_rodadas.py logs/ARQUIVO.wpilog [...] [--csv saida.csv]
     (--csv grava uma linha por rodada, para comparar/pontuar depois)
"""
import argparse
import contextlib
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402
from wpilog import read_wpilog  # noqa: E402
import log_ana  # noqa: E402
import mec  # noqa: E402

CFG = mec.carrega()
# Ajustes que mudam sozinhos durante o teste (não são a "configuração" da rodada)
IGNORAR = {"RemoteCmd", "TestSetpointDeg", "OscReset", "ZeroNow"}


def valor_em(tv, t, padrao=np.nan):
    tt, vv = tv
    sel = tt <= t
    return float(vv[sel][-1]) if sel.any() else padrao


def _canal_telem(s, nome):
    for k in (CFG.TELEM + "/" + nome, "NT:" + CFG.TELEM + "/" + nome):
        if k in s:
            return s[k]
    return None


def _subidas(tv, a, b):
    """Quantas vezes o booleano (t, v) passou de falso para verdadeiro dentro de [a, b]."""
    if tv is None:
        return 0
    tt, vv = np.asarray(tv[0], float), np.asarray(tv[1], float)
    antes = vv[tt < a]
    ant = bool(antes[-1]) if len(antes) else False
    n = 0
    for v in vv[(tt >= a) & (tt <= b)]:
        if v and not ant:
            n += 1
        ant = bool(v)
    return n


_DIR = os.path.dirname(os.path.abspath(__file__))


def _chave_cache(path):
    """Muda se o log, o código da análise ou o .conf do mecanismo mudarem."""
    import hashlib
    h = hashlib.sha1()
    st = os.stat(path)
    h.update(f"{os.path.abspath(path)}|{st.st_size}|{st.st_mtime_ns}".encode())
    for f in ("log_rodadas.py", "log_ana.py", "wpilog.py", "mec.py",
              os.path.join("mecanismos", CFG.arquivo + ".conf")):
        with open(os.path.join(_DIR, f), "rb") as fh:
            h.update(fh.read())
    return h.hexdigest()


def segmentos(path):
    """Como _segmentos, com cache em logs/.cache (ler um .wpilog de centenas de MB leva minutos)."""
    import pickle
    pasta = os.path.join(_DIR, "logs", ".cache")
    arq = os.path.join(pasta, _chave_cache(path) + ".pkl")
    try:
        with open(arq, "rb") as f:
            return pickle.load(f)
    except (OSError, EOFError, pickle.UnpicklingError):
        pass
    res = _segmentos(path)
    try:
        os.makedirs(pasta, exist_ok=True)
        with open(arq, "wb") as f:
            pickle.dump(res, f)
    except OSError:
        pass
    return res


def _segmentos(path):
    """(rodadas do robusto, testes de parada) de um .wpilog.

    Cada trecho com RemoteCmd 7 é um teste. Se todos os alvos dele estão em PARADA_ALVO/PARADA_DE,
    é um teste de PARADA (testeparada.sh, ou as paradas da bateria): conta as paradas e quantas
    vezes a proteção do robô disparou (isOscTripped). Senão é uma rodada do robusto (> 40 s)."""
    s = read_wpilog(path, "")
    rc = CFG.ajuste(s, "RemoteCmd")
    if rc is None:
        print(f"== {os.path.basename(path)}: sem {CFG.TUN}/RemoteCmd no log (MEC={CFG.arquivo}?)")
        return [], []
    tr, r = rc
    trechos, ini = [], None
    for t, v in zip(tr, r):
        if v == 7 and ini is None:
            ini = t
        elif v != 7 and ini is not None:
            trechos.append((ini, t))
            ini = None
    sp = CFG.ajuste(s, "TestSetpointDeg")
    osc = _canal_telem(s, "isOscTripped")
    pontos_parada = ({round(CFG.PARADA_ALVO, 6)} | {round(x, 6) for x in CFG.PARADA_DE}
                     if CFG.PARADA_ALVO is not None else set())

    def alvos(a, b):
        if sp is None:
            return []
        tt, vv = np.asarray(sp[0], float), np.asarray(sp[1], float)
        return [valor_em(sp, a)] + list(vv[(tt > a) & (tt <= b)])

    runs, pars = [], []
    for a, b in trechos:
        al = alvos(a, b)
        if pontos_parada and al and all(round(x, 6) in pontos_parada for x in al):
            n_par = sum(1 for x in al[1:] if round(x, 6) == round(CFG.PARADA_ALVO, 6))
            if n_par:
                pars.append((a, b, n_par))
        elif b - a >= 40:
            runs.append((a, b))

    ajustes = {k: tv for k, tv in CFG.ajustes(s).items() if k not in IGNORAR}
    todos = runs + [(a, b) for a, b, _ in pars]
    config = {a: {k: valor_em(tv, a + 1.0) for k, tv in ajustes.items()} for a, _ in todos}
    variam = sorted(k for k in ajustes
                    if len({float(f"{c[k]:.6g}") for c in config.values() if not np.isnan(c[k])}) > 1)

    out = []
    for a, b in runs:
        c = config[a]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            res = log_ana.analisa(path, a, b)
        if not res:
            continue
        rr = np.array(res, float)
        grandes = rr[rr[:, 6] >= CFG.GRANDE]
        piores = sorted(res, key=lambda x: -x[1])[:3]
        bat = {}
        for nome in ("busVolts", "rioVolts"):
            tv = CFG.canal5ms(s, nome)
            if tv is not None:
                tb, vb = np.asarray(tv[0]), np.asarray(tv[1])
                sel = (tb >= a) & (tb <= b) & (vb > 1.0)  # > 1 V: o Spark manda 0 antes de ler
                if sel.any():
                    bat[nome] = (float(vb[sel].min()), float(vb[sel].mean()))
        # período do laço de 5 ms nesta rodada (2026-11-13: ficou em 25 ms sozinho e a medida
        # não valia; o relatório deixa de fora as rodadas com laço lento)
        laco = np.nan
        tv = CFG.canal5ms(s, "dtMs")
        if tv is not None:
            tl, vl = np.asarray(tv[0]), np.asarray(tv[1])
            sel = (tl >= a) & (tl <= b)
            if sel.any():
                laco = float(np.median(vl[sel]))
        out.append(dict(laco_ms=laco,
            arquivo=os.path.basename(path), t=a, config={k: c[k] for k in variam}, ajustes=dict(c),
            passa_med=float(rr[:, 1].mean()), passa_max=float(rr[:, 1].max()),
            grandes_med=float(grandes[:, 1].mean()) if len(grandes) else np.nan,
            erro=float(np.abs(rr[:, 2]).mean()), chega=float(np.nanmean(rr[:, 0])),
            nao_chegou=int(np.isnan(rr[:, 0]).sum()),
            tremor_med=float(rr[:, 3].mean()), tremor_max=float(rr[:, 3].max()),
            osc=_subidas(osc, a, b),
            n=len(rr), piores=[(p[4], p[5], p[1]) for p in piores], bat=bat))
    paradas = [dict(arquivo=os.path.basename(path), t=a,
                    config={k: config[a][k] for k in variam}, ajustes=dict(config[a]), paradas=n,
                    oscilou=_subidas(osc, a, b + 0.5)) for a, b, n in pars]
    return out, paradas


def rodadas(path):
    """Só as rodadas do robusto (ver segmentos)."""
    return segmentos(path)[0]


def fmt(x):
    return f"{x:g}"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("arquivos", nargs="+")
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()
    todas = []
    for f in args.arquivos:
        rs, ps = segmentos(f)
        todas += rs
        for d in ps:
            cfg_txt = " ".join(f"{k}={fmt(v)}" for k, v in d["config"].items()) or "(igual)"
            print(f"{d['arquivo'][4:19]} t={d['t']:6.0f} {cfg_txt} | PARADAS em "
                  f"{fmt(CFG.PARADA_ALVO)}: oscilou {d['oscilou']} de {d['paradas']}")
        for d in rs:
            cfg_txt = " ".join(f"{k}={fmt(v)}" for k, v in d["config"].items()) or "(igual)"
            piores = " ".join(f"{fmt(de)}>{fmt(para)} {p:.1f}" for de, para, p in d["piores"])
            bat = " | ".join(f"{k[:3]} mín {v[0]:.2f} méd {v[1]:.2f} V" for k, v in d["bat"].items())
            print(f"{d['arquivo'][4:19]} t={d['t']:6.0f} {cfg_txt} | passa méd {d['passa_med']:.2f}"
                  f" máx {d['passa_max']:.2f} | grandes {d['grandes_med']:.2f} | erro {d['erro']:.2f}"
                  f" | chega {d['chega']:.2f}s"
                  + (f" (NÃO chegou {d['nao_chegou']})" if d["nao_chegou"] else "")
                  + (f" | PROTEÇÃO DISPAROU {d['osc']}x" if d["osc"] else "")
                  + (f" | LAÇO LENTO {d['laco_ms']:.1f} ms (não vale)" if d.get("laco_ms", 0) > 8 else "")
                  + f" | tremor {d['tremor_med']:.2f}/{d['tremor_max']:.2f} | piores: {piores}"
                  + (f" | {bat}" if bat else "") + f" | n={d['n']}")
    if args.csv and todas:
        chaves = sorted({k for d in todas for k in d["config"]})
        with open(args.csv, "w", encoding="utf-8") as w:
            w.write(",".join(["arquivo", "t"] + chaves + ["passa_med", "passa_max", "grandes_med",
                                                          "erro", "chega", "nao_chegou",
                                                          "tremor_med", "tremor_max", "n"]) + "\n")
            for d in todas:
                w.write(",".join([d["arquivo"], f"{d['t']:.1f}"]
                                 + [fmt(d["config"].get(k, np.nan)) for k in chaves]
                                 + [f"{d[k]:.4f}" for k in ("passa_med", "passa_max",
                                                            "grandes_med", "erro", "chega")]
                                 + [str(d["nao_chegou"]), f"{d['tremor_med']:.4f}",
                                    f"{d['tremor_max']:.4f}", str(d["n"])]) + "\n")
        print(f"(gravado {args.csv})")
