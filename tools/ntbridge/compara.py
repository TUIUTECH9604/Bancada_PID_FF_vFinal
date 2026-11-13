"""Compara as configurações de uma bateria (bateria.sh) pelas gravações do Wi-Fi (mapa_ana).

Para decidir, prefira o log a 5 ms: python log_rodadas.py logs/ARQUIVO.wpilog (lição 19). Este
resumo serve para acompanhar a bateria sem baixar o log.

Genérico: MEC=<mecanismo> (mecanismos/<nome>.conf): a sequência de alvos é o ALVOS do .conf e
"grandes" = movimentos com passo ≥ GRANDE.
uso: [MEC=...] python compara.py pasta_da_bateria
"""
import glob
import os
import re
import subprocess
import sys

NB = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, NB)
import mec  # noqa: E402

CFG = mec.carrega()
OUT = sys.argv[1]
# Configurações = prefixos dos arquivos NOME_rN.csv, na ordem em que foram gravados
_arqs = sorted(glob.glob(os.path.join(OUT, "*_r*.csv")), key=os.path.getmtime)
NOMES = list(dict.fromkeys(os.path.basename(f).rsplit("_r", 1)[0] for f in _arqs))
RODADAS = sorted({int(os.path.basename(f).rsplit("_r", 1)[1].split(".")[0]) for f in _arqs})
SEQ = CFG.ALVOS

row_re = re.compile(r"\s*(-?\d+)\s+(sobe|desce)\s+\|\s+([\d.]+)\S*\s+([\d.]+)\S*\s+([\d.]+)Hz\s+([\d.]+)V"
                    r"\s+\|\s+([\d.]+)\S*\s+([-+\d.]+)")
sum_re = re.compile(r"pp medio ([\d.]+)\S* max ([\d.]+)\S* .*tremendo \(>0,5\S*\): (\d+) de (\d+) .*"
                    r"overshoot medio ([\d.]+)\S* max ([\d.]+)")


def analisa(csv):
    env = dict(os.environ, MEC=CFG.arquivo)
    txt = subprocess.run([sys.executable, "mapa_ana.py", csv], cwd=NB, capture_output=True,
                         text=True, encoding="utf-8", errors="replace", env=env).stdout
    rows = [(int(m.group(1)), float(m.group(3)), float(m.group(7)), float(m.group(8)))
            for m in row_re.finditer(txt)]
    return rows, sum_re.search(txt)


def passo(k):
    """Tamanho do movimento k (SEQ[k] → SEQ[k+1]); as linhas do mapa_ana começam no 2º alvo."""
    if k + 1 >= len(SEQ):
        return 0.0
    return abs(float(CFG.erro(SEQ[k + 1], SEQ[k])))


print(f"MEC={CFG.arquivo} ({CFG.UNIDADE}); grandes = passo >= {CFG.GRANDE:g}")
print(f"{'config':10} {'r':2} | {'ov med':>6} {'ov max':>6} | {'pp med':>6} {'pp max':>6} {'trem':>5} |"
      f" {'grandes':>7} {'pior':>12} | erro|med")
for n in NOMES:
    for r in RODADAS:
        f = os.path.join(OUT, f"{n}_r{r}.csv")
        if not os.path.exists(f):
            print(f"{n:10} {r:2} | (sem arquivo)")
            continue
        rows, s = analisa(f)
        if not s:
            print(f"{n:10} {r:2} | ABORTADO ou sem resumo ({len(rows)} movimentos)")
            continue
        grandes = [o for k, (ang, pp, o, e) in enumerate(rows) if passo(k) >= CFG.GRANDE]
        kp = max(range(len(rows)), key=lambda k: rows[k][2]) if rows else None
        pior = (f"{SEQ[kp]:g}>{SEQ[kp + 1]:g} {rows[kp][2]:.1f}"
                if kp is not None and kp + 1 < len(SEQ) else "-")
        errm = sum(abs(e) for a, pp, o, e in rows) / max(1, len(rows))
        gm = sum(grandes) / len(grandes) if grandes else float("nan")
        print(f"{n:10} {r:2} | {float(s.group(5)):6.1f} {float(s.group(6)):6.1f} | {float(s.group(1)):6.2f}"
              f" {float(s.group(2)):6.2f} {s.group(3):>2}/{s.group(4):<2} | {gm:7.2f} {pior:>12} | {errm:.2f}")
