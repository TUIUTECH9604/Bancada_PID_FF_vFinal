"""Bateria do robô pelos logs do Driver Station (.dslog), minuto a minuto: mediana e menor valor.

O Driver Station grava sozinho, a cada 20 ms, a tensão da bateria que o robô informa (mesmo sem o
código do robô gravar nada). Formato v4: cabeçalho de 20 bytes; cada registro tem 47 bytes (com a
placa de distribuição desta bancada) e a bateria nos bytes 2-3 (big-endian, /256 = volts).

uso: python dslog_bateria.py [AAAA_MM_DD] [pasta]
     (padrão: hoje e C:/Users/Public/Documents/FRC/Log Files/DSLogs)
"""
import datetime as dt
import glob
import os
import re
import struct
import sys

import numpy as np

DIA = sys.argv[1] if len(sys.argv) > 1 else dt.date.today().strftime("%Y_%m_%d")
PASTA = sys.argv[2] if len(sys.argv) > 2 else "C:/Users/Public/Documents/FRC/Log Files/DSLogs"


def tamanho_registro(b):
    """Acha o tamanho do registro: o que dá tensões plausíveis (6-14 V) quase sempre."""
    # 47 primeiro (o desta bancada); amostras SEGUIDAS: a bateria muda pouco a cada 20 ms
    for L in [47] + [x for x in range(10, 80) if x != 47]:
        n = (len(b) - 20) // L
        if n < 200:
            continue
        v = np.array([struct.unpack(">H", b[20 + i * L + 2:20 + i * L + 4])[0] / 256
                      for i in range(min(n, 3000))])
        par = (v[:-1] > 6) & (v[:-1] < 14) & (v[1:] > 6) & (v[1:] < 14)
        if par.sum() > 100 and (np.abs(np.diff(v))[par] < 0.5).mean() > 0.98:
            return L
    return None


for f in sorted(glob.glob(os.path.join(PASTA, DIA + "*.dslog"))):
    b = open(f, "rb").read()
    if len(b) < 2000 or struct.unpack(">i", b[:4])[0] != 4:
        continue
    L = tamanho_registro(b)
    if L is None:
        print(f"{os.path.basename(f)}: formato não reconhecido")
        continue
    m = re.search(r"(\d+)_(\d+)_(\d+) (\d+)_(\d+)_(\d+)", os.path.basename(f))
    t0 = dt.datetime(*map(int, m.groups()))
    n = (len(b) - 20) // L
    a = np.frombuffer(b[20:20 + n * L], dtype=np.uint8).reshape(n, L)
    v = (a[:, 2].astype(int) * 256 + a[:, 3]) / 256
    for k in range(0, n, 3000):  # 3000 registros de 20 ms = 1 min
        seg = v[k:k + 3000]
        seg = seg[seg > 1.0]  # 0 = robô desconectado
        if len(seg):
            t = t0 + dt.timedelta(seconds=k * 0.02)
            print(f"{t:%H:%M}  mediana {np.median(seg):5.2f} V  menor {seg.min():5.2f} V")
