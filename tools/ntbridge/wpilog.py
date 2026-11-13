"""Leitor mínimo de arquivos .wpilog (DataLog do WPILib), sem dependências além de numpy.

Formato (docs do WPILib, "DataLog file format"): cabeçalho "WPILOG" + versão + texto extra; depois
registros com um byte de bitfield (tamanhos do id, do payload e do timestamp), id da entrada,
tamanho, timestamp em µs e payload. Entrada 0 = controle (start/finish/metadata).

Uso (linha de comando): python wpilog.py arquivo.wpilog Mech5ms/

Uso (Python):
    from wpilog import read_wpilog
    series = read_wpilog("arquivo.wpilog", prefix="Mech5ms/")
    t, v = series["/Mech5ms/deg"]          # arrays numpy: tempo (s) e valores
"""
import struct
import sys

import numpy as np

_DOUBLE = struct.Struct("<d")
_INT64 = struct.Struct("<q")
_FLOAT = struct.Struct("<f")


def _uint(b, off, n):
    return int.from_bytes(b[off:off + n], "little"), off + n


def read_wpilog(path, prefix=""):
    """Devolve {nome: (t_segundos, valores)} das entradas numéricas que começam com prefix.

    O prefixo pode vir sem a barra inicial ("Mech5ms/"): o Git Bash transforma "/Mech5ms/" num
    caminho do Windows quando ele vem pela linha de comando.
    """
    if prefix and not prefix.startswith("/"):
        prefix = "/" + prefix
    with open(path, "rb") as f:
        b = f.read()
    if b[:6] != b"WPILOG":
        raise ValueError(f"{path}: não é um .wpilog")
    extra_len = struct.unpack_from("<I", b, 8)[0]
    off = 12 + extra_len
    entries = {}   # id -> (nome, tipo)
    data = {}      # nome -> ([t], [v])
    n = len(b)
    while off < n:
        bits = b[off]
        off += 1
        id_len = (bits & 0x3) + 1
        size_len = ((bits >> 2) & 0x3) + 1
        ts_len = ((bits >> 4) & 0x7) + 1
        if off + id_len + size_len + ts_len > n:
            break  # registro cortado no fim (log não fechado)
        eid, off = _uint(b, off, id_len)
        size, off = _uint(b, off, size_len)
        ts, off = _uint(b, off, ts_len)
        payload = b[off:off + size]
        off += size
        if len(payload) < size:
            break
        if eid == 0:
            if payload and payload[0] == 0:  # start
                p = 1
                new_id = struct.unpack_from("<I", payload, p)[0]
                p += 4
                ln = struct.unpack_from("<I", payload, p)[0]
                p += 4
                name = payload[p:p + ln].decode("utf-8", "replace")
                p += ln
                ln = struct.unpack_from("<I", payload, p)[0]
                p += 4
                typ = payload[p:p + ln].decode("utf-8", "replace")
                entries[new_id] = (name, typ)
            continue
        ent = entries.get(eid)
        if ent is None:
            continue
        name, typ = ent
        if not name.startswith(prefix):
            continue
        if typ == "double" and size == 8:
            v = _DOUBLE.unpack(payload)[0]
        elif typ == "int64" and size == 8:
            v = float(_INT64.unpack(payload)[0])
        elif typ == "float" and size == 4:
            v = _FLOAT.unpack(payload)[0]
        elif typ == "boolean" and size == 1:
            v = float(payload[0])
        else:
            continue
        lst = data.setdefault(name, ([], []))
        lst[0].append(ts / 1e6)
        lst[1].append(v)
    return {k: (np.array(t), np.array(v)) for k, (t, v) in data.items()}


if __name__ == "__main__":
    s = read_wpilog(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "")
    for k in sorted(s):
        t, v = s[k]
        print(f"{k:45s} {len(v):7d} amostras  {t[0]:9.3f}–{t[-1]:9.3f} s")
