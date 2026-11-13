"""Configuração do mecanismo para as análises em Python (mesmo arquivo que o mec.sh lê).

MEC=<nome> (variável de ambiente) escolhe mecanismos/<nome>.conf; padrão: bancada.

    import mec
    cfg = mec.carrega()          # cfg.NOME, cfg.ALVOS, cfg.CIRCULAR, ...
    e = cfg.erro(medido, alvo)   # erro pelo caminho mais curto se CIRCULAR, senão a diferença
    t, v = cfg.canal5ms(series, "deg")   # /Mech5ms/<NOME>/deg (ou o nome antigo /Mech5ms/deg)
"""
import os
import shlex
import sys

import numpy as np

# Console do Windows (cp1252) não tem →, ≥, ←...: troca por "?" em vez de derrubar o script
for _f in (sys.stdout, sys.stderr):
    try:
        _f.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass

_DIR = os.path.dirname(os.path.abspath(__file__))
# Nota de uma rodada (pontua.py): passa·passa_méd + grandes·passa_nos_grandes + max·passa_máx
# + erro·erro_final + chega·chegada(s) + tremor·tremor_méd + problema (rodada com oscilação ou que
# não chegou) + parada·(paradas que oscilaram por rodada). Mude por mecanismo com NOTA_PESOS.
PESOS_PADRAO = dict(passa=1.0, grandes=0.5, max=0.25, erro=2.0, chega=1.0, tremor=2.0,
                    problema=10.0, parada=10.0)


class Mecanismo:
    def __init__(self, nome_arquivo, valores):
        self.arquivo = nome_arquivo
        self.NOME = valores["NOME"]
        self.TELEM = valores["TELEM"]
        self.UNIDADE = valores.get("UNIDADE", "graus")
        self.CIRCULAR = valores.get("CIRCULAR", "1") == "1"
        self.GRAVIDADE = valores.get("GRAVIDADE", "COSINE").upper()
        self.ALVOS = [float(x) for x in valores["ALVOS"].split()]
        self.INICIO = float(valores.get("INICIO", self.ALVOS[0]))
        self.ESPERA = float(valores.get("ESPERA", "2.6"))
        self.GRANDE = float(valores.get("GRANDE", "60"))
        self.TOL = float(valores.get("TOL", "1.0"))
        self.PARADA_ALVO = float(valores["PARADA_ALVO"]) if "PARADA_ALVO" in valores else None
        self.PARADA_DE = [float(x) for x in valores.get("PARADA_DE", "").split()] or [self.INICIO]
        # Pesos da nota do pontua.py (o que importa mais neste mecanismo); faltando, o padrão
        self.PESOS = dict(PESOS_PADRAO)
        for kv in valores.get("NOTA_PESOS", "").split():
            k, v = kv.split("=", 1)
            if k not in PESOS_PADRAO:
                raise SystemExit(f"ERRO: NOTA_PESOS com chave desconhecida '{k}' "
                                 f"(conhecidas: {' '.join(PESOS_PADRAO)})")
            self.PESOS[k] = float(v)
        self.TUN = "/Tuning/" + self.NOME
        self.extra = valores  # todas as chaves do .conf (LQR_*, ATRASO_MS, ...)

    def erro(self, medido, alvo):
        d = np.asarray(medido, float) - np.asarray(alvo, float)
        return (d + 180.0) % 360.0 - 180.0 if self.CIRCULAR else d

    def fator_gravidade(self, pos):
        """Multiplica o kG (igual ao MechanismConfig.Gravity.factor): cos θ, 1 ou 0."""
        pos = np.asarray(pos, float)
        if self.GRAVIDADE == "COSINE":
            return np.cos(np.radians(pos))
        if self.GRAVIDADE == "CONSTANT":
            return np.ones_like(pos)
        return np.zeros_like(pos)

    def norm(self, x):
        """Valor para exibir (0-360 se circular)."""
        return x % 360.0 if self.CIRCULAR else x

    def prefixos5ms(self):
        # novo (um por mecanismo) e o antigo, de antes de 2026-11-13
        return ["/Mech5ms/" + self.NOME + "/", "/Mech5ms/"]

    def canal5ms(self, series, nome):
        """(t, v) do canal de 5 ms deste mecanismo, ou None."""
        for p in self.prefixos5ms():
            if p + nome in series:
                return series[p + nome]
        return None

    def ajuste(self, series, chave):
        """(t, v) de um ajuste ao vivo /Tuning/<NOME>/<chave> gravado no .wpilog, ou None."""
        alvo = self.TUN + "/" + chave
        for k in series:
            if k.endswith(alvo):
                return series[k]
        return None

    def ajustes(self, series):
        """{chave: (t, v)} de todos os ajustes ao vivo deste mecanismo no .wpilog."""
        out = {}
        for k, tv in series.items():
            i = k.find(self.TUN + "/")
            if i >= 0:
                out[k[i + len(self.TUN) + 1:]] = tv
        return out


def le_conf(caminho):
    valores = {}
    with open(caminho, encoding="utf-8") as f:
        for linha in f:
            linha = linha.strip()
            if not linha or linha.startswith("#") or "=" not in linha:
                continue
            k, v = linha.split("=", 1)
            partes = shlex.split(v, comments=True)
            valores[k.strip()] = " ".join(partes)
    return valores


def carrega(nome=None):
    nome = nome or os.environ.get("MEC", "bancada")
    caminho = os.path.join(_DIR, "mecanismos", nome + ".conf")
    if not os.path.exists(caminho):
        disp = sorted(x[:-5] for x in os.listdir(os.path.join(_DIR, "mecanismos")))
        raise SystemExit(f"ERRO: não achei {caminho} (MEC={nome}). Mecanismos: {' '.join(disp)}")
    return Mecanismo(nome, le_conf(caminho))
