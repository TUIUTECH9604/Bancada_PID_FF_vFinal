"""Nota de cada configuração de uma bateria, pelo log a 5 ms, e a recomendação.

Agrupa as rodadas pelos ajustes que mudaram (log_rodadas.py descobre quais) e dá uma NOTA (menor =
melhor) a cada rodada, com os pesos NOTA_PESOS do .conf do mecanismo (padrão em mec.py):

    nota = passa·passa_méd + grandes·passa_nos_grandes + max·passa_máx + erro·erro_final
           + chega·chegada(s) + tremor·tremor_méd + problema (se oscilou/não chegou)

Pesos padrão (1; 0,5; 0,25; 2; 1; 2; 10): passar do alvo é o que mais incomoda (e o pior caso
conta); o erro final pesa porque é a precisão; o tremor parado pesa porque desgasta e é sinal de
pouca margem. A nota da configuração é a MÉDIA das rodadas (e mostra a pior, para ver a
repetibilidade) + parada × (paradas no PARADA_ALVO que oscilaram, por rodada) — os testes de
parada que a bateria faz depois de cada rodada (PARADAS_POR_RODADA no .conf).

Recomendação: a menor nota entre as que NÃO oscilaram (nem no robusto, nem paradas). Se outra
estiver a menos de EMPATE (10%) e for mais CONSERVADORA (ganhos menores), recomenda a conservadora:
diferença menor que o ruído entre rodadas não justifica mais ganho (menos margem de estabilidade).
Uma pessoa aprova antes de gravar no código.

uso: [MEC=...] python pontua.py logs/ARQ.wpilog [...] [--empate 0.10]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402
import log_rodadas  # noqa: E402

CFG = log_rodadas.CFG
P = CFG.PESOS
# Ajustes que aumentam a "agressividade" (para escolher o conservador no empate)
AGRESSIVOS = ("kP", "kI", "kD", "ProfMaxVelDegps", "ProfMaxAccelDegps2")


def nota(d):
    prob = d["nao_chegou"] > 0 or d["tremor_max"] > 2.0 or d.get("osc", 0) > 0
    g = d["grandes_med"] if not np.isnan(d["grandes_med"]) else d["passa_med"]
    return (P["passa"] * d["passa_med"] + P["grandes"] * g + P["max"] * d["passa_max"]
            + P["erro"] * d["erro"] + P["chega"] * d["chega"] + P["tremor"] * d["tremor_med"]
            + (P["problema"] if prob else 0.0)), prob


def _chave(d):
    # 6 algarismos significativos (não casas decimais: kD ~1e-5 perderia a diferença)
    return tuple(sorted((k, float(f"{v:.6g}")) for k, v in d["config"].items()))


def agrupa(rodadas, paradas=()):
    """{chave: {"rod": [rodadas do robusto], "par": [testes de parada]}}"""
    grupos = {}
    for d in rodadas:
        grupos.setdefault(_chave(d), {"rod": [], "par": []})["rod"].append(d)
    for d in paradas:
        if _chave(d) in grupos:  # parada sem rodada do robusto da mesma config: não pontua
            grupos[_chave(d)]["par"].append(d)
    return grupos


def agressividade(chave):
    return sum(v for k, v in chave if k in AGRESSIVOS and not np.isnan(v))


def recomenda(grupos, empate=0.10):
    tab = []
    for chave, g in grupos.items():
        rs, ps = g["rod"], g["par"]
        ns = [nota(d) for d in rs]
        n_par = sum(d["paradas"] for d in ps)
        osc_par = sum(d["oscilou"] for d in ps)
        pen_par = P["parada"] * osc_par / len(rs)
        tab.append(dict(chave=chave, n=len(rs), nota=float(np.mean([x[0] for x in ns])) + pen_par,
                        pior=float(max(x[0] for x in ns)) + pen_par,
                        problemas=sum(x[1] for x in ns) + osc_par,
                        paradas=n_par, osc_paradas=osc_par,
                        passa=float(np.mean([d["passa_med"] for d in rs])),
                        grandes=float(np.nanmean([d["grandes_med"] for d in rs])),
                        maxp=float(max(d["passa_max"] for d in rs)),
                        erro=float(np.mean([d["erro"] for d in rs])),
                        chega=float(np.mean([d["chega"] for d in rs]))))
    tab.sort(key=lambda x: x["nota"])
    boas = [t for t in tab if t["problemas"] == 0]
    if not tab:
        return tab, None, "nenhuma"
    if not boas:
        return tab, tab[0], "TODAS oscilaram ou não chegaram: menor nota, NÃO use sem investigar"
    escolhida, motivo = boas[0], "menor nota"
    for alt in boas[1:]:
        if (alt["nota"] <= boas[0]["nota"] * (1 + empate)
                and agressividade(alt["chave"]) < agressividade(escolhida["chave"])):
            escolhida, motivo = alt, (f"empate (< {empate:.0%}) com a melhor nota: fica a "
                                      f"mais conservadora")
    return tab, escolhida, motivo


def paradas_txt(t):
    return f"{t['osc_paradas']}/{t['paradas']}" if t["paradas"] else "-"


def texto(chave):
    return " ".join(f"{k}={v:g}" for k, v in chave) or "(sem diferença)"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("arquivos", nargs="+")
    ap.add_argument("--empate", type=float, default=0.10)
    a = ap.parse_args()
    todas, pars = [], []
    for f in a.arquivos:
        r, p = log_rodadas.segmentos(f)
        todas += r
        pars += p
    if not todas:
        sys.exit("nenhuma rodada do robusto encontrada")
    tab, esc, motivo = recomenda(agrupa(todas, pars), a.empate)
    print(f"pesos: {' '.join(f'{k}={v:g}' for k, v in P.items())}")
    print(f"{'configuração':44} {'n':>2} | {'nota':>5} {'pior':>5} | passa grandes  máx | erro chega"
          f" | paradas osc")
    for t in tab:
        marca = " <==" if t is esc else ""
        prob = f"  ({t['problemas']} com oscilação/sem chegar)" if t["problemas"] else ""
        print(f"{texto(t['chave'])[:44]:44} {t['n']:>2} | {t['nota']:5.2f} {t['pior']:5.2f} | "
              f"{t['passa']:5.2f} {t['grandes']:7.2f} {t['maxp']:4.1f} | {t['erro']:4.2f} "
              f"{t['chega']:5.2f} | {paradas_txt(t):>7}{marca}{prob}")
    print(f"RECOMENDAÇÃO: {texto(esc['chave'])}  ({motivo}). Uma pessoa aprova antes de gravar.")
    empatadas = [t for t in tab if t is not esc and t["problemas"] == 0
                 and t["nota"] <= esc["nota"] * (1 + a.empate)]
    if empatadas:
        print("  EMPATE TÉCNICO (diferença < ruído entre rodadas) com: "
              + "; ".join(texto(t["chave"]) for t in empatadas)
              + " — qualquer uma serve; prefira a já testada/gravada ou a mais conservadora.")
