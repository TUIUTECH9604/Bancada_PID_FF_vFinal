"""Parte de CÁLCULO do assistente de ajuste (o autoajuste.sh cuida do robô). Guarda tudo em
<pasta>/estado.json e escreve <pasta>/RELATORIO.md.

subcomandos (MEC=<mecanismo> escolhe o .conf):
  atual     <pasta> <saida do 'nt.sh get'>   guarda os valores ao vivo de partida
  ff        <pasta> <saida do 'nt.sh get'>   guarda kS/kG/kV medidos pelo kit (só feedforward)
  modelo    <pasta> <log.wpilog...>          identifica kV/kA no log a 5 ms e calcula kP/kD (LQR)
  campanha  <pasta>                          escreve <pasta>/campanha_auto.sh (atual × modelo × ...)
  relatorio <pasta> <log.wpilog...>          nota de cada configuração + recomendação
  campanha2 <pasta>                          2ª volta: se a melhor for a de ganho mais alto (ou mais
                                             baixo) testado, escreve campanha_volta2.sh com mais 2
                                             degraus para aquele lado (sai com 2 se não precisa)
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402
import mec  # noqa: E402

CFG = mec.carrega()
# Ajustes que o assistente lê/varia (os nomes do MechanismSubsystem, /Tuning/<NOME>/...)
CHAVES = ["kG", "kS", "kA", "kP", "kI", "kD", "IZoneDeg", "IDeadbandDeg", "DFilterSec",
          "DParadoFator", "AntiFolga", "ProfMaxVelDegps", "ProfMaxAccelDegps2", "VelkV"]


def carrega_estado(pasta):
    p = os.path.join(pasta, "estado.json")
    return json.load(open(p, encoding="utf-8")) if os.path.exists(p) else {}


def salva_estado(pasta, e):
    with open(os.path.join(pasta, "estado.json"), "w", encoding="utf-8") as f:
        json.dump(e, f, indent=2, ensure_ascii=False)


def le_get(texto):
    """'topico = valor' (saída do nt.sh get) → {última parte do tópico: float}."""
    out = {}
    for linha in texto.splitlines():
        if " = " in linha:
            k, v = linha.split(" = ", 1)
            try:
                out[k.strip().rsplit("/", 1)[-1]] = float(v.strip())
            except ValueError:
                pass
    return out


def g(x):
    return f"{x:.6g}"


def cmd_atual(pasta, arq):
    e = carrega_estado(pasta)
    e["atual"] = {k: v for k, v in le_get(open(arq, encoding="utf-8").read()).items() if k in CHAVES}
    salva_estado(pasta, e)
    print("valores de partida:", " ".join(f"{k}={g(v)}" for k, v in e["atual"].items()))


def cmd_ff(pasta, arq):
    e = carrega_estado(pasta)
    r = le_get(open(arq, encoding="utf-8").read())
    e["ff"] = {k: r.get(k, float("nan")) for k in ("kS", "kG", "kV", "erroV", "amostras")}
    salva_estado(pasta, e)
    print("feedforward pelo kit:", " ".join(f"{k}={g(v)}" for k, v in e["ff"].items()))


def cmd_modelo(pasta, logs):
    import identifica
    import ganhos
    e = carrega_estado(pasta)
    atraso = float(CFG.extra.get("ATRASO_MS", 10.0))
    r = identifica.identifica(logs, atraso_ms=atraso)
    if r is None:
        sys.exit("sem dados a 5 ms para identificar")
    v = r["valores"]
    if v["kV"] <= 0 or v["kA"] <= 0:
        sys.exit(f"identificação sem sentido físico (kV {v['kV']:g}, kA {v['kA']:g}): não sigo")
    s = ganhos.sugestao(v["kV"], v["kA"], CFG)
    e["modelo"] = dict(kV=v["kV"], kA=v["kA"], kG_log=v.get("kG", float("nan")),
                       kS_log=v["kS"], r2=r["r2"], n=r["n"], kP=s["kP"], kD=s["kD"],
                       criterios=dict(tol_pos=s["tol_pos"], tol_vel=s["tol_vel"], umax=s["umax"],
                                      atraso_ms=s["atraso_ms"]))
    salva_estado(pasta, e)
    print(f"modelo: kV {v['kV']:.6g} V/({CFG.UNIDADE}/s), kA {v['kA']:.6g} (R² {r['r2']:.3f}) → "
          f"LQR kP {s['kP']:.5f}, kD {s['kD']:.7f}")


def cmd_campanha(pasta):
    e = carrega_estado(pasta)
    at, mo, ff = e.get("atual", {}), e.get("modelo"), e.get("ff", {})
    if not at or not mo:
        sys.exit("falta 'atual' ou 'modelo' no estado.json")
    base = {k: at[k] for k in CHAVES if k in at and k not in ("kS", "VelkV")}
    # kI acompanha o kP na mesma proporção de hoje (o LQR não dá kI)
    razao_i = at.get("kI", 0.0) / at["kP"] if at.get("kP") else 0.0
    nomes, muda = ["atual"], [""]
    cands = {"atual": dict(params=dict(base), fp=None, extra="")}

    def cand(nome, fp, fd=1.0, extra=""):
        kp = mo["kP"] * fp
        kd = mo["kD"] * fd
        nomes.append(nome)
        m = f"kP={g(kp)} kI={g(kp * razao_i)} kD={g(kd)}" + (" " + extra if extra else "")
        muda.append(m)
        cands[nome] = dict(params=params_de(base, m), fp=fp, extra=extra)

    kg_ff = ff.get("kG", float("nan"))
    extra_kg = ""
    if CFG.GRAVIDADE != "NONE" and np.isfinite(kg_ff) and kg_ff > 0 and at.get("kG"):
        if abs(kg_ff - at["kG"]) > 0.15 * at["kG"]:
            extra_kg = f"kG={g(kg_ff)}"  # o kG do kit difere >15% do atual: testa junto
    cand("modelo", 1.0)
    cand("mod07", 0.7)
    cand("mod14", 1.4)
    if extra_kg:
        cand("modeloG", 1.0, extra=extra_kg)
    e["base"], e["razao_i"], e["cands"] = base, razao_i, cands
    salva_estado(pasta, e)
    escreve_campanha(os.path.join(pasta, "campanha_auto.sh"), base, nomes, muda, mo, 0)


def params_de(base, muda):
    p = dict(base)
    p.update({k: float(v) for k, v in (kv.split("=", 1) for kv in muda.split())})
    return p


def escreve_campanha(p, base, nomes, muda, mo, para_se_abortar):
    linhas = ["# Campanha GERADA pelo autoajuste (não edite à mão; rode o autoajuste de novo)",
              f"# MEC={CFG.arquivo}; modelo: kV {mo['kV']:.6g}, kA {mo['kA']:.6g} → kP "
              f"{mo['kP']:.5f}, kD {mo['kD']:.7f} (LQR {mo['criterios']})",
              "BASE=\"" + " ".join(f"{k}={g(v)}" for k, v in base.items()) + "\"",
              "NOMES=(" + " ".join(nomes) + ")",
              "MUDA=(" + "\n      ".join(f"\"{m}\"" for m in muda) + ")",
              f"PARA_SE_ABORTAR={para_se_abortar}"]
    with open(p, "w", encoding="utf-8") as f:
        f.write("\n".join(linhas) + "\n")
    print(open(p, encoding="utf-8").read())


def nome_da_config(e, chave):
    """Nome da candidata (atual, modelo, mod14...) cujos ajustes batem com a chave do pontua."""
    for nome, c in e.get("cands", {}).items():
        pr = c["params"]
        if all(k in pr and abs(pr[k] - v) <= 1e-5 * max(1e-12, abs(v)) for k, v in chave):
            return nome
    return ""


def cmd_campanha2(pasta):
    """Segunda volta: só se a melhor (sem oscilar) for a de fator MAIS ALTO ou MAIS BAIXO entre as
    candidatas simples (kP/kI/kD do modelo × fator, sem mudar o kG). Então testa a melhor de novo
    (referência contra o ruído entre horários) + 2 degraus para aquele lado (×F e ×F², ou ÷).
    Indo para ganhos MAIORES: em ordem crescente e PARA_SE_ABORTAR=1 (a 1ª que oscilar encerra)."""
    e = carrega_estado(pasta)
    tab, cands, mo = e.get("tabela"), e.get("cands", {}), e.get("modelo")
    if not tab or not cands or not mo:
        sys.exit("falta rodar 'campanha' e 'relatorio' antes")
    simples = {n: c for n, c in cands.items() if c["fp"] is not None and not c["extra"]}
    boas = [t for t in tab if t["problemas"] == 0 and t["nome"]]
    if len(simples) < 2 or not boas:
        print("2ª volta: não se aplica (poucas candidatas ou todas com problema)")
        sys.exit(2)
    melhor = boas[0]["nome"]
    fps = sorted(c["fp"] for c in simples.values())
    F = float(CFG.extra.get("VOLTA2_FATOR", 1.3))
    if melhor in simples and simples[melhor]["fp"] == fps[-1]:
        lado, fatores = "acima", [fps[-1] * F, fps[-1] * F * F]
    elif melhor in simples and simples[melhor]["fp"] == fps[0]:
        lado, fatores = "abaixo", [fps[0] / F, fps[0] / F / F]
    else:
        print(f"2ª volta: não precisa — a melhor ({melhor}) não está na ponta da faixa testada "
              f"(fatores {', '.join(f'{x:g}' for x in fps)})")
        sys.exit(2)
    base, razao_i = e["base"], e["razao_i"]
    c0 = simples[melhor]["params"]
    nomes, muda = [melhor], [f"kP={g(c0['kP'])} kI={g(c0['kI'])} kD={g(c0['kD'])}"]
    for fp in fatores:
        kp = mo["kP"] * fp
        nome = f"mod{int(round(fp * 100))}"
        m = f"kP={g(kp)} kI={g(kp * razao_i)} kD={g(c0['kD'])}"
        nomes.append(nome)
        muda.append(m)
        cands[nome] = dict(params=params_de(base, m), fp=fp, extra="")
    e["cands"] = cands
    e["volta2"] = dict(lado=lado, nomes=nomes)
    salva_estado(pasta, e)
    print(f"2ª volta: a melhor foi {melhor} (fator {simples[melhor]['fp']:g}, a ponta {lado} da "
          f"faixa) → testa {', '.join(f'{x:.2f}' for x in fatores)}")
    escreve_campanha(os.path.join(pasta, "campanha_volta2.sh"), base, nomes, muda, mo,
                     1 if lado == "acima" else 0)


def cmd_relatorio(pasta, logs):
    import log_rodadas
    import pontua
    e = carrega_estado(pasta)
    # junta os logs das voltas anteriores (o robô pode ter reiniciado entre elas: outro arquivo)
    usados = [f for f in e.get("logs", []) if os.path.exists(f) and f not in logs] + list(logs)
    e["logs"] = usados
    rods, pars = [], []
    for f in usados:
        r, p = log_rodadas.segmentos(f)
        rods += r
        pars += p
    if not rods:
        sys.exit("nenhuma rodada do robusto nos logs")
    tab, esc, motivo = pontua.recomenda(pontua.agrupa(rods, pars))
    for t in tab:
        t["nome"] = nome_da_config(e, t["chave"])
    e["tabela"] = [dict(nome=t["nome"], nota=t["nota"], problemas=t["problemas"]) for t in tab]
    e["escolhida"] = esc["nome"]
    salva_estado(pasta, e)
    at, mo, ff = e.get("atual", {}), e.get("modelo", {}), e.get("ff", {})
    final = dict(at)
    final.update(dict(esc["chave"]))
    kv_rpm = mo.get("kV", float("nan")) * 6 if CFG.CIRCULAR else float("nan")
    L = [f"# Relatório do assistente de ajuste — {CFG.arquivo} ({CFG.NOME})", "",
         "Gerado pelo `autoajuste.sh`. **É uma recomendação: uma pessoa aprova antes de gravar no "
         "código**, depois deploy e `robusto.sh` 2× vindo do código.", "",
         "## Medidas", "",
         f"- Valores de partida (ao vivo): " + ", ".join(f"{k} {g(v)}" for k, v in at.items()),
         f"- Feedforward pelo kit (passadas lentas): kS {g(ff.get('kS', np.nan))} V, kG "
         f"{g(ff.get('kG', np.nan))} V, kV {g(ff.get('kV', np.nan))} V/RPM" if ff else
         "- Feedforward pelo kit: não rodou",
         f"- Modelo pelo log a 5 ms: kV {g(mo.get('kV', np.nan))} V/({CFG.UNIDADE}/s)"
         f" (= {g(kv_rpm)} V/RPM), kA {g(mo.get('kA', np.nan))} V/({CFG.UNIDADE}/s²), "
         f"R² {mo.get('r2', np.nan):.3f}" if mo else "- Modelo: não rodou",
         f"- Ganhos do modelo (LQR, {mo.get('criterios')}): kP {g(mo.get('kP', np.nan))}, "
         f"kD {g(mo.get('kD', np.nan))}" if mo else "", "",
         "## Bateria (log a 5 ms; nota menor = melhor)", "",
         f"Pesos da nota (NOTA_PESOS): {' '.join(f'{k}={v:g}' for k, v in pontua.P.items())}. "
         f"Paradas = oscilaram / feitas no PARADA_ALVO ({CFG.PARADA_ALVO:g}).", "",
         "| configuração | rodadas | nota | pior | passa méd | grandes | máx | erro final | chegada "
         "| paradas |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for t in tab:
        marca = " **←**" if t is esc else ""
        prob = f" ({t['problemas']} com oscilação/sem chegar)" if t["problemas"] else ""
        nome = f"**{t['nome']}** " if t["nome"] else ""
        L.append(f"| {nome}{pontua.texto(t['chave'])}{marca}{prob} | {t['n']} | {t['nota']:.2f} | "
                 f"{t['pior']:.2f} | {t['passa']:.2f} | {t['grandes']:.2f} | {t['maxp']:.1f} | "
                 f"{t['erro']:.2f} | {t['chega']:.2f} | {pontua.paradas_txt(t)} |")
    empatadas = [t for t in tab if t is not esc and t["problemas"] == 0
                 and t["nota"] <= esc["nota"] * 1.10]
    L += ["", "## Recomendação", "",
          f"**{pontua.texto(esc['chave'])}** ({motivo})."
          + (" Empate técnico com: " + "; ".join(pontua.texto(t["chave"]) for t in empatadas)
             + " — a diferença é menor que o ruído entre rodadas." if empatadas else ""), "",
          "Linha para a `MechanismConfig` (confira as unidades; kS fica 0 — lição 4):", "",
          f"    .pid({g(final.get('kP', np.nan))}, {g(final.get('kI', np.nan))}, "
          f"{g(final.get('kD', np.nan))})",
          f"    .feedforward(0.0, {g(final.get('kG', np.nan))}, {g(at.get('VelkV', np.nan))}, "
          f"{g(final.get('kA', np.nan))})", "",
          "Ainda a fazer à mão: teste de parada no ponto fraco (`testeparada.sh`) com os valores "
          "escolhidos, e olhar os piores movimentos no log (`log_ana.py`)."]
    p = os.path.join(pasta, "RELATORIO.md")
    with open(p, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    print("\n".join(L))
    print(f"\n(gravado {p})")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    sub, pasta, resto = sys.argv[1], sys.argv[2], sys.argv[3:]
    os.makedirs(pasta, exist_ok=True)
    {"atual": lambda: cmd_atual(pasta, resto[0]), "ff": lambda: cmd_ff(pasta, resto[0]),
     "modelo": lambda: cmd_modelo(pasta, resto), "campanha": lambda: cmd_campanha(pasta),
     "relatorio": lambda: cmd_relatorio(pasta, resto),
     "campanha2": lambda: cmd_campanha2(pasta)}[sub]()
