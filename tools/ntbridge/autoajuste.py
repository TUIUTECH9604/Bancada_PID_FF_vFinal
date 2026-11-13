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
  campanha3 <pasta>                          3ª volta: refino de kD (×0,5, ×2) e kI (0, ×2; ou um
                                             valor de partida se for 0) em volta da escolhida
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
    registra_volta(e, "1ª volta", "atual", nomes)
    salva_estado(pasta, e)
    escreve_campanha(os.path.join(pasta, "campanha_auto.sh"), base, nomes, muda, mo, 0)


def registra_volta(e, nome, ref, nomes):
    """Guarda as configurações de cada volta e a REFERÊNCIA dela (a que se repete de uma volta para
    a outra). O relatório compara cada volta só com a referência medida NA MESMA VOLTA: o mecanismo
    pode mudar entre as voltas (parafuso afrouxando, folga — aconteceu na bancada em 2026-11-12 e
    2026-11-13) e aí comparar rodadas de horários diferentes engana."""
    vs = [v for v in voltas_do_estado(e) if v["nome"] != nome]
    vs.append(dict(nome=nome, ref=ref, nomes=list(nomes)))
    e["voltas"] = vs


def voltas_do_estado(e):
    if "voltas" in e:
        return e["voltas"]
    # estado antigo (antes de 2026-11-13): reconstrói pelo que as campanhas guardaram
    cands = e.get("cands", {})
    vs = []
    v1 = [n for n, c in cands.items() if c.get("extra") != "volta3"
          and n not in e.get("volta2", {}).get("nomes", [])[1:]]
    if v1:
        vs.append(dict(nome="1ª volta", ref="atual", nomes=v1))
    if e.get("volta2"):
        vs.append(dict(nome="2ª volta", ref=e["volta2"]["nomes"][0], nomes=e["volta2"]["nomes"]))
    if e.get("volta3"):
        c3 = e["volta3"]["centro"]
        vs.append(dict(nome="3ª volta", ref=c3,
                       nomes=[c3] + [n for n, c in cands.items() if c.get("extra") == "volta3"]))
    return vs


def nome_de_ajustes(e, ajustes):
    """Nome da candidata cujos parâmetros batem com os ajustes gravados no log ('' se nenhuma)."""
    for nome, c in e.get("cands", {}).items():
        pr = c["params"]
        comuns = [k for k in pr if k in ajustes and not np.isnan(ajustes[k])]
        if comuns and all(abs(pr[k] - ajustes[k]) <= 1e-5 * max(1e-12, abs(pr[k])) for k in comuns):
            return nome
    return ""


def separa_voltas(e, rods, pars):
    """{nome da volta: (rodadas, paradas)}. Uma configuração que aparece em mais de uma volta (a
    referência) vai para a volta cujas outras configurações rodaram mais perto no tempo, no mesmo
    log."""
    vs = voltas_do_estado(e)
    for d in list(rods) + list(pars):
        d["_nome"] = nome_de_ajustes(e, d.get("ajustes", {}))
    dono = {}
    for v in vs:
        for n in v["nomes"]:
            dono.setdefault(n, []).append(v["nome"])
    faixa = {}  # (volta, arquivo) → (t mín, t máx) das configurações SÓ dela
    for d in rods:
        vv = dono.get(d["_nome"], [])
        if len(vv) == 1:
            a, b = faixa.get((vv[0], d["arquivo"]), (d["t"], d["t"]))
            faixa[(vv[0], d["arquivo"])] = (min(a, d["t"]), max(b, d["t"]))

    def volta_de(d):
        vv = dono.get(d["_nome"], [])
        if len(vv) <= 1:
            return vv[0] if vv else None

        def dist(v):
            if (v, d["arquivo"]) not in faixa:
                return float("inf")
            a, b = faixa[(v, d["arquivo"])]
            return 0.0 if a <= d["t"] <= b else min(abs(d["t"] - a), abs(d["t"] - b))
        return min(vv, key=dist)

    out = {v["nome"]: ([], []) for v in vs}
    for d in rods:
        v = volta_de(d)
        if v:
            out[v][0].append(d)
    for d in pars:
        v = volta_de(d)
        if v:
            out[v][1].append(d)
    return out


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
        # compara só os ajustes que a candidata define (a chave pode ter outros, sem valor ou
        # que só mudaram entre logs)
        comuns = [(k, v) for k, v in chave if k in pr and v is not None]
        if comuns and all(abs(pr[k] - v) <= 1e-5 * max(1e-12, abs(v)) for k, v in comuns):
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
    # só as candidatas que TÊM resultado (uma 2ª volta gerada e ainda não testada não conta)
    testadas = {t["nome"] for t in tab if t["nome"]}
    simples = {n: c for n, c in cands.items()
               if c["fp"] is not None and not c["extra"] and n in testadas}
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
    registra_volta(e, "2ª volta", melhor, nomes)
    salva_estado(pasta, e)
    print(f"2ª volta: a melhor foi {melhor} (fator {simples[melhor]['fp']:g}, a ponta {lado} da "
          f"faixa) → testa {', '.join(f'{x:.2f}' for x in fatores)}")
    escreve_campanha(os.path.join(pasta, "campanha_volta2.sh"), base, nomes, muda, mo,
                     1 if lado == "acima" else 0)


def cmd_campanha3(pasta):
    """Terceira volta (REFINO de kD e kI) em volta da configuração recomendada. O LQR não dá kI (o
    modelo não tem atrito) e o kD dele não leva em conta a folga: os dois só se acham testando.

    Muda UMA coisa de cada vez em relação à escolhida (que entra de novo como referência):
      kD × 0,5 e × 2          (se a escolhida tem kD 0: o kD do LQR e o dobro)
      kI = 0 e kI × 2         (se a escolhida tem kI 0: kP/Ti e o dobro; Ti = VOLTA3_TI_S do .conf,
                               o tempo em que o integral corrige um erro parado — 0,3 s dá o
                               kI 0,02 da bancada com kP 0,006)
    PARA_SE_ABORTAR=0: não é uma subida em ordem; quem oscilar leva a nota de problema."""
    e = carrega_estado(pasta)
    cands, mo, esc = e.get("cands", {}), e.get("modelo", {}), e.get("escolhida")
    if not esc or esc not in cands:
        sys.exit("falta o 'relatorio' (a configuração escolhida) antes da 3ª volta")
    base = e["base"]
    c0 = dict(cands[esc]["params"])
    kp, ki, kd = c0["kP"], c0.get("kI", 0.0), c0.get("kD", 0.0)
    ti = float(CFG.extra.get("VOLTA3_TI_S", 0.3))
    fixos = {k: v for k, v in c0.items() if k in ("kP", "kI", "kD") or base.get(k) != v}

    def m(**troca):
        d = dict(fixos)
        d.update(troca)
        return " ".join(f"{k}={g(v)}" for k, v in d.items())

    nomes, muda, origem = [esc], [m()], {esc: "referência"}
    if kd > 0:
        op_d = [("D05", kd * 0.5, "kD ×0,5"), ("D2", kd * 2, "kD ×2")]
    else:
        op_d = [("Dlqr", mo.get("kD", 0.0), "kD do LQR"), ("Dlqr2", 2 * mo.get("kD", 0.0), "kD do LQR ×2")]
    if ki > 0:
        op_i = [("I0", 0.0, "sem kI"), ("I2", ki * 2, "kI ×2")]
    else:
        op_i = [("Iini", kp / ti, f"kI = kP/{ti:g} s"), ("Iini2", 2 * kp / ti, f"kI = 2·kP/{ti:g} s")]
    for suf, v, txt in op_d:
        if v > 0:
            n = f"{esc}_{suf}"
            nomes.append(n); muda.append(m(kD=v)); origem[n] = txt
    for suf, v, txt in op_i:
        n = f"{esc}_{suf}"
        nomes.append(n); muda.append(m(kI=v)); origem[n] = txt
    for n, mm in zip(nomes[1:], muda[1:]):
        cands[n] = dict(params=params_de(base, mm), fp=None, extra="volta3")
    e["cands"], e["volta3"] = cands, dict(centro=esc, origem=origem)
    registra_volta(e, "3ª volta", esc, nomes)
    salva_estado(pasta, e)
    print(f"3ª volta (refino de kD e kI) em volta de {esc}: "
          + ", ".join(f"{n} ({origem[n]})" for n in nomes))
    escreve_campanha(os.path.join(pasta, "campanha_volta3.sh"), base, nomes, muda, mo, 0)


def origem_dos_ganhos(e, nome):
    """Texto: de onde veio cada ganho da configuração escolhida (para quem aprova entender)."""
    c = e.get("cands", {}).get(nome)
    mo, ff, at = e.get("modelo", {}), e.get("ff", {}), e.get("atual", {})
    if not c:
        return ["- (configuração sem nome: compare a tabela com os valores de partida)"]
    p = c["params"]
    L = []
    if c["fp"] is not None:
        L.append(f"- **kP {g(p['kP'])}**: LQR ({g(mo.get('kP', np.nan))}) × {c['fp']:g}, escolhido "
                 "pela bateria entre os fatores testados.")
    elif nome == "atual":
        L.append(f"- **kP {g(p['kP'])}**: o de partida (os do LQR não ganharam por margem "
                 "maior que o ruído).")
    else:
        L.append(f"- **kP {g(p['kP'])}**: da configuração que serviu de centro do refino.")
    v3 = e.get("volta3", {})
    if v3:
        L.append(f"- **kD {g(p.get('kD', 0))} e kI {g(p.get('kI', 0))}**: refino da 3ª volta em "
                 f"volta de {v3['centro']} ({v3['origem'].get(nome, 'referência')} ganhou).")
    else:
        L.append(f"- **kD {g(p.get('kD', 0))}**: LQR ou de partida (sem refino: rode a 3ª volta). "
                 f"**kI {g(p.get('kI', 0))}**: proporção kI/kP de partida (sem refino).")
    if CFG.GRAVIDADE != "NONE":
        L.append(f"- **kG {g(p.get('kG', np.nan))}**: " + ("medido pelo kit e testado"
                 if ff and abs(p.get("kG", 0) - ff.get("kG", -1)) < 1e-9 else
                 f"o de partida (o kit mediu {g(ff.get('kG', np.nan))}; ficou dentro do ruído)"
                 if ff else "o de partida (kit não rodou)"))
    kv_log = mo.get("kV", np.nan)
    kv_txt = (f"{g(kv_log * 6)} V/RPM" if CFG.CIRCULAR else f"{g(kv_log)} V/({CFG.UNIDADE}/s)")
    L.append(f"- kV {g(at.get('VelkV', np.nan))} V/RPM e kA {g(p.get('kA', np.nan))}: de partida; o "
             f"log mediu kV {kv_txt} e kA {g(mo.get('kA', np.nan))} — se diferirem muito, corrija na "
             "config.")
    return L


EMPATE = 0.10    # diferença de nota menor que isto = ruído entre rodadas
MUDOU = 1.30     # a MESMA configuração piorou mais que isto entre voltas = o mecanismo mudou


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
    # Rodadas com o laço de controle lento não valem (2026-11-13: 25 ms em vez de 5 → a passada
    # triplicou com os mesmos ganhos): ficam de fora e o relatório avisa
    lentas = [d for d in rods if d.get("laco_ms", 0) > 8]
    rods = [d for d in rods if not d.get("laco_ms", 0) > 8]
    if not rods:
        sys.exit(f"todas as {len(lentas)} rodadas rodaram com o laço lento (> 8 ms): reinicie o "
                 "código do robô e rode de novo")

    # Cada volta é avaliada SÓ com as rodadas dela (mesmo horário, mesma condição do mecanismo)
    por_volta = separa_voltas(e, rods, pars)
    voltas = [v for v in voltas_do_estado(e) if por_volta.get(v["nome"], ([], []))[0]]
    if not voltas:
        sys.exit("nenhuma rodada das configurações do assistente nos logs")
    res = []
    for v in voltas:
        rv, pv = por_volta[v["nome"]]
        tab, esc_v, motivo_v = pontua.recomenda(pontua.agrupa(rv, pv), EMPATE)
        for t in tab:
            t["nome"] = nome_da_config(e, t["chave"])
        ref = next((t for t in tab if t["nome"] == v["ref"]), None)
        res.append(dict(volta=v, tab=tab, esc=esc_v, motivo=motivo_v, ref=ref))

    # Recomendação encadeada: a 1ª volta escolhe; nas seguintes só MUDA se alguma configuração
    # ganhar da referência DAQUELA volta por mais que o empate (e sem oscilar)
    rec, motivo, por = None, "", ""
    for k, r in enumerate(res):
        if k == 0 or rec is None or r["ref"] is None:
            if r["esc"] is not None:
                rec, motivo, por = r["esc"]["nome"], r["motivo"], r["volta"]["nome"]
            continue
        melhores = [t for t in r["tab"] if t["problemas"] == 0 and t is not r["ref"]
                    and t["nota"] < r["ref"]["nota"] * (1 - EMPATE)]
        if melhores:
            t = min(melhores, key=lambda x: x["nota"])
            rec, por = t["nome"], r["volta"]["nome"]
            motivo = (f"ganhou da referência {r['ref']['nome']} da mesma volta por "
                      f"{1 - t['nota'] / r['ref']['nota']:.0%}")
        else:
            motivo += f"; na {r['volta']['nome']} nada ganhou da referência por mais que o ruído"
    # O mecanismo mudou? A mesma configuração medida em voltas diferentes
    avisos, vistos = [], {}
    for r in res:
        for t in r["tab"]:
            if t["nome"] and t["problemas"] == 0:
                vistos.setdefault(t["nome"], []).append((r["volta"]["nome"], t["nota"]))
    for nome, lst in vistos.items():
        if len(lst) > 1:
            lo, hi = min(lst, key=lambda x: x[1]), max(lst, key=lambda x: x[1])
            if hi[1] > lo[1] * MUDOU:
                avisos.append(f"**{nome}** (mesmos ganhos) teve nota {lo[1]:.2f} na {lo[0]} e "
                              f"{hi[1]:.2f} na {hi[0]} ({hi[1] / lo[1]:.1f}×)")

    ultima = res[-1]
    e["tabela"] = [dict(nome=t["nome"], nota=t["nota"], problemas=t["problemas"])
                   for t in ultima["tab"]]
    e["escolhida"] = rec
    e["mecanismo_mudou"] = avisos
    salva_estado(pasta, e)

    at, mo, ff = e.get("atual", {}), e.get("modelo", {}), e.get("ff", {})
    final = dict(at)
    if rec in e.get("cands", {}):
        final.update(e["cands"][rec]["params"])
    kv_rpm = mo.get("kV", float("nan")) * 6 if CFG.CIRCULAR else float("nan")
    L = [f"# Relatório do assistente de ajuste — {CFG.arquivo} ({CFG.NOME})", "",
         "Gerado pelo `autoajuste.sh`. **É uma recomendação: uma pessoa aprova antes de gravar no "
         "código**, depois deploy e `robusto.sh` 2× vindo do código.", ""]
    if lentas:
        L += ["## ⚠️ Rodadas com o LAÇO DE CONTROLE LENTO (deixadas de fora)", "",
              f"{len(lentas)} rodada(s) rodaram com o laço a ~"
              f"{np.median([d['laco_ms'] for d in lentas]):.0f} ms em vez de 5 ms: com isso o "
              "mecanismo passa bem mais do alvo e a comparação não vale. Reinicie o código do robô "
              "(Restart Robot Code) e rode essas configurações de novo. Em 2026-11-13 isso aconteceu "
              "sozinho depois de ~1 h de programa ligado.", ""]
    if avisos:
        L += ["## ⚠️ O MECANISMO MUDOU durante os testes", "",
              "A mesma configuração ficou bem pior de uma volta para outra — não foram os ganhos:"
              " parafuso afrouxando, folga aumentando, correia, bateria. **Confira a mecânica** "
              "(e meça a folga) antes de confiar nos números absolutos. As comparações abaixo são "
              "DENTRO de cada volta (mesma condição), por isso a recomendação ainda vale como "
              "comparação; depois de consertar, rode o `robusto.sh` de novo para ter a referência.",
              ""] + [f"- {a}" for a in avisos] + [""]
    L += ["## Medidas", "",
          "- Valores de partida (ao vivo): " + ", ".join(f"{k} {g(v)}" for k, v in at.items()),
          f"- Feedforward pelo kit (passadas lentas): kS {g(ff.get('kS', np.nan))} V, kG "
          f"{g(ff.get('kG', np.nan))} V, kV {g(ff.get('kV', np.nan))} V/RPM" if ff else
          "- Feedforward pelo kit: não rodou",
          f"- Modelo pelo log a 5 ms: kV {g(mo.get('kV', np.nan))} V/({CFG.UNIDADE}/s)"
          f" (= {g(kv_rpm)} V/RPM), kA {g(mo.get('kA', np.nan))} V/({CFG.UNIDADE}/s²), "
          f"R² {mo.get('r2', np.nan):.3f}" if mo else "- Modelo: não rodou",
          f"- Ganhos do modelo (LQR, {mo.get('criterios')}): kP {g(mo.get('kP', np.nan))}, "
          f"kD {g(mo.get('kD', np.nan))}" if mo else "", "",
          "## Voltas (log a 5 ms; nota menor = melhor; cada volta comparada com a SUA referência)",
          "",
          f"Pesos da nota (NOTA_PESOS): {' '.join(f'{k}={v:g}' for k, v in pontua.P.items())}. "
          f"Paradas = oscilaram / feitas no PARADA_ALVO ({CFG.PARADA_ALVO:g}). 'rel.' = nota / "
          f"nota da referência da volta (abaixo de {1 - EMPATE:.2f} = ganhou de verdade)."]
    for r in res:
        refn = r["ref"]["nota"] if r["ref"] else float("nan")
        L += ["", f"### {r['volta']['nome']} — referência: {r['volta']['ref']}", "",
              "| configuração | rodadas | nota | rel. | pior | passa méd | grandes | máx | erro final "
              "| chegada | paradas |", "|---|---|---|---|---|---|---|---|---|---|---|"]
        for t in r["tab"]:
            marca = " **←**" if t["nome"] == rec and r["volta"]["nome"] == por else ""
            prob = f" ({t['problemas']} com oscilação/sem chegar)" if t["problemas"] else ""
            nome = f"**{t['nome']}** " if t["nome"] else ""
            L.append(f"| {nome}{pontua.texto(t['chave'])}{marca}{prob} | {t['n']} | {t['nota']:.2f} "
                     f"| {t['nota'] / refn:.2f} | {t['pior']:.2f} | {t['passa']:.2f} | "
                     f"{t['grandes']:.2f} | {t['maxp']:.1f} | {t['erro']:.2f} | {t['chega']:.2f} | "
                     f"{pontua.paradas_txt(t)} |")
    ganhos = ", ".join(f"{k} {g(final.get(k, np.nan))}" for k in ("kP", "kI", "kD", "kG"))
    L += ["", "## Recomendação", "",
          f"**{rec}** — {ganhos} ({motivo}; decidido na {por}).", "",
          "### De onde veio cada valor", ""] + origem_dos_ganhos(e, rec) + ["",
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
     "campanha2": lambda: cmd_campanha2(pasta),
     "campanha3": lambda: cmd_campanha3(pasta)}[sub]()
