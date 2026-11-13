"""kP e kD de PARTIDA pelo modelo (LQR, como o LinearQuadraticRegulator do WPILib).

Modelo do mecanismo (de kV e kA, medidos pelo identifica.py ou pelo SysId), tensão já sem o
feedforward:   posição' = velocidade ;  velocidade' = −(kV/kA)·velocidade + tensão/kA
Discretizado no período da malha (5 ms), com o atraso do acionamento compensado. O LQR escolhe o
ganho que equilibra "erro de posição", "erro de velocidade" e "tensão gasta" pelos critérios de
Bryson: Q = diag(1/TOL_POS², 1/TOL_VEL²), R = 1/UMAX² (no .conf: LQR_TOL_POS, LQR_TOL_VEL,
LQR_UMAX). Mais apertado (tolerâncias menores, UMAX maior) = ganhos maiores.

É um PONTO DE PARTIDA: o modelo não tem folga, atrito nem saturação. A bateria em volta dele
(autoajuste.sh) decide. Na bancada (kV 0,0053 V/(°/s), kA 0,00005–0,00007) deu kP 0,0047–0,0071,
perto do 0,006 achado por bateria; o kD sugerido (~0,00002) é menor que o do código (0,00015) —
coerente com a bateria de 2026-11-12, em que menos D foi melhor.

uso: [MEC=...] python ganhos.py kV kA [--atraso 10] [--tol-pos 2] [--tol-vel 50] [--umax 6]
     (kV em V/(unidade/s) e kA em V/(unidade/s²), como o identifica.py imprime)
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mec  # noqa: E402

NOMINAL_V = 12.0  # o PID do código trabalha em fração de 12 V por unidade (kP·erro → ×12 V)


def _expm(M):
    """exp(M) por série de Taylor com escala e quadrado (matrizes pequenas)."""
    n = np.linalg.norm(M, 1)
    s = max(0, int(np.ceil(np.log2(n))) + 1) if n > 0 else 0
    A = M / 2 ** s
    E = np.eye(len(M))
    T = np.eye(len(M))
    for k in range(1, 20):
        T = T @ A / k
        E = E + T
    for _ in range(s):
        E = E @ E
    return E


def _dlqr(A, B, Q, R):
    P = Q.copy()
    for _ in range(20000):
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        Pn = Q + A.T @ P @ (A - B @ K)
        if np.max(np.abs(Pn - P)) <= 1e-12 * max(1.0, np.max(np.abs(P))):
            P = Pn
            break
        P = Pn
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


def lqr(kV, kA, tol_pos, tol_vel, umax, dt=0.005, atraso=0.010):
    """Devolve (K_pos [V/unidade], K_vel [V/(unidade/s)])."""
    if kA <= 0 or kV <= 0:
        raise ValueError("kV e kA precisam ser positivos")
    Ac = np.array([[0.0, 1.0], [0.0, -kV / kA]])
    Bc = np.array([[0.0], [1.0 / kA]])
    M = np.zeros((3, 3))
    M[:2, :2] = Ac * dt
    M[:2, 2:] = Bc * dt
    E = _expm(M)
    A, B = E[:2, :2], E[:2, 2:]
    K = _dlqr(A, B, np.diag([1 / tol_pos ** 2, 1 / tol_vel ** 2]), np.array([[1 / umax ** 2]]))
    n = int(round(atraso / dt))
    if n > 0:  # compensação de atraso (igual ao latencyCompensate do WPILib)
        K = K @ np.linalg.matrix_power(A - B @ K, n)
    return float(K[0, 0]), float(K[0, 1])


def sugestao(kV, kA, cfg=None, atraso=None, tol_pos=None, tol_vel=None, umax=None):
    """kP/kD no formato do código (fração de 12 V por unidade) + os critérios usados."""
    cfg = cfg or mec.carrega()
    v = cfg.extra
    tp = tol_pos or float(v.get("LQR_TOL_POS", 2.0))
    tv = tol_vel or float(v.get("LQR_TOL_VEL", 50.0))
    um = umax or float(v.get("LQR_UMAX", 6.0))
    at = (atraso if atraso is not None else float(v.get("ATRASO_MS", 10.0))) / 1000.0
    kp_v, kd_v = lqr(kV, kA, tp, tv, um, atraso=at)
    return dict(kP=kp_v / NOMINAL_V, kD=kd_v / NOMINAL_V, Kpos_V=kp_v, Kvel_V=kd_v,
                tol_pos=tp, tol_vel=tv, umax=um, atraso_ms=at * 1000)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("kV", type=float)
    ap.add_argument("kA", type=float)
    ap.add_argument("--atraso", type=float, default=None, help="ms (padrão ATRASO_MS do .conf ou 10)")
    ap.add_argument("--tol-pos", type=float, default=None)
    ap.add_argument("--tol-vel", type=float, default=None)
    ap.add_argument("--umax", type=float, default=None)
    a = ap.parse_args()
    cfg = mec.carrega()
    s = sugestao(a.kV, a.kA, cfg, a.atraso, a.tol_pos, a.tol_vel, a.umax)
    u = cfg.UNIDADE
    print(f"MEC={cfg.arquivo}: kV {a.kV:g} V/({u}/s), kA {a.kA:g} V/({u}/s²), atraso "
          f"{s['atraso_ms']:.0f} ms | critérios: posição {s['tol_pos']:g} {u}, velocidade "
          f"{s['tol_vel']:g} {u}/s, tensão {s['umax']:g} V")
    print(f"  kP = {s['kP']:.5f}  ({s['Kpos_V']:.4f} V/{u})")
    print(f"  kD = {s['kD']:.7f}  ({s['Kvel_V']:.5f} V/({u}/s))")
