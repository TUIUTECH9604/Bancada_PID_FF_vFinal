"""Ajuste do modelo do braço a partir de /home/lvuser/sysid.csv.

Modelo: V = kS*sgn(v) + kG*cos(theta) + kV*v + kA*a   (v em graus/s, a em graus/s^2)

Dois métodos:
  1) Direto: derivadas numéricas suavizadas, mínimos quadrados em V.
  2) Discreto (como o SysId do WPILib): v[k+1] = alpha*v[k] + beta*V[k] + gamma*sgn + delta*cos
     -> kV = (1-alpha)/beta, kA = -kV*dt/ln(alpha), kS = -gamma/beta, kG = -delta/beta
"""
import sys
import numpy as np

path = sys.argv[1]
d = np.genfromtxt(path, delimiter=',', names=True)
print(f"{len(d)} amostras, testes: {sorted(set(d['test'].astype(int)))}")

V_MIN_DEGPS = 15.0  # abaixo disso o atrito estático domina; fora do ajuste


def smooth_deriv(t, x, half=3):
    """Derivada por regressão linear local (janela de 2*half+1 amostras)."""
    n = len(x)
    out = np.full(n, np.nan)
    for i in range(half, n - half):
        tt = t[i - half:i + half + 1] - t[i]
        xx = x[i - half:i + half + 1]
        out[i] = np.polyfit(tt, xx, 1)[0]
    return out


rows_direct, rows_disc = [], []
for test in sorted(set(d['test'].astype(int))):
    m = d['test'].astype(int) == test
    t, V, th = d['t'][m], d['cmdV'][m], d['posDeg'][m]
    if len(t) < 30:
        continue
    v = smooth_deriv(t, th, 3)
    a = smooth_deriv(t, v, 4)
    dt = np.median(np.diff(t))
    print(f"teste {test}: {len(t)} amostras, {t[-1]-t[0]:.2f} s, dt {dt*1000:.2f} ms, "
          f"V {V.min():+.2f}..{V.max():+.2f}, vel max {np.nanmax(np.abs(v))/6:.0f} RPM")
    for i in range(len(t) - 1):
        if np.isnan(v[i]) or np.isnan(a[i]) or abs(v[i]) < V_MIN_DEGPS:
            continue
        c = np.cos(np.radians(th[i]))
        rows_direct.append((np.sign(v[i]), c, v[i], a[i], V[i], test))
        if not np.isnan(v[i + 1]):
            rows_disc.append((v[i], V[i], np.sign(v[i]), c, v[i + 1], t[i + 1] - t[i]))

R = np.array(rows_direct)
X, y = R[:, :4], R[:, 4]
coef, *_ = np.linalg.lstsq(X, y, rcond=None)
kS, kG, kV, kA = coef
res = y - X @ coef
r2 = 1 - res.var() / y.var()
print("\n== Método direto ==")
print(f"kS = {kS:.3f} V   kG = {kG:.3f} V   kV = {kV:.6f} V/(°/s) = {kV*6:.5f} V/RPM   "
      f"kA = {kA:.7f} V/(°/s²)   R² = {r2:.3f}  (n={len(y)})")

D = np.array(rows_disc)
Xd = D[:, :4]
yd = D[:, 4]
cd, *_ = np.linalg.lstsq(Xd, yd, rcond=None)
alpha, beta, gamma, delta = cd
dt = np.median(D[:, 5])
kVd = (1 - alpha) / beta
kAd = -kVd * dt / np.log(alpha) if 0 < alpha < 1 else float('nan')
print("\n== Método discreto (SysId) ==")
print(f"alpha={alpha:.5f} beta={beta:.4f} dt={dt*1000:.2f} ms")
print(f"kS = {-gamma/beta:.3f} V   kG = {-delta/beta:.3f} V   kV = {kVd:.6f} V/(°/s) = "
      f"{kVd*6:.5f} V/RPM   kA = {kAd:.7f} V/(°/s²)")

# Por teste (consistência)
print("\n== Resíduo RMS do método direto por teste ==")
for test in sorted(set(R[:, 5].astype(int))):
    mm = R[:, 5] == test
    print(f"teste {test}: {np.sqrt(np.mean(res[mm]**2)):.3f} V  (n={mm.sum()})")
