"""Analisa uma gravação do robusto.sh/mapa.sh (CSV do nt.sh rec, pelo Wi-Fi).

Genérico: MEC=<mecanismo> escolhe mecanismos/<nome>.conf (CIRCULAR decide se o erro dá a volta).
Para decidir ganhos, prefira o log a 5 ms (log_rodadas.py): pelo Wi-Fi os picos curtos somem
(lição 19).
uso: [MEC=...] python mapa_ana.py arquivo.csv | python mapa_ana.py -r arq1.csv arq2.csv ...
"""
import csv, collections, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mec  # noqa: E402

CFG = mec.carrega()


def wrap(x):
    return (x + 180) % 360 - 180 if CFG.CIRCULAR else x


def load(f):
    d = collections.defaultdict(list)
    for r in csv.DictReader(open(f)):
        try:
            d[r['topic'].split('/')[-1]].append((float(r['t']), float(r['value'])))
        except ValueError:
            pass
    return d


def amostra(serie, T):
    ts = np.array([t for t, _ in serie]); xs = np.array([x for _, x in serie])
    i = np.clip(np.searchsorted(ts, T, side='right') - 1, 0, len(xs) - 1)
    return xs[i]


def analisa(f, settle=1.0):
    d = load(f)
    segs = []
    for t, v in d['getGoalDeg']:
        if v != v:
            continue
        if not segs or segs[-1][1] != v:
            segs.append([t, v])
    res = []
    for i, (t0, g) in enumerate(segs):
        if i == 0:
            continue
        t1 = segs[i + 1][0] if i + 1 < len(segs) else t0 + 2.4
        sobe = wrap(g - segs[i - 1][1]) > 0
        T = np.arange(t0 + settle, t1 - 0.05, 0.01)          # reamostra a 100 Hz (segura o
        if len(T) < 30:                                       # último valor: o NT só manda
            continue                                          # quando o valor muda)
        q = np.column_stack([T, amostra(d['getQuadDeg'], T)])
        v = amostra(d['getCommandedVolts'], T)
        e = [wrap(x - g) for t, x in d['getDeg'] if t0 < t < t1]
        x = q[:, 1] - np.polyval(np.polyfit(q[:, 0], q[:, 1], 1), q[:, 0])  # tira a deriva lenta
        dt = np.median(np.diff(q[:, 0]))
        X = np.abs(np.fft.rfft(x * np.hanning(len(x))))
        fr = np.fft.rfftfreq(len(x), dt)
        k = 1 + np.argmax(X[1:])
        # lado de onde chegou: sinal do erro quando estava entre 1 e 30 graus do alvo (num passo de
        # 180 graus, "subir" ou "descer" e ambiguo)
        antes = [y for y in e if 1.0 < abs(y) < 30.0]
        sgn = (-1 if antes[0] > 0 else 1) if antes else (1 if sobe else -1)
        chegou = next((k for k, y in enumerate(e) if abs(y) < 1.0), len(e))
        over = max([0] + [sgn * y for y in e[chegou:]])
        # Com a velocidade do perfil gravada (robusto.sh desde 2026-11-12): passa do alvo = depois
        # que o perfil PAROU de vez, no sentido em que ele terminou. A aproximação anti-folga vai
        # além do alvo de propósito e volta; isso não conta.
        pv = [(t, v) for t, v in d.get('getProfileVelDegps', []) if t0 <= t < t1]
        mov = [(t, v) for t, v in pv if abs(v) >= 1e-6]
        if mov:
            tfim = next((t for t, v in pv if t > mov[-1][0] and abs(v) < 1e-6), mov[-1][0])
            sgn = 1 if mov[-1][1] > 0 else -1
            over = max([0] + [sgn * wrap(x - g) for t, x in d['getDeg'] if tfim <= t < t1])
        # tremor RAPIDO: o que sobra tirando a media movel de 0,15 s (vibracao > ~5 Hz)
        w = 15
        mm = np.convolve(q[:, 1], np.ones(w) / w, mode='same')
        hf = np.std((q[:, 1] - mm)[w:-w]) if len(q) > 3 * w else 0.0
        # tempo para assentar: ultima vez que o ponteiro (quadratura, 0,044 grau) esteve a mais
        # de 0,3 grau de onde terminou
        Tq = np.arange(t0, t1 - 0.05, 0.01)
        qq = amostra(d['getQuadDeg'], Tq)
        longe = Tq[np.abs(qq - qq[-1]) > 0.3]
        assenta = (longe.max() - t0) if len(longe) else 0.0
        res.append(dict(ang=round(CFG.norm(g)), sobe=sobe, pp=np.ptp(q[:, 1]), rms=np.std(x),
                        f=fr[k], dv=np.std(v), over=over, erro=np.mean(e[-10:]),
                        hf=hf, assenta=assenta))
    return res


def resumo(r):
    a = lambda k: np.array([x[k] for x in r])
    return ('tremor rapido %.3f (max %.3f) | pp %.2f (max %.2f) | assenta %.2fs (max %.2f) | '
            'overshoot %.1f (max %.1f) | erro final %.2f | n=%d' % (
                a('hf').mean(), a('hf').max(), a('pp').mean(), a('pp').max(), a('assenta').mean(),
                a('assenta').max(), a('over').mean(), a('over').max(), np.abs(a('erro')).mean(),
                len(r)))


if __name__ == '__main__':
    if sys.argv[1] == '-r':
        for f in sys.argv[2:]:
            print('%-26s %s' % (f.split('/')[-1].replace('.csv', ''), resumo(analisa(f))))
        sys.exit()
    for f in sys.argv[1:]:
        r = analisa(f)
        print('== ' + f.split('/')[-1])
        print('ang  dir | pp     rms    freq   std V | overshoot  erro')
        for a in r:
            print('%3d  %s  | %5.2f° %5.3f° %4.1fHz %5.3fV | %6.1f°  %+5.2f°' % (
                a['ang'], 'sobe' if a['sobe'] else 'desce', a['pp'], a['rms'], a['f'], a['dv'],
                a['over'], a['erro']))
        pp = np.array([a['pp'] for a in r]); rms = np.array([a['rms'] for a in r])
        print('RESUMO: pp medio %.2f° max %.2f° | rms medio %.3f° | tremendo (>0,5°): %d de %d | '
              'overshoot medio %.1f° max %.1f°' % (pp.mean(), pp.max(), rms.mean(),
              (pp > 0.5).sum(), len(pp), np.mean([a['over'] for a in r]),
              max(a['over'] for a in r)))
