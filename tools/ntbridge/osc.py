import csv, sys
import numpy as np
# conta inversoes de sentido com amplitude > 0,7 grau (oscilacao); movimento normal nao inverte
x = [float(r['value']) for r in csv.DictReader(open(sys.argv[1])) if r['topic'].endswith('getQuadDeg')
     and r['value'] not in ('', None)]
if len(x) < 10:
    print(0); sys.exit()
ext = [x[0]]; sentido = 0
for v in x[1:]:
    dv = v - ext[-1]
    if sentido >= 0 and dv > 0 or sentido <= 0 and dv < 0:
        if sentido == 0 and abs(dv) < 0.7:
            continue
        sentido = 1 if dv > 0 else -1
        ext[-1] = v if len(ext) > 1 else ext[-1]
        if len(ext) == 1:
            ext.append(v)
    elif abs(dv) > 0.7:
        sentido = -sentido
        ext.append(v)
print(max(0, len(ext) - 2))
