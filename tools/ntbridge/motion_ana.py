import csv,collections,sys,bisect
d=collections.defaultdict(list)
for r in csv.DictReader(open(sys.argv[1])):
    try:d[r['topic'].split('/')[-1]].append((float(r['t']),float(r['value'])))
    except:pass
def at(k,t):
    a=d[k]; ts=[x for x,_ in a]; i=max(0,bisect.bisect_right(ts,t)-1); return a[i][1] if a else float('nan')
def wrap(x): return (x+180)%360-180
g=[(t,v) for t,v in d['getGoalDeg'] if v==v]
segs=[]
for t,v in g:
    if not segs or segs[-1][1]!=v: segs.append([t,v])
for i,(t0,goal) in enumerate(segs[1:],1):
    t1=segs[i+1][0] if i+1<len(segs) else t0+3
    moving=[(t,e) for t,e in d['getErrorDeg'] if t0<=t<t1 and abs(wrap(at('getTargetDeg',t)-goal))>0.01]
    tdone=max([t for t,_ in moving],default=t0)
    after=[wrap(x-goal) for t,x in d['getDeg'] if tdone<t<t1]
    tail=[wrap(x-goal) for t,x in d['getDeg'] if t1-1<t<t1]
    sgn=1 if wrap(goal-segs[i-1][1])>0 else -1
    over=max([sgn*e for e in after],default=0)
    vmax=max([abs(v) for t,v in d['getCommandedVolts'] if t0<=t<t1],default=0)
    print('-> %5.1f: movimento %.2fs | erro de seguimento max %5.2f° | overshoot %5.2f° | erro final %5.2f° pp %4.2f | V max %.2f'%(goal,tdone-t0,max([abs(e) for _,e in moving],default=0),max(0,over),sum(tail)/max(1,len(tail)),(max(tail)-min(tail)) if tail else 0,vmax))
