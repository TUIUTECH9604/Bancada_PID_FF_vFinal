import csv,collections,sys
d=collections.defaultdict(list)
for r in csv.DictReader(open(sys.argv[1],encoding='utf-8',errors='replace')):
    try:d[r['topic'].split('/')[-1]].append((float(r['t']),float(r['value'])))
    except:pass
def wrap(x): return (x+180)%360-180
tg=d['getTargetDeg']
segs=[];cur=None
for t,v in tg:
    if v!=v: cur=None; continue
    if cur is None or v!=cur[1]: cur=[t,v,t]; segs.append(cur)
    cur[2]=t
end=max(t for t,_ in d['getDeg'])
for i,s in enumerate(segs):
    if i+1<len(segs): s[2]=segs[i+1][0]
    else: s[2]=max(s[2],min(end,s[0]+4.2))
for s0,tgt,s1 in segs:
    pts=[(t,wrap(x-tgt)) for t,x in d['getDeg'] if s0<=t<=s1]
    if len(pts)<5: continue
    e=[p for _,p in pts]; tail=[p for t,p in pts if t>s1-1.0]
    v=[x for t,x in d['getCommandedVolts'] if s0<=t<=s1]
    print("alvo %5.1f (%.1fs): erro min %6.1f max %6.1f | ultimo 1s: medio %5.2f pp %5.2f | V max %.2f"%(tgt,s1-s0,min(e),max(e),sum(tail)/max(1,len(tail)),(max(tail)-min(tail)) if tail else -1,max(map(abs,v)) if v else 0))
