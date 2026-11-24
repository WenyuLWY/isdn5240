"""Read-only native geometry audit around an actual guarded clearance intervention."""
from integrated import *
from exact_strips import NativeStrips
from collections import Counter
from axes import CAL
import sys,tempfile
run=Path(sys.argv[1]);out=Path(sys.argv[2]);ev=[json.loads(x) for x in (run/'events.jsonl').read_text().splitlines()]
blocked=next(x for x in ev if x['phase']=='strip_exhausted');t=blocked['time'];active=blocked['graph']['allowed']
ck=next(f for f in (run/'snapshots').glob('*_returned.npz') if abs(float(np.load(f)['time'])-t)<1e-7)
with tempfile.TemporaryDirectory() as tmp:
 dest=Path(tmp);(dest/'scene.xml').symlink_to((run/'scene.xml').resolve());s=object.__new__(Integrated);s.reopening=True;Integrated.__init__(s,dest)
 z=np.load(ck);mj.mj_setState(s.m,s.d,z['integration'],mj.mjtState.mjSTATE_INTEGRATION);s.m.eq_data[:]=z['eq_data'];s.fwd();s.accepted=list(map(int,z['accepted']))
 rows=observe(s);assert len(active)==2
 q=NativeStrips(s);rejections={};results={}
 for name in active:
    row=next(x for x in rows if x['brick_id']==name);B=s.d.body(name).xmat.reshape(3,3);q.stats=Counter();tested=0;passes=[]
    for axis in [2,0,1]:
     sign=1 if B[2,axis]>=0 else -1
     for sg in ([sign] if abs(B[2,axis])>.28 else [sign,-sign]):
      for roll in [False,True]:
       for cmd in [85,80,75,150,155,165,0]:
        jaw=next(x for x in CAL['samples'] if x['command']==cmd)
        for lean in [0,.4,-.4,.8,-.8,1.10,-1.10,1.35,-1.35]:
         tested+=1
         for audit,f in q.candidates(s,row,axis,lean,jaw,sg,abs(lean)>.25,tight=True,roll=roll):
          passes.append(dict(axis=axis,sign=sg,roll=roll,command=cmd,lean=lean,common=audit['common_m']))
    significant=[(k[0],k[1],v) for k,v in q.stats.most_common(10)]
    results[name]=dict(variants=tested,certified_grips=passes,most_frequent_rejected_shape_pairs=significant)
 a,b=(s.m.geom(n+'_geom').id for n in active);distance=float(mj.mj_geomDistance(s.m,s.d,a,b,.1,None))
after=next(x for x in ev if x['phase']=='decluttered' and x['time']>t)
selected=next(x for x in ev if x['phase']=='select' and x['time']>=after['time'])
out.write_text(json.dumps(dict(before_time=t,checkpoint=ck.name,permitted=active,initial_surface_gap_m=distance,
  exhaustive_within_sampled_families=results,actual_push=after,after_time=selected['time'],after_target=selected['target'],after_common_strip_m=selected['strip']['common_m']),indent=1))
print('passage audit',distance,{k:len(v['certified_grips']) for k,v in results.items()})
