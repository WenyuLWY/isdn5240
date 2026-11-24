from inspect_hold import *
from axes import options
z=np.load(sys.argv[2]);ids=set(map(int,z['accepted']));rows=observe(s)
while len(ids)<30:
 g=SupportGraph(s.m).update(s.m,s.d,rows,[f'brick_{k:03}' for k in ids])
 s.strips.boxes=[k for k in s.strips.boxes if s.names[k]=='work_table' or int(s.names[k][6:9]) not in ids]
 allowed=sorted((r for r in rows if r['brick_id'] in g['allowed']),key=lambda r:-r['position_m'][2]);sol=None
 for r in allowed:
  R=s.d.body(r['brick_id']).xmat.reshape(3,3)
  for axis in [2]+[k for k in np.argsort(-abs(R[2])) if k!=2 and abs(R[2,k])>.34]:
   if any(any(True for _ in options(s,r,lean,int(axis))) for lean in [0,.18,-.18,.32,-.32]):sol=(r,axis);break
  if sol:break
 if not sol:print('no strip',g['allowed']);break
 print(sol[0]['brick_id'],sol[1],end=' ');ids.add(int(sol[0]['brick_id'][6:]))
