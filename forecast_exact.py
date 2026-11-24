from inspect_hold import *
from exact_strips import search
from axes import options
z=np.load(sys.argv[2]);seen=set(map(int,z['accepted']));rows=observe(s);all_boxes=s.strips.boxes[:]
while len(seen)<30:
 names=[f'brick_{k:03}' for k in seen];g=SupportGraph(s.m).update(s.m,s.d,rows,names)
 s.strips.boxes=[k for k in all_boxes if s.names[k]=='work_table' or int(s.names[k][6:9]) not in seen]
 sol=None
 for r in sorted((r for r in rows if r['brick_id'] in g['allowed']),key=lambda r:-r['position_m'][2]):
  R=s.d.body(r['brick_id']).xmat.reshape(3,3);fast=False
  for axis in [2]+[k for k in np.argsort(-abs(R[2])) if k!=2 and abs(R[2,k])>.34]:
   if any(any(True for _ in options(s,r,lean,int(axis))) for lean in [0,.18,-.18,.32,-.32]):fast=True;break
  if fast or search(s,r):sol=r;break
 if not sol:print('geometric stopping set',g['allowed'],flush=True);break
 print(sol['brick_id'], 'OBB' if fast else 'native shape',flush=True);seen.add(int(sol['brick_id'][6:]))
