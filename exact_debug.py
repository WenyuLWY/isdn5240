from inspect_hold import *
from exact_strips import NativeStrips
from collections import Counter
rows=observe(s);s.strips.boxes=[k for k in s.strips.boxes if s.names[k]!='brick_029_geom']
n=NativeStrips(s);n.stats=Counter()
for name in ['brick_006','brick_015','brick_022','brick_029']:
 r=next(x for x in rows if x['brick_id']==name);axis=2 if name!='brick_029' else 1;sign=np.sign(s.d.body(name).xmat.reshape(3,3)[2,axis]);n.stats.clear();v=list(n.candidates(s,r,axis,0,s.strips.full,sign));print(name,len(v),n.stats.most_common(7),flush=True)
