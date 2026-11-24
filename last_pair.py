from inspect_hold import *
from exact_strips import NativeStrips
from collections import Counter
s.strips.boxes=[k for k in s.strips.boxes if s.names[k] in ('work_table','brick_010_geom','brick_011_geom')]
native=NativeStrips(s);native.stats=Counter()
for target,other in [(10,11),(11,10)]:
 row=observe(s)[target];name=row['brick_id'];R=s.d.body(name).xmat.reshape(3,3);p=np.asarray(row['position_m']);b,B=s.brick(other);print(name, np.round(R.T@(b-p),5),np.round(R.T@B,3),flush=True)
 for w in [.085,.068,.062]:
  jaw=min(CAL['samples'],key=lambda j:abs(j['opening_m']-w))
  for a in [0,.6,-.6,1.0,-1.0]:
   native.stats.clear();v=list(native.candidates(s,row,2,a,jaw,1.,abs(a)>.2))
   if a in [0.,1.]:print(w,a,'=>',len(v),native.stats.most_common(3),flush=True)
