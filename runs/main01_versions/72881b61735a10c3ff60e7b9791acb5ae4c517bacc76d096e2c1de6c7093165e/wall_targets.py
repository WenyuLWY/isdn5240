from rig import ROOT, clean
from baseline_input import wall,L,W,T
import json,numpy as np
def build():
 rows=[];boxes=[]
 for i,(p,R,k) in enumerate(wall()):
  p=p+[0,.65,0];half=abs(R[:2,:2])@np.array([L,W])/2;lo=p[:2]-half;hi=p[:2]+half
  expected={}
  for j,(a,b,layer) in enumerate(boxes):
   area=float(np.prod(np.maximum(0,np.minimum(hi,b)-np.maximum(lo,a))))
   if k==layer:assert area<1e-10
   if k==layer+1 and area>1e-10:expected[j]=area
  assert (not k or sum(expected.values())>.65*L*W) and (k<3 or len(expected)>=2)
  parking=[-.18,p[1]-.49+.25];assert parking[1]-.45>-.09
  rows.append(dict(slot=i,course=k+1,p=p,rotation=R,parking_xy=parking,underlap_m2=expected))
  boxes.append((lo,hi,k))
 assert len({tuple(r['p']) for r in rows})==30
 return rows
if __name__=='__main__':(ROOT/'targets_world.json').write_text(json.dumps(build(),default=clean,indent=1))
