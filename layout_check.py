"""Audit all thirty target rectangles, bounds, seams and positive overlap before running."""
from baseline_input import wall,L,W,T
import json
import numpy as np
from pathlib import Path

def audit():
    targets=wall()
    assert len(targets)==len({tuple(p) for p,R,k in targets})==30
    boxes=[]
    rows=[]
    for i,(p,R,k) in enumerate(targets):
        half=(abs(R)@np.array([L,W,T]))[:2]/2
        lo,hi=p[:2]-half,p[:2]+half
        support={}
        for j,(a,b,layer) in enumerate(boxes):
            span=np.maximum(0,np.minimum(hi,b)-np.maximum(lo,a))
            area=span.prod()
            if layer==k:
                assert area<1e-10,(i,j)
            if layer==k-1 and area>1e-10:
                support[j]=area
        assert k==0 or sum(support.values())>L*W*.65,(i,support)
        if k==3:
            assert len(support)>=2
        boxes.append((lo,hi,k))
        rows.append(dict(id=i,course=k+1,center_m=p.tolist(),wall_long_axis_world=R[:,0].tolist(),candidate_support_area_m2=support,ground=k==0))
    result=dict(course_counts=np.bincount([k for _,_,k in targets]).tolist(),targets=rows)
    assert result['course_counts']==[10,8,6,6]
    Path('layout30.json').write_text(json.dumps(result,indent=2))
if __name__=='__main__':
    audit()
