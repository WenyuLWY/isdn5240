from scene import *
from disassembly.oracle_strips import Strips,CAL
from disassembly.oracle_support import SupportGraph
def observe(s):
    rows=[]
    for i in range(30):
        name=f'brick_{i:03}';b=s.d.body(name);rows.append(dict(brick_id=name,position_m=b.xpos.tolist(),quaternion_wxyz=b.xquat.tolist(),size_m=(2*s.m.geom(name+'_geom').size).tolist()))
    return rows

def options(s, row, lean=0):
    size=np.array(row['size_m']);axis=int(size.argmin());R=s.d.body(row['brick_id']).xmat.reshape(3,3);normal=R[:,axis].copy()
    if normal[2]<0:normal*=-1
    p=dict(wide_axis=axis,normal=normal,entry_lean_rad=lean,insertion_aperture=s.strips.full)
    for closing,audit_fam in s.strips.families(s.d,row,p):
        audit,fam=audit_fam
        if fam:
            yield audit,fam

def select(s):
    rows=observe(s);g=s.graph.update(s.m,s.d,rows,[f'brick_{i:03d}' for i in s.accepted]);nodes=set(g['allowed'])
    candidates=[r for r in rows if r['brick_id'] in nodes]
    # Geometric order, independent of base position. Prefer the exposed upper surface within a tilt band.
    def priority(r):
        R=s.d.body(r['brick_id']).xmat.reshape(3,3)
        return (-round(abs(R[2,int(np.argmin(r['size_m']))]),2),-r['position_m'][2],r['brick_id'])
    for r in sorted(candidates,key=priority):
        for lean in [0,.18,-.18,.32,-.32]:
            for audit,f in options(s,r,lean):
                return r,g,f,audit
    raise RuntimeError('no bilateral common strip amongst permitted endpoints')

class PileStrips(Strips):
    def __init__(self,model,data):
        self.model=model
        self.full=max(CAL['samples'],key=lambda x:x['opening_m'])
        self.boxes=[g for g in range(model.ngeom) if model.geom(g).name=='work_table' or model.geom(g).name.startswith('brick_')]
        self.apertures={}
