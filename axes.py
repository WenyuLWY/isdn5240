from scene import *
from disassembly.oracle_strips import Strips,CAL
from disassembly.oracle_support import SupportGraph
def observe(s):
    rows=[]
    for i in range(30):
        name=f'brick_{i:03}';b=s.d.body(name);rows.append(dict(brick_id=name,position_m=b.xpos.tolist(),quaternion_wxyz=b.xquat.tolist(),size_m=(2*s.m.geom(name+'_geom').size).tolist()))
    return rows

def options(s,row,lean=0,entry_axis=None):
    size=np.array(row['size_m']);axis=int(size.argmin()) if entry_axis is None else entry_axis;R=s.d.body(row['brick_id']).xmat.reshape(3,3);normal=R[:,axis].copy()
    if normal[2]<0:normal*=-1
    p=dict(wide_axis=axis,normal=normal,entry_lean_rad=lean,insertion_aperture=s.strips.full,pre_distance_m=.105)
    for closing,audit_fam in s.strips.families(s.d,row,p):
        audit,fam=audit_fam
        if fam:
            audit.update(entry_axis=axis,entry_normal=normal.tolist(),full_pad_footprint=True);yield audit,fam

def select(s):
    rows=observe(s);g=s.graph.update(s.m,s.d,rows,[f'brick_{i:03d}' for i in s.accepted]);nodes=set(g['allowed'])
    candidates=[r for r in rows if r['brick_id'] in nodes]
    # Geometric order, independent of base position. Prefer the exposed upper surface within a tilt band.
    def priority(r):
        R=s.d.body(r['brick_id']).xmat.reshape(3,3)
        return (-round(abs(R[2,int(np.argmin(r['size_m']))]),2),-r['position_m'][2],r['brick_id'])
    for r in sorted(candidates,key=priority):
        R=s.d.body(r['brick_id']).xmat.reshape(3,3)
        wide=int(np.argmin(r['size_m']))
        # An edge-standing brick exposes its long narrow face upwards. The
        # opposed move then closes across thickness with a complete common strip.
        axes=[wide]+[k for k in np.argsort(-np.abs(R[2])) if k!=wide and abs(R[2,k])>.34]
        for axis in axes:
            for lean in [0,.18,-.18,.32,-.32]:
                for audit,f in options(s,r,lean,int(axis)):
                    return r,g,f,audit
        from exact_strips import search
        refined=search(s,r)
        if refined:
            audit,f=refined;f['certificate']=audit
            return r,g,f,audit
    from declutter_native import NoGrasp
    raise NoGrasp('no bilateral common strip amongst permitted endpoints')

class PileStrips(Strips):
    def __init__(self,model,data):
        self.model=model
        self.full=max(CAL['samples'],key=lambda x:x['opening_m'])
        self.boxes=[g for g in range(model.ngeom) if model.geom(g).name=='work_table' or model.geom(g).name.startswith('brick_')]
        self.apertures={}
