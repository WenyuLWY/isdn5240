"""Refine A's conservative link bounding boxes with the calibrated native collision shapes."""
import itertools
import numpy as np
import mujoco as mj
from disassembly.oracle_strips import CAL
from disassembly.robot_planning import fingers,tool_pose
class NativeStrips:
    def __init__(self,s):
        self.m=s.m
        self.hand=np.array([i for i,n in enumerate(s.names) if s.m.geom_contype[i] and (n=='base_1' or n.startswith(('left_','right_')))])
        self.obstacles=np.array(s.strips.boxes)
        self.local={}
    def shapes(self,d,jaw):
        key=jaw['command']
        if key not in self.local:
            m=self.m;trial=mj.MjData(m);trial.qpos[:]=d.qpos;fingers(m,trial,jaw);mj.mj_kinematics(m,trial)
            p,R=tool_pose(m,trial);g=self.hand
            self.local[key]=((trial.geom_xpos[g]-p)@R,R.T@trial.geom_xmat[g].reshape(-1,3,3))
        return self.local[key]
    def candidates(self,s,row,axis,lean,opening,normal_sign,clipped=False,clearance=.105):
        m,d=self.m,s.d;size=np.array(row['size_m']);name=row['brick_id'];target=m.geom(name+'_geom').id
        Rb=d.body(name).xmat.reshape(3,3);normal=normal_sign*Rb[:,axis];jaw=opening
        boxes=s.strips.boxes;s.strips.boxes=[]
        try:
            all_fam=list(s.strips.families(d,row,dict(wide_axis=axis,normal=normal,entry_lean_rad=lean,insertion_aperture=jaw,pre_distance_m=clearance,allow_certified_contact_clipping=clipped)))
        finally:s.strips.boxes=boxes
        trial=mj.MjData(m);trial.qpos[:]=d.qpos;mj.mj_kinematics(m,trial)
        obs=np.array([k for k in boxes if k!=target]);cent=trial.geom_xpos[obs].copy();obrot=trial.geom_xmat[obs].reshape(-1,3,3);half=m.geom_size[obs];g=self.hand;pts=np.zeros(6)
        for closing,(audit,f) in all_fam:
            if not f or jaw['opening_m']<f['width']+.0025:continue
            R=f['frame'];O=f['grasp']-R@f['shift'];long=f['longitudinal']
            close=min(CAL['samples'],key=lambda c:abs(c['opening_m']-f['width']))
            commands=[x for x in CAL['samples'] if jaw['command']<=x['command']<=close['command']]
            if not commands:continue
            # Fixed-aperture insertion is linear. Distance is 1-Lipschitz in translation.
            stages=[(jaw,a,.00345) for a in np.linspace(0,clearance,1+int(np.ceil(clearance/.0055)))]
            stages +=[(c,0,.0027) for c in commands[1:]]
            geometry=[]
            for j,a,margin in stages:
                p,lr=self.shapes(d,j);geometry.append((O+p@R.T-a*R[:,2],R@lr,margin))
            def gap_at(x):
                lower=.030
                for xyz,mat,margin in geometry:
                    where=xyz+x*long;trial.geom_xpos[g]=where;trial.geom_xmat[g]=mat.reshape(-1,9)
                    # Distance from each hand bounding sphere to each obstacle OBB.
                    local=np.einsum('gok,okj->goj',where[:,None,:]-cent[None,:,:],obrot)
                    dist=np.linalg.norm(np.maximum(np.abs(local)-half,0.),axis=2)-m.geom_rbound[g,None]
                    for a,b in zip(*np.where(dist<margin+.0015)):
                        sep=mj.mj_geomDistance(m,trial,int(g[a]),int(obs[b]),margin+.0015,pts)
                        if sep<margin+.00035:
                            if getattr(self,'stats',None) is not None:self.stats[(m.geom(g[a]).name,m.geom(obs[b]).name,round(margin,5))]+=1
                            return None
                        lower=min(lower,sep-margin)
                return lower
            intervals=[];tested=0
            for lo,hi in audit['common_m']:
                offsets=sorted(np.linspace(lo+.0001,hi-.0001,max(3,int(np.ceil((hi-lo)/.0035))+1)),key=abs)
                for x in offsets:
                    tested+=1;gap=gap_at(x)
                    if gap is None:continue
                    radius=min(.0012,gap-.00015,x-lo,hi-x)
                    if radius<.00009:continue
                    intervals.append((float(x-radius),float(x+radius)))
                    if len(intervals)>=4:break
            if not intervals:continue
            centers=[(lo+hi)/2 for lo,hi in intervals]
            f.update(primary=centers[0],alternate=centers[-1],refined=True,clipped=clipped,normal_sign=float(normal_sign),world_axis=int(axis))
            audit.update(common_m=intervals,left_m=intervals,right_m=intervals,entry_axis=int(axis),entry_normal=normal.tolist(),full_pad_footprint=not clipped,
                         calibrated_open_command=jaw['command'],opening_m=jaw['opening_m'],centers_tested=tested,
                         method='native convex/box hand sweep on both sides and palm, followed by opposed physical pad contact')
            yield audit,f

def search(s,row):
    if not hasattr(s,'native_strips'):s.native_strips=NativeStrips(s)
    R=s.d.body(row['brick_id']).xmat.reshape(3,3);size=np.array(row['size_m'])
    # Geometry-only order. The vehicle position enters the execution planner afterwards.
    axes=sorted(range(3),key=lambda a:-abs(R[2,a]))
    for extended in (False,True):
        for axis in axes:
            up=R[2,axis];sgn=1 if up>=0 else -1
            signs=[sgn] if abs(up)>.25 else [sgn,-sgn]
            if not extended and abs(up)<.28:continue
            widths=sorted((size[c] for c in range(3) if c!=axis and size[c]<.08),reverse=True)
            if not widths:continue
            jaws=[s.strips.full]
            for width in widths:
                for gap in [.009,.004,.018]:
                    j=min(CAL['samples'],key=lambda j:abs(j['opening_m']-(width+gap)))
                    if j not in jaws:jaws.append(j)
            for sign in signs:
                for jaw in jaws:
                    angles=[(v,False) for v in [0,.28,-.28,.55,-.55]]
                    if extended:angles += [(v,True) for v in [.46,-.46,.77,-.77,1.03,-1.03]]
                    for angle,clipped in angles:
                        for a,f in s.native_strips.candidates(s,row,axis,angle,jaw,sign,clipped):return a,f
    return None
