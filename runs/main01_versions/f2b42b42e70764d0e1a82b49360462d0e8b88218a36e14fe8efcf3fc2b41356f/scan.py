"""Evaluate SE(2) receiver candidates against the live held brick and donor.
The scan transforms projected shape bounds. Live state and payload stay physical.
"""
import numpy as np
import mujoco as mj
from clearance_check import Surfaces
from rig import A,T,rz

def scan(rig,points,off,face):
    m,d=rig.m,rig.d
    s=Surfaces(m)
    rear=[i for i in np.unique(s.a) if m.geom(i).name.startswith(('rear_arm__pump_section','rear_arm__lip','rear_arm__adapter','rear_arm__bracelet'))]
    front=np.unique(s.b)
    tool,R=rig.site(1)
    brick,B=rig.brick()
    B=B@face
    offsets=(d.geom_xpos[rear]-tool)@R
    local_rot=R.T@d.geom_xmat[rear].reshape(-1,3,3)
    candidates=[]
    nearest=None
    # A positive support-plane gap certifies separation for convex shapes.
    # Query fixed directions in world space, including brick axes and dense sphere normals.
    phi=np.arange(162)*2.3999632297
    z=1-2*(np.arange(162)+.5)/162
    dirs=np.vstack((B.T,np.column_stack((np.sqrt(1-z*z)*np.cos(phi),np.sqrt(1-z*z)*np.sin(phi),z))))
    dirs=np.vstack((dirs,-dirs))
    def intervals(g,R,p):
        axis=dirs@R
        size=m.geom_size[g]
        if g in s.verts:
            dots=s.verts[g]@axis.T
            return p@dirs.T+dots.min(axis=0),p@dirs.T+dots.max(axis=0)
        typ=m.geom_type[g]
        if typ==mj.mjtGeom.mjGEOM_CAPSULE:
            r=size[0]+np.abs(axis[:,2])*size[1]
        elif typ==mj.mjtGeom.mjGEOM_CYLINDER:
            r=size[0]*np.linalg.norm(axis[:,:2],axis=1)+np.abs(axis[:,2])*size[1]
        else:
            r=np.abs(axis)@size
        return p@dirs.T-r,p@dirs.T+r
    fmin=np.array([intervals(g,d.geom_xmat[g].reshape(3,3),d.geom_xpos[g])[0] for g in front])
    # Certify the pump against the swept donor envelope during jaw release.
    # The support function of a union is the extremum over the jaw samples.
    from disassembly.robot_planning import CAL,fingers
    trial=mj.MjData(m);trial.qpos[:]=d.qpos
    gap=np.linalg.norm(d.geom("left_pad1").xpos-d.geom("right_pad1").xpos)
    opening=max(CAL['samples'],key=lambda c:c['opening_m'])['opening_m']
    commands=[]
    for width in np.linspace(min(gap,opening),opening,6):
        jaw=min(CAL['samples'],key=lambda j:abs(j['opening_m']-width))
        if jaw['command'] in commands:continue
        commands.append(jaw['command']);fingers(m,trial,jaw);mj.mj_kinematics(m,trial)
        moved=[k for k,g in enumerate(front) if m.geom(g).name.startswith(('left_','right_'))]
        env=np.array([intervals(front[k],trial.geom_xmat[front[k]].reshape(3,3),trial.geom_xpos[front[k]])[0] for k in moved])
        fmin[moved]=np.minimum(fmin[moved],env)
    for yaw in range(0,360,15):
        RR=B@A@rz(yaw)
        rot=RR@local_rot
        pos=brick+offsets@RR.T
        rmax=np.array([intervals(g,r,p)[1] for g,r,p in zip(rear,rot,pos)])
        zero=fmin[None,:,:]-rmax[:,None,:]
        for _,_,x,y in points:
            dp=B@np.array([x,y,T/2])
            gaps=(zero-(dp@dirs.T)).max(axis=2)
            smallest=gaps.min()
            ia,ib=np.unravel_index(gaps.argmin(),gaps.shape)
            entity=[m.geom(rear[ia]).name,m.geom(front[ib]).name]
            if smallest>.0040:
                candidates.append((np.hypot(x,y)-.25*min(.020,smallest),-round(min(.020,smallest),5),x,y,yaw))
            elif nearest is None or smallest>nearest[0]:
                nearest=(smallest,entity,x,y,yaw)
    rig.event('rigid_scan',face_sign=face[2,2],survivors=len(candidates),best_rejected=nearest,point_count=len(points),roll_step_deg=15)
    return sorted(candidates)
