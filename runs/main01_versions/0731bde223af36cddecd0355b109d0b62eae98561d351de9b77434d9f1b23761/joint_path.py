"""Collision-checked bidirectional joint-space paths for the empty arm."""
import time
import numpy as np
import mujoco as mj
from rig import IKError,rel,compose,quat

def route(s, arm, goal, avoid_brick=True, interarm_margin=0.,max_seconds=None,carrying=False):
    q0=s.goals[arm].copy()
    q1=goal.copy()
    q1[::2]=q0[::2]+(q1[::2]-q0[::2]+np.pi)%(2*np.pi)-np.pi
    d=mj.MjData(s.m)
    d.qpos[:]=s.d.qpos
    m=s.m
    if interarm_margin:
        ids = [k for k in range(m.ngeom) if m.geom_contype[k]]
        aa,bb = np.array([(a,b) for a in ids if s.groups[a]=='front' for b in ids if s.groups[b]=='rear']).T
        radii = m.geom_rbound[aa]+m.geom_rbound[bb]+interarm_margin
        near = np.zeros(6)
    if carrying:
        payload=rel(s.site(arm),s.brick())
        active=f'brick_{s.i:03}_geom'
    def clear(q):
        d.qpos[s.q[arm]]=q
        mj.mj_kinematics(m,d)
        if carrying:
            pos,rot=compose(s.site(arm,d),payload)
            d.qpos[s.brickq[s.i]]=np.r_[pos,quat(rot)]
            mj.mj_kinematics(m,d)
        mj.mj_comPos(m,d)
        mj.mj_collision(m,d)
        if s.bad(d,arm):
            return False
        if interarm_margin:
            v=d.geom_xpos[aa]-d.geom_xpos[bb]
            for k in np.flatnonzero(np.einsum('ij,ij->i',v,v)<radii*radii):
                if mj.mj_geomDistance(m,d,aa[k],bb[k],interarm_margin,near)<interarm_margin-1e-8:
                    return False
        if avoid_brick:
            for c in d.contact:
                names=[s.names[c.geom1],s.names[c.geom2]]
                if carrying and active in names:
                    if s.group(names[1-names.index(active)])!=('front','rear')[arm]:return False
                    continue
                a,b=s.groups[c.geom1],s.groups[c.geom2]
                if {a,b}=={'brick',('front','rear')[arm]}:
                    return False
        return True
    def edge(a,b):
        n=max(1,int(np.ceil(np.max(abs(a-b))/(.025 if interarm_margin else .055))))
        return all(clear(a+(b-a)*i/n) for i in range(1,n+1))
    if edge(q0,q1):
        return [q1]
    if not clear(q1):
        raise IKError('empty-arm roadmap target collision')
    trees=[([q0],[-1]),([q1],[-1])]
    rng=np.random.default_rng(641)
    def grow(tree,target):
        nodes,parents=tree
        idx=int(np.argmin(np.linalg.norm(np.array(nodes)-target,axis=1)))
        delta=target-nodes[idx]
        dn=np.linalg.norm(delta)
        q=nodes[idx]+delta*min(1.,.55/max(1e-12,dn))
        if not edge(nodes[idx],q):
            return None
        nodes.append(q);parents.append(idx)
        return len(nodes)-1
    def back(tree,idx):
        out=[]
        while idx>=0:
            out.append(tree[0][idx])
            idx=tree[1][idx]
        return out
    began=time.perf_counter()
    for it in range(1300):
        if max_seconds is not None and time.perf_counter()-began>max_seconds:break
        a,b=trees[it%2],trees[(it+1)%2]
        t=rng.uniform(-np.pi,np.pi,7)+(q0+q1)/2
        t[[1,3,5]]=np.clip(t[[1,3,5]],[-2.23,-2.56,-2.08],[2.23,2.56,2.08])
        ia=grow(a,t)
        if ia is None:continue
        mid=a[0][ia]
        for _ in range(30):
            ib=grow(b,mid)
            if ib is None:break
            if np.linalg.norm(b[0][ib]-mid)>1e-6:continue
            pa,pb=back(a,ia),back(b,ib)
            out=pa[::-1]+pb[1:] if it%2==0 else pb[::-1]+pa[1:]
            smooth=[out[0]]
            j=0
            while j<len(out)-1:
                k=len(out)-1
                while k>j+1 and not edge(out[j],out[k]):k-=1
                smooth.append(out[k]);j=k
            return smooth[1:]
    raise IKError('empty-arm roadmap exhausted')
