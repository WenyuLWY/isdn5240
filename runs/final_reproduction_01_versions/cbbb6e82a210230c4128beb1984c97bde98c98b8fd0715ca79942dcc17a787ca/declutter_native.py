"""A-style, guarded end-face push after the exposed candidates exhaust their bilateral strips."""
from rig import *
from axes import observe,options,CAL,SupportGraph
from disassembly.robot_planning import fingers
from joint_path import route
class NoGrasp(RuntimeError):pass

def geometry(s):
    rows=observe(s);g=s.graph.update(s.m,s.d,rows,[f'brick_{i:03}' for i in s.accepted])
    table=s.d.geom('work_table');half_table=s.m.geom('work_table').size[:2];table_lo=table.xpos[:2]-half_table;table_hi=table.xpos[:2]+half_table
    live=s.d;out=[]
    for row in sorted((r for r in rows if r['brick_id'] in g['allowed']),key=lambda r:r['brick_id']):
        body=row['brick_id'];i=int(body[6:]);B=live.body(body).xmat.reshape(3,3);p=np.array(row['position_m']);sz=np.array(row['size_m'])
        long=sz.argmax();wide=sz.argmin();half=abs(B[:2])@(sz/2)
        if abs(B[2,wide])<.97 or not any(s.names[c.geom1]=='work_table' and s.names[c.geom2]==body+'_geom' or s.names[c.geom2]=='work_table' and s.names[c.geom1]==body+'_geom' for c in live.contact):continue
        for sign in [1,-1]:
            v=sign*B[:,long].copy();v[2]=0;v/=np.linalg.norm(v)
            d=mj.MjData(s.m);d.qpos[:]=live.qpos
            for step in range(1,17):
                length=.005*step;position=p+length*v
                if min(np.min(position[:2]-half-table_lo),np.min(table_hi-position[:2]-half))<.006:break
                d.qpos[s.brickq[i][:3]]=position;mj.mj_forward(s.m,d)
                hit=False
                for c in d.contact:
                    a,b=s.names[c.geom1],s.names[c.geom2]
                    if body+'_geom' in (a,b) and a.startswith('brick_') and b.startswith('brick_'):hit=True;break
                if hit:break
                if step<3 or step%2:continue
                rr=dict(row,position_m=position.tolist());s.d=d
                try:qualified=any(any(True for _ in options(s,rr,lean,int(wide))) for lean in (0,.25,-.25))
                finally:s.d=live
                if qualified:
                    out.append((length,i,v,p,B,sz));break
    if not out:raise NoGrasp('no supported end-face push with predicted opposed-finger passage')
    return sorted(out,key=lambda x:(x[0],x[1]))

def tip(s,jaw,frame,dir,top):
    m=s.m;d=mj.MjData(m);d.qpos[:]=s.d.qpos;fingers(m,d,jaw);mj.mj_kinematics(m,d)
    p,R=s.site(0,d);hulls=[]
    for k,name in enumerate(s.names):
        if not name.startswith(('left_','right_')) or not m.geom_contype[k]:continue
        if m.geom_type[k]==mj.mjtGeom.mjGEOM_MESH:
            a=m.geom_dataid[k];v=m.mesh_vert[m.mesh_vertadr[a]:m.mesh_vertadr[a]+m.mesh_vertnum[a]]
        else:v=np.array(list(itertools.product([-1,1],repeat=3)))*m.geom_size[k]
        v=((v@d.geom_xmat[k].reshape(3,3).T+d.geom_xpos[k]-p)@R)@frame.T;hulls.append((name,v))
    from scipy.spatial import ConvexHull
    best=None
    for depth in [.004,.0065,.008,.0095,.011]:
        zsite=top-depth-min(v[:,2].min() for name,v in hulls)
        leading=[]
        for name,points in hulls:
            if points[:,2].min()+zsite>top:continue
            below=points[points[:,2]+zsite<=top]
            candidates=[(below@dir).max()]
            if points[:,2].max()+zsite>top:
                hull=ConvexHull(points)
                for tri in hull.simplices:
                    for i,j in zip(tri,np.roll(tri,1)):
                        a,b=points[i],points[j]
                        if (a[2]+zsite-top)*(b[2]+zsite-top)<0:
                            v=a+(b-a)*(top-zsite-a[2])/(b[2]-a[2]);candidates.append(v@dir)
            leading.append((max(candidates),name))
        pad=max((x for x,n in leading if n.startswith(('left_pad','right_pad'))),default=-10.)
        other=max((x for x,n in leading if not n.startswith(('left_pad','right_pad'))),default=-10.)
        if pad-other>.0015:return zsite,pad,pad-other
    return None

def tool_frames(v):
    for angle in [0,18,32,-18,-32,46,-46]:
        pitch=Rot.from_rotvec(np.cross(UP,v)*np.deg2rad(angle)).as_matrix()
        for axis in [np.cross(v,UP),v,-v,-np.cross(v,UP)]:
            yield pitch@np.column_stack([axis,np.cross(-UP,axis),-UP])

def push(s):
    plans=geometry(s);live=s.d;failures=[]
    for length,i,v,center,B,sz in plans:
        s.i=i;row=observe(s)[i];jaw=min(CAL['samples'],key=lambda c:abs(c['opening_m']-.012))
        top=float(center[2]+.5*(abs(B[2])@sz));side=sz.argmax()
        for frame in tool_frames(v):
            witness=tip(s,jaw,frame,v,top)
            if witness is None:continue
            z,extent,pad_lead=witness
            touch=center-v*(sz[side]/2+extent);touch[2]=z
            start=touch-v*.008;pre=start+.085*UP
            for xy,yaw in s.stations(row):
                test=s.copy_station(xy,yaw);fingers(s.m,test,jaw);mj.mj_forward(s.m,test)
                if s.bad(test):continue
                s.d=test
                try:
                    j=s.iksolve(0,(pre,frame),s.goals[0],tries=5)
                    j2=s.iksolve(0,(start,frame),j,tries=4)
                    s.iksolve(0,(touch+v*length,frame),j2,tries=3)
                    route(s,0,j,max_seconds=.6)
                except IKError as e:failures.append(str(e));continue
                finally:s.d=live
                if s.travel_path(xy,yaw):break
            else:continue
            break
        else:continue
        break
    else:raise NoGrasp('no verified station for guarded push '+str(failures[-4:]))
    old={k:s.d.body(k).xpos.copy() for k in [f'brick_{j:03}' for j in range(30)]};target=f'brick_{i:03}';p0,R0=s.base_pose()
    s.event('declutter_plan',target=target,direction=v,stroke_m=length,contact_approach=start,tool_frame=frame,station=xy,pad_lead_m=pad_lead,method='guarded short-end push after both-side strip exhaustion')
    if np.linalg.norm(p0[:2]-xy)>.012 or abs((yaw-np.arctan2(R0[1,0],R0[0,0])+np.pi)%(2*np.pi)-np.pi)>.025:s.drive(xy,yaw,'reposition_clearance')
    s.move('prepare_clearance',.65,grip={0:jaw['command']})
    j=s.iksolve(0,(pre,frame),tries=12);path=route(s,0,j)
    for waypoint in path:s.move('prepare_clearance',max(1.1,1.25*np.max(abs(waypoint-s.goals[0]))),{0:[waypoint]})
    s.cart('approach_clearance',0,(start,frame),1.4)
    s.segment('clearance_ready');s.snapshot('clearance_ready')
    s.push_guard=dict(brick=target,old=old,peak=0.,pairs=set(),neighbor_m=0.)
    try:
        origin=start.copy()
        for dist in np.arange(.004,length+.029,.004):
            s.cart('push_clearance',0,(origin+dist*v,frame),.30)
            shift=s.d.body(target).xpos-old[target]
            if shift@v>=length-.0015:break
        else:raise NoGrasp(f'commanded push did not reach predicted passage, measured {shift@v:.4f} m')
        s.cart('withdraw_clearance',0,(s.site(0)[0]-.012*v,frame),.38)
        s.cart('withdraw_clearance',0,(s.site(0)[0]+.085*UP,frame),.8)
        s.move('settle_clearance',.30)
        s.event('decluttered',translation=s.d.body(target).xpos-old[target],peak_pad_force_N=s.push_guard['peak'],contact_entities=sorted(s.push_guard['pairs']),max_neighbor_move_m=s.push_guard['neighbor_m'])
    finally:s.push_guard=None
    s.move('open_for_pick',.55,grip={0:0})
    s.segment('declutter');s.snapshot('decluttered')

def monitor(s):
    guard=s.push_guard;target=guard['brick'];m=s.m;d=s.d
    for idx,c in enumerate(d.contact):
        pair=[s.names[c.geom1],s.names[c.geom2]]
        if not any(k.startswith('brick_') for k in pair) or not any(s.group(k)=='front' for k in pair):continue
        w=np.zeros(6);mj.mj_contactForce(m,d,idx,w)
        if w[0]<.10:continue
        hand=next(k for k in pair if s.group(k)=='front')
        if target+'_geom' not in pair or not hand.startswith(('left_pad','right_pad')):raise ContactError('unexpected declutter contact '+str((pair,w[0])))
        guard['pairs'].add(hand);guard['peak']=max(guard['peak'],float(w[0]))
        if w[0]>20:raise ContactError('declutter contact force '+str(w[0]))
    if s.tick%20==0:
        mx=max(np.linalg.norm(d.body(k).xpos-v) for k,v in guard['old'].items() if k!=target)
        guard['neighbor_m']=max(mx,guard['neighbor_m'])
        if mx>.0025:raise ContactError(f'declutter neighbor moved {mx:.5f} m')
