from demo import Demo,HeldBrick
from rig import *
from baseline_input import wall
from axes import *
from disassembly.mobile_ranger import wheel_reference,ROOT as FREE
from disassembly.robot_model import tool_pose
from joint_path import route
from vehicle_margin import Margin
import argparse,traceback,os
from collections import defaultdict

BASE0=np.array([-.18,.25,.31])
WALL_SHIFT=np.array([0,.65,0])
def data_json(path,obj):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(obj,default=clean,indent=1));tmp.replace(path)
class Integrated(Demo):
    def __init__(self,out):
        super().__init__(out,30)
        self.targets=[(p+WALL_SHIFT,R,k) for p,R,k in wall()]
        self.strips=PileStrips(self.m,self.d);self.graph=SupportGraph(self.m)
        self.brickq=np.array([self.m.jnt_qposadr[self.m.body_jntadr[b]]+np.arange(7) for b in self.bodyids])
        self.home_joints=None;self.picked=[];self.clock=defaultdict(float);self.samples=[];self.cycle_wall=time.monotonic()
        self.basebody=self.m.body('ranger_base_link').id
        self.carj=self.m.joint(FREE).id;self.carq=self.m.jnt_qposadr[self.carj];self.carv=self.m.jnt_dofadr[self.carj]
        self.margin=Margin(self.m);self.escaping=False;self.phase='start';self.driving=False;self.rejected_physical=set();self.last_pick_key=None
    def base_pose(self,d=None):
        d=self.d if d is None else d
        b=d.body(self.basebody);return b.xpos.copy(),b.xmat.reshape(3,3).copy()
    def on_vehicle(self,pose):
        p,R=self.base_pose();S=R@rz(-90)
        return p+S@(pose[0]-BASE0),S@pose[1]
    def home(self,a):
        return self.on_vehicle((np.array([.20,0 if a==0 else .49,.66 if a==0 else .70]),rz(78 if a==0 else 90)@A))
    def start(self):
        # Only setup the initial robot; original bricks were loaded from the A snapshot in the MJCF.
        for j,k in enumerate(json.loads((self.out/'source.json').read_text())['bricks']):
            adr=self.m.jnt_dofadr[self.m.body_jntadr[self.bodyids[j]]];self.d.qvel[adr:adr+6]=k['velocity']
        super().start()
        self.move('ready',.4,grip={0:0})
        self.record('overview')
    def bad(self,d,arm=None):
        hit=super().bad(d,arm)
        if hit:return hit
        for c in d.contact:
            n1,n2=self.names[c.geom1],self.names[c.geom2]
            if 'work_table' in (n1,n2):
                other=n2 if n1=='work_table' else n1
                if self.escaping and other=='ranger_base_collision' and c.dist>-.0018:continue
                if self.group(other) in ('front','rear','base'):return n1,n2,float(c.dist)
        return None
    @staticmethod
    def group(name):
        if name=='work_table':return 'fixture'
        return Demo.group(name)
    def step(self,phase):
        self.phase=phase
        super().step(phase)
        if phase=='wall_clear' and self.tick%5==0 and any(self.group(x)=='front' for x in self.hits()):
            raise ContactError("donor contacted its former payload during empty retreat")
        if self.driving and self.tick%10==0:
            for c in self.d.contact:
                a,b=self.groups[c.geom1],self.groups[c.geom2]
                if 'brick' in (a,b) and {a,b}&{'front','rear','base'}:
                    if self.vac.on and any(self.names[k]==f'brick_{self.i:03}_geom' for k in [c.geom1,c.geom2]) and 'rear' in (a,b):continue
                    raise ContactError(f'travel {self.names[c.geom1]} versus {self.names[c.geom2]}')
    def move(self,phase,*args,**kw):
        t=time.perf_counter();sim=self.d.time
        try:return super().move(phase,*args,**kw)
        finally:self.clock[phase]+=time.perf_counter()-t;self.clock[phase+'_sim']+=self.d.time-sim
    def copy_station(self,xy,yaw):
        m=self.m;d=mj.MjData(m);d.qpos[:]=self.d.qpos
        bp,R=self.base_pose();heading=np.arctan2(R[1,0],R[0,0]);shift=Rot.from_euler('z',yaw-heading).as_matrix()
        d.qpos[self.carq:self.carq+7]=np.r_[xy,bp[2],quat(shift@R)]
        if self.vac.on:
            b,B=self.brick();new=np.r_[xy,bp[2]]+shift@(b-bp);d.qpos[self.brickq[self.i]]=np.r_[new,quat(shift@B)]
        mj.mj_forward(m,d);return d
    def drive_clear(self,xy,yaw):
        bp,R=self.base_pose();ang=np.arctan2(R[1,0],R[0,0]);delta=(yaw-ang+np.pi)%(2*np.pi)-np.pi
        certified=min(.025,self.margin.gap(self.d))
        for f in np.linspace(0,1,2+int(np.linalg.norm(xy-bp[:2])/.035+abs(delta)/.07)):
            d=self.copy_station((1-f)*bp[:2]+f*xy,ang+f*delta)
            err=self.bad(d)
            if err:return err
            gap=self.margin.gap(d)
            if gap<certified-.00015:return ("work_table","vehicle safety clearance")
            certified=max(certified,min(.025,gap))
            for c in d.contact:
                ab=[self.groups[c.geom1],self.groups[c.geom2]]
                if 'brick' in ab and set(ab)&{'base','front','rear'}:
                    if self.vac.on and 'rear' in ab and f'brick_{self.i:03d}_geom' in [self.names[c.geom1],self.names[c.geom2]]:continue
                    return self.names[c.geom1],self.names[c.geom2]
        return None
    def drive_leg(self,xy,yaw,phase):
        xy=np.asarray(xy);start,R=self.base_pose();why=self.drive_clear(xy,yaw)
        if why:raise ContactError(f'blocked vehicle corridor {why}')
        t=time.perf_counter();s0=self.d.time
        anchor=rel(self.site(1),self.brick()) if self.vac.on else None
        self.driving=True;vel=np.zeros(2);stable=0;latched=False;h=self.m.opt.timestep
        try:
            for k in range(int((6+np.linalg.norm(start[:2]-xy)/.10)/h)):
                if k%6==0:
                    p,R=self.base_pose();err=xy-p[:2];dist=np.linalg.norm(err);angle_err=(yaw-np.arctan2(R[1,0],R[0,0])+np.pi)%(2*np.pi)-np.pi
                    if dist<.006 and abs(angle_err)<.016:latched=True
                    if dist>.012 or abs(angle_err)>.028:latched=False
                    desired=np.zeros(2) if latched else err/max(dist,1e-9)*min(.32,max(.035,2.5*dist))
                    vel+=np.clip(desired-vel,-.65*h*6,.65*h*6)
                    if latched:vel[:]=0
                    wheel_reference(self.m,self.d,vel,np.clip(2*angle_err,-.35,.35) if abs(angle_err)>.008 and not latched else 0.,steer_only=k<160)
                    stable=stable+6*h if latched and dist<.012 and abs(angle_err)<.028 and np.linalg.norm(self.d.qvel[self.carv:self.carv+2])<.018 else 0
                    if stable>.09:break
                self.step(phase)
            else:raise RuntimeError(f'drive timeout {phase}: error {dist} yaw {angle_err} actual {p} goal {xy}')
        finally:
            self.driving=False
            for n in ['fl','fr','rl','rr']:self.d.ctrl[self.m.actuator(n+'_native_roll').id]=0
            self.fwd()
        p,R=self.base_pose();info=dict(start=start,end=p,distance=np.linalg.norm(p[:2]-start[:2]),error_xy=np.linalg.norm(p[:2]-xy),elapsed_sim=self.d.time-s0)
        if anchor:
            actual=rel(self.site(1),self.brick());info.update(slip_m=np.linalg.norm(actual[0]-anchor[0]),slip_deg=angle(actual[1]@anchor[1].T))
            if info['slip_m']>.005 or info['slip_deg']>6:raise ContactError('slip on wheels')
        self.event(phase,**info);self.clock[phase]+=time.perf_counter()-t;self.clock[phase+'_sim']+=self.d.time-s0
    def travel_path(self,xy,yaw):
        xy=np.asarray(xy);start,R=self.base_pose();h=np.arctan2(R[1,0],R[0,0])
        if not self.drive_clear(xy,yaw):return [(xy,yaw)]
        # Bypass the measured table footprint on its north or west side.
        paths=[[(np.array([start[0],.55]),h)],[(np.array([-.52,start[1]]),h),(np.array([-.52,xy[1]]),yaw)],[(np.array([start[0],.55]),h),(np.array([1.30,.55]),yaw),(np.array([1.30,xy[1]]),yaw)],[(np.array([-.52,.52]),h),(np.array([-.52,-1.03]),yaw),(np.array([xy[0],-1.03]),yaw)]]
        live=self.d
        for prefix in paths:
            result=prefix+[(xy,yaw)]
            try:
                for q,heading in result:
                    if self.drive_clear(q,heading):break
                    self.d=self.copy_station(q,heading)
                else:return result
            finally:self.d=live
        return None
    def drive(self,xy,yaw,phase):
        steps=self.travel_path(xy,yaw)
        if not steps:raise ContactError(f"no collision-free vehicle route to {xy}")
        for k,(point,heading) in enumerate(steps):
            self.drive_leg(point,heading,phase if k==len(steps)-1 else phase+"_waypoint")
    def clear_table_corner(self):
        contacts=[]
        for c in self.d.contact:
            names=[self.names[c.geom1],self.names[c.geom2]]
            if set(names)=={'work_table','ranger_base_collision'}:
                contacts.append(c.frame[:2].copy()*(1 if names[0]=='work_table' else -1))
        if not contacts:return
        normal=np.mean(contacts,axis=0);normal/=np.linalg.norm(normal)
        start=self.base_pose()[0];min_dist=0.;self.escaping=True
        try:
            for k in range(3300):
                if k%5==0:wheel_reference(self.m,self.d,.083*normal,0.,steer_only=k<130)
                self.step('clear_chassis')
                if k%10==0:
                    self.fwd();gap=self.margin.gap(self.d)
                    if gap>.035:break
                    for c in self.d.contact:
                        if set([self.names[c.geom1],self.names[c.geom2]])=={'work_table','ranger_base_collision'}:min_dist=min(min_dist,c.dist)
            else:raise ContactError('vehicle could not separate from table corner')
        finally:
            self.escaping=False
            for n in ['fl','fr','rl','rr']:self.d.ctrl[self.m.actuator(n+'_native_roll').id]=0
        self.move('parking_settle',.32)
        self.event('physical_corner_recovery',start=start,end=self.base_pose()[0],clearance_m=self.margin.gap(self.d),deepest_contact_m=min_dist)
    def drive_wall(self,target):
        self.drive(np.array([BASE0[0],target[1]-.49+BASE0[1]]),np.pi/2,'drive_wall')
    def approach_plan(self,f,offset,flip):
        R=f['frame']@rz(180 if flip else 0)
        O=f['grasp']+offset*f['longitudinal']-R@f['shift']
        tool_to_site=rel(tool_pose(self.m,self.d),self.site(0));target=compose((O,R),tool_to_site)
        pre=(target[0]-.105*target[1][:,2],target[1]);j=self.iksolve(0,pre,self.goals[0],tries=5)
        js=[];seed=j
        for u in [.4,.72,1.0]:
            seed=self.iksolve(0,(pre[0]*(1-u)+target[0]*u,target[1]),seed,tries=2);js.append(seed)
            self.ik.qpos[:]=self.d.qpos;self.ik.qpos[self.q[0]]=seed;mj.mj_forward(self.m,self.ik)
            for c in self.ik.contact:
                names=[self.names[c.geom1],self.names[c.geom2]]
                if 'front' in [self.groups[c.geom1],self.groups[c.geom2]] and any(x.startswith('brick_') for x in names):
                    if f'brick_{self.i:03d}_geom' in names and any(x.startswith(('left_pad','right_pad')) for x in names):continue
                    raise IKError('ingress '+str(names))
        path=route(self,0,j)
        return pre,target,path,js
    def stations(self,r):
        p,R=self.base_pose();yield p[:2],np.arctan2(R[1,0],R[0,0])
        x,y,_=r['position_m']
        # Table's north and west edges; yaw lets the donor face the locked target.
        for cy in [.46,.51]:
            for cx in [x-.15,x,x+.12,x-.30]:
                yield np.array([cx,cy]),np.pi/2
        for cx in [-.31,-.40,1.20,1.28]:
            heading=np.pi if cx<0 else 0
            for cy in [y,y+.16,y-.15]:
                yield np.array([cx,cy]),heading
        for cy in [-1.00,-1.08]:
            for cx in [x+.12,x,x-.14]:
                yield np.array([cx,cy]),-np.pi/2
    def pick_plan(self,row,f):
        failures=[];base_live=self.d
        for xy,yaw in self.stations(row):
            probe=self.copy_station(xy,yaw)
            from disassembly.robot_planning import fingers
            jaw=f['insertion_aperture']
            fingers(self.m,probe,jaw);mj.mj_forward(self.m,probe)
            collision=self.bad(probe)
            if collision:failures.append(str(collision));continue
            self.d=probe
            try:
                for lean in ([0] if f.get("refined") else [0,.18,-.18,.32,-.32]):
                    families=[(f["certificate"],f)] if f.get("refined") else options(self,row,lean,f["broad"])
                    for audit,family in families:
                        offsets=[family['primary'],family['alternate']]
                        for lo,hi in audit['common_m']:
                            inset=min(.001,(hi-lo)/3)
                            offsets.extend([np.clip(0,lo+inset,hi-inset),(lo+hi)/2])
                        for offset in dict.fromkeys(x for x in offsets if x is not None):
                            for flip in ([False] if f.get("refined") else [False,True]):
                                signature=(row['brick_id'],tuple(np.round(np.asarray(row['position_m'])/.003).astype(int)),tuple(np.round(xy/.03).astype(int)),round(yaw,1),round(lean,2),round(float(offset),3),flip)
                                if signature in self.rejected_physical:continue
                                try:pre,target,path,js=self.approach_plan(family,offset,flip)
                                except IKError as e:failures.append(str(e));continue
                                self.d=base_live
                                drivable=self.travel_path(xy,yaw)
                                self.d=probe
                                if drivable:
                                    self.last_pick_key=signature
                                    return xy,yaw,family,offset,flip
                                failures.append("blocked drive corridor");break
            finally:self.d=base_live
        self.event('pick_rejections',reasons=failures);raise IKError(f'no qualified station for {row["brick_id"]}: {failures[-3:]}')
    def grasp(self,row,f):
        self.i=int(row['brick_id'][6:]);self.phase="plan_grasp";t=time.monotonic()
        if self.d.ctrl[self.fingers[0]]>3:self.move('open_for_pick',.50,grip={0:0})
        xy,yaw,f,offset,flip=self.pick_plan(row,f)
        self.event('station_plan',xy=xy,yaw=yaw,offset=offset,flip=flip,wall_s=time.monotonic()-t)
        p,R=self.base_pose()
        if np.linalg.norm(p[:2]-xy)>.012 or abs((yaw-np.arctan2(R[1,0],R[0,0])+np.pi)%(2*np.pi)-np.pi)>.025:self.drive(xy,yaw,'reposition')
        opening=f['insertion_aperture']['command']
        if opening:
            self.move('calibrated_open',.55,grip={0:opening})
            self.event('calibrated_open',command=opening,nominal_m=f['insertion_aperture']['opening_m'],measured_pad_center_separation_m=np.linalg.norm(self.d.geom('left_pad1').xpos-self.d.geom('right_pad1').xpos))
        pre,target,path,js=self.approach_plan(f,offset,flip)
        self.move('approach',1.2+.4*(len(path)-1),{0:path},grip={0:opening});self.move('align',.16)
        self.cart('insert',0,target,.85);self.move('close',.48,grip={0:255});self.move('close',.18)
        self.event('closed',contacts=self.hits(),tool=self.site(0),brick=self.brick())
        if not self.bilateral(0):raise ContactError('no opposed pad contact '+str(self.hits()))
        prior=rel(self.site(0),self.brick());start=self.brick()[0];p,R=self.site(0)
        self.cart('test_lift',0,(p+.045*UP,R),.52);self.move('test_lift',.12)
        curr=rel(self.site(0),self.brick());dz=self.brick()[0][2]-start[2]
        if dz<.032 or not self.bilateral(0) or np.linalg.norm(prior[0]-curr[0])>.009:raise ContactError(f'failed test lift {dz} {self.hits()}')
        self.picked.append(self.i);self.event('picked',dz=dz,brick_in_tool=curr,contacts=self.hits(),front_joints=self.d.qpos[self.q[0]])
        return self.deliver_held()

    def held_path(self,phase,arm,goal):
        path=self.cart_path(phase,arm,goal,tries=12)
        # Only the planning copy carries the measured rigid offset during FK sweeps.
        d=mj.MjData(self.m);d.qpos[:]=self.d.qpos
        anchor=rel(self.site(arm),self.brick());active=f'brick_{self.i:03}_geom'
        curve=PchipInterpolator(np.linspace(0.,1.,len(path)+1),np.vstack([self.goals[arm],path]),axis=0)
        separating={}
        if phase=="extract":
            for c in self.d.contact:
                names=(self.names[c.geom1],self.names[c.geom2])
                if active in names:
                    other=names[1-names.index(active)]
                    if other.startswith("brick_") and c.dist>-.0015:
                        separating[other]=min(separating.get(other,0.),float(c.dist))
        if separating:curve=PchipInterpolator(np.linspace(0.,1.,len(path)+1),np.vstack([self.d.qpos[self.q[arm]],path]),axis=0)
        last_seen=set(separating)
        for u in np.linspace(0.,1.,61):
            seen=set()
            d.qpos[self.q[arm]]=curve(u);mj.mj_kinematics(self.m,d)
            pos,rot=compose(self.site(arm,d),anchor);d.qpos[self.brickq[self.i]]=np.r_[pos,quat(rot)]
            mj.mj_kinematics(self.m,d);mj.mj_comPos(self.m,d);mj.mj_collision(self.m,d)
            why=self.bad(d,arm)
            if why:raise IKError('payload sweep '+str(why))
            for c in d.contact:
                names=[self.names[c.geom1],self.names[c.geom2]]
                if active in names:
                    other=names[1-names.index(active)]
                    if self.group(other)!=('front','rear')[arm]:
                        if other in last_seen and c.dist>=separating[other]-.0003 and u<.7:seen.add(other)
                        else:raise IKError('payload collision '+str((names,round(u,3),round(c.dist,6),list(last_seen))))
                if self.group(names[0])==('front','rear')[arm] and names[1].startswith('brick_') and active!=names[1] or self.group(names[1])==('front','rear')[arm] and names[0].startswith('brick_') and active!=names[0]:
                    raise IKError('finger swept neighbor '+str(names))
            last_seen=seen
        return path

    def deliver_held(self):
        p,R=self.site(0);shoulder=self.d.body("shoulder_link").xpos
        inward=shoulder-p;inward[2]=0;inward/=max(np.linalg.norm(inward),1e-9)
        self.phase='extract_plan'
        failures=[]
        deltas=[.11*UP,.080*UP+.065*inward,.055*UP+.080*inward,.045*UP+.04*inward]
        active=f'brick_{self.i:03}_geom';normals=[]
        for c in self.d.contact:
            pair=[self.names[c.geom1],self.names[c.geom2]]
            if active in pair and pair[1-pair.index(active)].startswith("brick_"):
                normals.append(c.frame[:3].copy()*(1 if pair[1]==active else -1))
        if normals:
            direction=np.mean(normals,axis=0);direction/=max(1e-9,np.linalg.norm(direction))
            deltas=[.080*UP+.045*direction,.035*UP+.070*direction]+deltas
        for delta in deltas:
            try:path=self.held_path('extract',0,(p+delta,R));break
            except IKError as e:failures.append(str(e))
        else:raise IKError('extraction sweep failed '+str(failures))
        self.move('extract',.85,{0:path})
        self.phase='present_ready_plan'
        path=self.held_path('present_ready',0,self.home(0))
        self.move('present_ready',1.9,{0:path})
        self.move('present_ready',.20)
        if not self.bilateral(0):raise ContactError('lost front hold before handoff')
        h=HeldBrick(self.i,2*self.m.geom(f'brick_{self.i:03}_geom').size.copy(),self.brick(),self.d.qpos[self.q[0]].copy(),rel(self.site(0),self.brick()),self.hits())
        from dataclasses import asdict
        self.event('A_input',payload=asdict(h));return h

    def grasp_feedback(self,row,family):
        self.last_pick_key=None
        for attempt in range(3):
            try:return self.grasp(row,family)
            except (IKError,ContactError) as e:
                if self.i in self.picked or not self.last_pick_key or attempt==2:raise
                hits=self.hits()
                at_rest=any(n in ("work_table","ground") or n.startswith("brick_") for n in hits)
                jaw_contact=any(self.group(n)=='front' for n in hits)
                if jaw_contact and not at_rest:raise
                self.rejected_physical.add(self.last_pick_key)
                self.event("grasp_retry",error=str(e),attempt=attempt+1,contacts=hits)
                self.move("open_retry",.38,grip={0:0})
                p,R=self.site(0)
                self.cart("retreat_retry",0,(p-.075*R[:,2],R),.8)
                self.move("settle_retry",.2)
                rows=observe(self);g=self.graph.update(self.m,self.d,rows,[f"brick_{i:03}" for i in self.accepted])
                row=next(r for r in rows if r["brick_id"]==f"brick_{self.i:03}")
                if row["brick_id"] not in g["allowed"]:raise ContactError("new observation changed the locked target support")
                self.last_pick_key=None
        raise ContactError("bounded grasp retries exhausted")

    def snapshot(self,label):
        p=self.out/'snapshots';p.mkdir(exist_ok=True)
        path=p/f'{len(self.accepted):02d}_{self.i:03}_{label}.npz'
        spec=mj.mjtState.mjSTATE_INTEGRATION;integ=np.empty(mj.mj_stateSize(self.m,spec));mj.mj_getState(self.m,self.d,integ,spec)
        tmp=path.with_suffix('.tmp.npz');np.savez_compressed(tmp,integration=integ,eq_data=self.m.eq_data,goals=self.goals,home=self.home_joints,accepted=self.accepted,picked=self.picked,transfers=self.transfers,active=self.i,stage=label,time=self.d.time,tick=self.tick,trace_n=len(self.frames),event_n=len(self.events),vac_on=self.vac.on,anchor=getattr(self.vac,'local_anchor',[]),axes=getattr(self.vac,'initial_axes',[]),sign=getattr(self.vac,'face_sign',0));tmp.replace(path)
        data_json(self.out/'status.json',dict(stage=label,target=self.i,sim_time=self.d.time,picked=self.picked,transfers=self.transfers,placed=self.accepted,snapshot=str(path.resolve())))
        return path
    def segment(self,label):
        p=self.out/'segments';p.mkdir(exist_ok=True);path=p/f'{len(list(p.glob("*.npz"))):03}_{label}.npz'
        if self.frames:
            np.savez_compressed(path,qpos=np.array(self.frames),time=self.times,phase=self.tags,brick_id=self.ids)
            self.frames.clear();self.times.clear();self.tags.clear();self.ids.clear()
        self.contacts_file.flush()
        if self.vac.samples:
            np.savez_compressed(path.with_suffix('.force'),samples=self.vac.samples)
            self.vac.samples.clear()
        self.pose_log.clear()
    def cycle(self):
        start=self.d.time;clock=time.monotonic();self.clock.clear()
        r,g,f,a=select(self);self.i=int(r['brick_id'][6:]);self.record('select')
        self.event('select',graph=g,poses=observe(self),target=r['brick_id'],strip=a,car=self.base_pose())
        if not self.accepted:
            live=self.d;q0=self.copy_station(self.base_pose()[0][:2]+[.04,-.055],1.34);self.d=q0
            try:r2,*_=select(self);assert r2['brick_id']==r['brick_id'];self.event('base_invariance',target=r2['brick_id'],alternative_pose=self.base_pose())
            finally:self.d=live
        self.snapshot('selection')
        h=self.grasp_feedback(r,f);self.segment('held');self.snapshot('held')
        self.transfer(h);self.segment('receiver');self.snapshot('receiver')
        self.place()
        self.segment('placed');self.snapshot('placed')
        self.drive(np.array([-.18,.32]),np.pi/2,'return_pile')
        self.event('cycle',slot=len(self.accepted)-1,wall_s=time.monotonic()-clock,sim_s=self.d.time-start,stages=dict(self.clock))
        self.segment('return');self.snapshot('returned')
    def main(self,limit):
        self.start()
        while len(self.accepted)<limit:self.cycle()
        self.event('complete',counts=[len(self.picked),self.transfers,len(self.accepted)])
        self.segment('end');self.snapshot('complete')
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('out',type=Path);ap.add_argument('--limit',type=int,default=30);args=ap.parse_args()
    s=Integrated(args.out)
    try:s.main(args.limit)
    except Exception as e:
        s.event('blocked',error=str(e),phase_failed=s.phase,tool=[s.site(a) for a in [0,1]],brick=s.brick(),contacts=s.hits(),traceback=traceback.format_exc());s.segment('blocked');s.snapshot('blocked');raise
