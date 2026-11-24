from baseline_input import Demo as Donor, HeldBrick, wall
from vacuum import *
from scipy.spatial import ConvexHull
import argparse, traceback
from joint_path import route


def polygon_distance(point, poly):
    edge = np.roll(poly, -1, axis=0) - poly
    v = point - poly
    if np.all(edge[:, 0] * v[:, 1] - edge[:, 1] * v[:, 0] >= 0):
        return 0.0
    t = np.clip(np.sum(v * edge, axis=1) / np.sum(edge * edge, axis=1), 0, 1)
    return np.min(np.linalg.norm(v - t[:, None] * edge, axis=1))


class Demo(Donor):
    def __init__(self, out, n):
        super().__init__(out, n)
        self.hints = json.loads((ROOT / "ik_seeds.json").read_text())
        self.vac = Vacuum(self)
        self.pose_log = []
        self.place_contact = None
        self.bodyids = [self.m.body(f"brick_{i:03}").id for i in range(n)]
        self.contacts_file = (out / "contact_changes.jsonl").open("a" if getattr(self,"reopening",False) else "w")

    def step(self, phase):
        if self.vac.on:
            self.vac.check(phase,split=True)
        super().step(phase,solved=self.vac.on)
        if phase in ("lower", "seat", "release", "withdraw"):
            d = self.d
            self.pose_log.append(
                np.r_[
                    d.time,
                    self.i,
                    ("lower", "seat", "release", "withdraw").index(phase),
                    d.qpos[self.brickq].ravel(),
                ]
            )
            pairs = []
            for c in d.contact:
                a, b = self.names[c.geom1], self.names[c.geom2]
                if "brick_" in a + b and any(
                    x.startswith("rear_arm__") or x == f"brick_{self.i:03}_geom"
                    for x in (a, b)
                ):
                    pairs.append((a, b))
            key = (self.i, phase, tuple(sorted(set(pairs))))
            if key != self.place_contact:
                self.contacts_file.write(json.dumps([d.time, *key]) + "\n")
                self.place_contact = key

    def save(self):
        super().save()
        self.contacts_file.flush()
        np.savez_compressed(
            self.out / "force.npz", samples=np.asarray(self.vac.samples)
        )
        np.savez_compressed(
            self.out / "placement_trace.npz",
            samples=np.asarray(self.pose_log, dtype="float32"),
        )

    def patches(self, h, face):
        b, B = h.world_pose
        B=B@face
        vol = {}
        obstacles = {}
        for gid, n in enumerate(self.names):
            if not self.m.geom_contype[gid] or not (
                n == "base_1" or n.startswith(("left_", "right_"))
            ):
                continue
            m = self.m
            g = self.d.geom(gid)
            typ = m.geom_type[gid]
            if typ == mj.mjtGeom.mjGEOM_MESH:
                idx = m.geom_dataid[gid]
                j = m.mesh_vertadr[idx]
                v = m.mesh_vert[j : j + m.mesh_vertnum[idx]]
            else:
                v = (
                    np.array(list(itertools.product([-1, 1], repeat=3)))
                    * m.geom_size[gid]
                )
            v = (v @ g.xmat.reshape(3, 3).T + g.xpos - b) @ B
            xy = v[:, :2]
            hull = xy[ConvexHull(xy).vertices]
            lo, hi = v[:, 2].min(), v[:, 2].max()
            vol[n] = dict(height=[lo, hi], polygon=hull)
            if lo < T / 2 + 0.037 and hi > T / 2 - 0.001:
                obstacles[n] = (hull, OUTER if lo < T / 2 + 0.009 else 0.008)
        points = []
        for x in np.linspace(-0.0453, 0.0453, 21):
            for y in [0, -0.006, 0.006, -0.012, 0.012]:
                p = np.array([x, y])
                free = min(
                    [L / 2 - abs(x) - OUTER, W / 2 - abs(y) - OUTER]
                    + [
                        polygon_distance(p, g) - radius
                        for g, radius in obstacles.values()
                    ]
                )
                if free > 0.0006:
                    points.append((np.linalg.norm(p), -free, x, y))
        points.sort()
        off = rel(h.world_pose, self.site(0))[0][0]
        self.event(
            "top_surface",
            donor_x=off, face_sign=face[2,2],
            projections=vol,
            blocked_by=list(obstacles),
            available=[[x, y, -f] for _, f, x, y in points],
        )
        return points, off

    def hint(self, phase, pose):
        candidates = [e for e in self.hints if e["phase"] == phase]
        delta = pose[0] - self.d.body("ranger_base_link").xpos
        best = min(
            candidates,
            key=lambda e: (
                np.linalg.norm(delta - e["xyz"])
                + 0.12 * Rot.from_matrix(pose[1] @ np.array(e["R"]).T).magnitude()
            ),
        )
        return np.asarray(best["q"])

    def plan(self, h):
        assert (
            h.brick_id == self.i
            and np.allclose(h.dimensions, [L, W, T])
            and self.bilateral(0)
        )
        t, rot = h.brick_in_tool
        in_don = (-rot.T @ t, rot.T)
        reject = []
        self.rng = np.random.default_rng(87)
        # Shortlist distinct seal centres. Further candidates separate wrists when the nearest one fails.
        from scan import scan
        symmetries=[np.diag(a) for a in [(1,1,1),(-1,-1,1),(1,-1,-1),(-1,1,-1)]]
        # A direct pile grasp may enter either wide face, and either direction
        # along the long edge. Resolve the box-frame symmetry from live feedback.
        F=rel(h.world_pose,self.site(0))[1]
        initial=min(symmetries,key=lambda S:angle(S.T@F@A.T))
        faces=[]
        for face,turnover in [(initial,False),(initial@np.diag([1.,-1.,-1.]),True)]:
            points,off=self.patches(h,face)
            ranked=scan(self,points,off,face)
            if ranked:
                faces.append((min(np.hypot(u[2],u[3]) for u in ranked),turnover,face,ranked,off))
        for _,turnover,face,ranked,off in sorted(faces,key=lambda v:(round(v[0],3),v[1])):
            try:
                return self.solve_face(h,in_don,face,ranked,off,turnover)
            except IKError as error:
                reject.append(str(error))
        raise IKError("no joint transfer pose within budget: "+str(reject))

    def solve_face(self,h,in_don,face,ranked,off,turnover):
        reject=[]
        began=time.monotonic()
        in_don=compose((np.zeros(3),face.T),in_don)
        tried=[]
        for _, _, x, y, tool_yaw in ranked:
            if any(np.hypot(x-u,y-v)<.006 and abs((tool_yaw-w+180)%360-180)<40 for u,v,w in tried):
                continue
            tried.append((x,y,tool_yaw))
            if len(tried)>18 or time.monotonic()-began>40:break
            sign = 1 if x - off > 0 else -1
            for yaw in ([-66,-35,78,55,105,-100] if turnover else ([78,55,105] if sign>0 else [-66,-35,-100])):
                for pitch, z in [
                    (0, 0.56),
                    (15, 0.59),
                    (-22, 0.55),
                    (32, 0.52),
                    (0, 0.70),
                ]:
                    key=(int(face[2,2]),round(x,4),round(y,4),tool_yaw,yaw,pitch,z)
                    if key in getattr(self,"present_rejected",set()):continue
                    B = rz(yaw) @ Rot.from_euler("y", pitch, degrees=True).as_matrix()
                    H = (np.array([0.21, 0.235, z]), B)
                    # Centre the donor TCP in its clear turnover workspace.
                    # The held transform determines where the upper face comes to rest.
                    if turnover:
                        donor_center=np.array([.226,.200,z-.036])
                        H=(donor_center-B@in_don[0],B)
                    H=self.on_vehicle(H)
                    f = compose(H, in_don)
                    top = (np.array([x, y, T / 2]), A @ rz(tool_yaw))
                    r = compose(H, top)
                    live = self.d
                    probe = mj.MjData(self.m)
                    probe.qpos[:] = live.qpos
                    self.d = probe
                    self.fwd()
                    try:
                        self.delivery_path=self.cart_path("present",0,f,tries=4)
                        jf=self.delivery_path[-1]
                        probe.qpos[self.q[0]] = jf
                        self.fwd()
                        pre = (r[0] - 0.065 * r[1][:, 2], r[1])
                        jp = self.iksolve(1, pre, self.hint("rear_pre", pre), tries=12)
                        jr = self.iksolve(1, r, jp, tries=4)
                        probe.qpos[self.q[1]] = jr
                        self.fwd()
                        clearance = self.clearance()
                        if clearance["distance_m"] < 0.0025:
                            raise IKError("tool envelope " + str(clearance))
                    except IKError as e:
                        reject.append(str(e))
                        continue
                    finally:
                        self.d = live
                    self.present_key=key
                    self.planned_pre=jp
                    self.event(
                        "joint_solution",
                        donor=f,
                        rear=r,
                        local_seal=top[0],
                        eccentricity=np.hypot(x, y),
                        donor_joints=jf,
                        rear_joints=jr,
                        face_sign=face[2,2], cup_yaw=tool_yaw, yaw=yaw,
                        pitch=pitch,
                        rejected=reject[-5:],
                    )
                    return f, compose((np.zeros(3),face),top)
        self.event("rejects", reasons=reject)
        raise IKError("no accessible seal: " + str((tried, reject[-3:])))

    def clearance(self):
        from clearance_check import Surfaces
        if not hasattr(self,"surface_pairs"):
            self.surface_pairs=Surfaces(self.m)
        gap,entities=self.surface_pairs.bounds(self.d,.003)
        return dict(distance_m=gap,entities=entities)

    def transfer(self, h):
        self.present_rejected=set()
        for attempt in range(4):
            f, top = self.plan(h)
            self.move("present",4.6,{0:self.delivery_path})
            self.move("present",.45)
            r=compose(self.brick(),top)
            pre=(r[0]-.065*r[1][:,2],r[1])
            failure=None
            for seed in [self.planned_pre,self.goals[1],self.hint("rear_pre",pre)]:
                try:
                    pre_j=self.iksolve(1,pre,seed,tries=18)
                    path=route(self,1,pre_j)
                    break
                except IKError as error:failure=error
            else:
                if not self.bilateral(0) or attempt==3:raise failure
                self.present_rejected.add(self.present_key)
                h=HeldBrick(self.i,h.dimensions,self.brick(),self.d.qpos[self.q[0]].copy(),rel(self.site(0),self.brick()),self.hits())
                self.event("present_retry",error=str(failure),brick_in_tool=h.brick_in_tool,attempt=attempt+1)
                continue
            break
        self.move("rear_pre",1.8+0.6*(len(path)-1),{1:path})
        self.move("rear_pre", 0.30)
        r = compose(self.brick(), top)
        self.cart("rear_align", 1, (r[0] - 0.006 * r[1][:, 2], r[1]), 0.9)
        self.move("rear_align", 0.2)
        for bias in [0.00015, 0.0005, 0.001, 0.0015, 0.0022, 0.003]:
            r = compose(self.brick(), top)
            self.cart("rear_align", 1, (r[0] + bias * r[1][:, 2], r[1]), 0.8)
            self.move("rear_align", 0.28)
            clearance = self.clearance()
            if clearance["distance_m"] < 0.0008:
                raise ContactError("tool clearance at seal " + str(clearance))
            if self.vac.engage():
                break
            if not self.bilateral(0):
                raise ContactError("donor lost during reseat")
        else:
            raise ContactError("no seal; donor retains brick " + str(self.vac.seal()))
        self.event("tool_clearance", **self.clearance())
        self.move("dual", 0.22)
        r = self.site(1)
        self.cart("preload", 1, (r[0] + 0.0018 * UP, r[1]), 0.25)
        self.move("preload", 0.18)
        force, torque = self.vac.check("preload")
        self.event("preload", force=force, moment=torque)
        if force[2] < 0.4:
            raise ContactError("no tensile load before donor release")
        prior = rel(self.site(1), self.brick())
        self.event("donor_release_command",command=self.release_command,staged=self.staged_release)
        self.move("unload", .70, grip={0:self.release_command})
        if any(self.group(n)=="front" for n in self.hits()):
            raise ContactError("donor still contacts brick after unloading")
        self.segment("released");self.snapshot("released")
        self.after_donor_release(prior)

    def after_donor_release(self,prior):
        if not getattr(self,"staged_release",False):return self.separate_hands(prior)
        p,R=self.site(0);away=p-self.site(1)[0];away/=max(np.linalg.norm(away),1e-9)
        candidates=[-.09*R[:,2],-.065*R[:,2],-.040*R[:,2],-.045*R[:,2]+.045*away,
                    .075*away,.055*away+.04*UP,.045*away-.035*UP,-.032*R[:,2]+.045*UP]
        for delta in candidates:
            try:
                j=self.iksolve(0,(p+delta,R),self.goals[0],tries=10)
                path=route(self,0,j,max_seconds=.8)
                if max(np.max(np.abs(x-y)) for x,y in zip([self.goals[0]]+path[:-1],path))>1.1:continue
                break
            except IKError:continue
        else:return self.separate_hands(prior)
        for j in path:self.move("clear",max(1.25,2.*np.max(np.abs(j-self.goals[0]))),{0:[j]})
        self.finish_handover(prior)

    def separate_hands(self,prior):
        p2,R2=self.site(1)
        p0,_=self.site(0);away=p2-p0;away/=max(np.linalg.norm(away),1e-9)
        shoulder=self.d.body("rear_arm__shoulder_link").xpos
        inward=shoulder-p2;inward/=max(np.linalg.norm(inward),1e-9)
        for delta in [.085*UP,.055*UP+.04*inward,.04*UP+.045*away,.065*away,.035*UP,.035*inward]:
            try:path=self.held_path("lift_free",1,(p2+delta,R2));break
            except IKError:continue
        else:raise IKError("no collision-free loaded receiver lift")
        self.move("lift_free",1.1,{1:path})
        self.move("lift_free",.15)
        p, R = self.site(0)
        # Retract opposite the donor approach. After the turn the fingers
        # are below the brick, so an upward withdrawal would strike the pump.
        horizontal=p-self.site(1)[0]
        horizontal[2]=0
        horizontal/=max(np.linalg.norm(horizontal),1e-6)
        for displacement in [-.057*R[:,2],-.030*R[:,2]+.035*horizontal,.055*horizontal,.043*horizontal+.045*UP,-.024*R[:,2]]:
            try:
                clear_path=self.cart_path("clear",0,(p+displacement,R),tries=10)
                break
            except IKError:
                continue
        else:
            raise IKError("no collision-free donor retreat")
        self.move("clear",.95,{0:clear_path})
        self.finish_handover(prior)

    def finish_handover(self,prior):
        # A measured handoff can put the live receiver across the nominal home pose.
        # Try clear parking pockets on the donor half of the chassis.
        already_clear=bool(getattr(self,"staged_release",False) and self.clearance()["distance_m"]>=.0025 and not any(self.group(n)=="front" for n in self.hits()))
        home_path=[] if already_clear else None
        for nominal in [None,[.13,-.12,.64],[.24,-.14,.69],[.02,-.10,.63],[.15,-.20,.55],[-.12,-.07,.79]]:
            if already_clear:break
            try:
                j=self.home_joints[0] if nominal is None else self.iksolve(0,self.on_vehicle((np.array(nominal),rz(78)@A)),self.home_joints[0],tries=10)
                home_path=route(self,0,j,interarm_margin=.009,max_seconds=2.)
                break
            except IKError:
                continue
        if home_path is None:
            # The preceding retreat already separated the donor. Hold its joints
            # here while the receiver moves, then use the measured wheel corridor.
            clearance=self.clearance()
            if clearance["distance_m"]<.0025 or any(self.group(n)=="front" for n in self.hits()):raise IKError("no clear donor parking pocket")
            self.event("donor_hold_clear",position=self.site(0),receiver=self.site(1),clearance=clearance)
            home_path=[]
        for waypoint in home_path:
            delta=np.max(np.abs(waypoint-self.goals[0]))
            self.move("clear",max(.70,1.3*delta),{0:[waypoint]})
        p, R = self.site(1)
        start = self.brick()[0]
        shoulder=self.d.body("rear_arm__shoulder_link").xpos
        inward=(shoulder-p)/max(1e-8,np.linalg.norm(shoulder-p))
        feasible=[]
        for delta in [np.array([.012,.028,.035]),.047*inward,-.041*UP,.042*R[:,0],-.042*R[:,0],.042*R[:,1],-.042*R[:,1]]:
            try:
                path=self.held_path("rear_only",1,(p+delta,R))
                step=float(np.max(np.abs(np.diff(np.vstack([self.goals[1],path]),axis=0))))
                if step<.6:
                    feasible.append((step,path,delta))
                    if step<.20:break
            except IKError:continue
        if not feasible:raise IKError("no continuous independent receiver motion")
        step,path,delta=min(feasible,key=lambda e:e[0])
        duration=max(1.35,3.5*step)
        self.event("solo_plan",max_joint_step_rad=step,duration=duration,offset=delta)
        self.move("rear_only",.28)
        self.move("rear_only",duration,{1:path})
        self.move("rear_only", .25)
        actual = rel(self.site(1), self.brick())
        slip = np.linalg.norm(prior[0] - actual[0])
        ang = angle(actual[1] @ prior[1].T)
        travel = np.linalg.norm(self.brick()[0] - start)
        if (
            slip > 0.005
            or ang > 6
            or travel < 0.025
            or not self.vac.on
            or any(self.group(n) != "rear" for n in self.hits())
        ):
            raise ContactError("solo carry validation failed")
        self.transfers += 1
        self.event(
            "handover", solo_m=travel, slip_m=slip, slip_deg=ang, contacts=self.hits()
        )

    def load(self):
        d = self.d
        total = 0.0
        contacts = {}
        for k, c in enumerate(d.contact):
            n = [self.names[c.geom1], self.names[c.geom2]]
            b = f"brick_{self.i:03}_geom"
            if b not in n:
                continue
            other = n[1 - n.index(b)]
            if not (
                other == "ground"
                or (
                    other.startswith("brick_")
                    and self.d.geom(other).xpos[2] < self.brick()[0][2] - 0.01
                )
            ):
                continue
            w = np.zeros(6)
            mj.mj_contactForce(self.m, d, k, w)
            fz = (c.frame.reshape(3, 3).T @ w[:3])[2] * (1 if n[1] == b else -1)
            total += fz
            contacts[other] = contacts.get(other, 0) + fz
        return total, contacts

    def place(self):
        p,R,layer=self.targets[len(self.accepted)]
        self.drive_wall(p)
        self.segment("at_wall");self.snapshot("at_wall")
        self.place_at_wall(p,R,layer)

    def park_donor_at_wall(self):
        p,R=self.site(0);rear,_=self.site(1)
        if np.linalg.norm(p-rear)>.26:return
        direction=p-rear;direction[2]=0;direction/=max(1e-8,np.linalg.norm(direction))
        options=[(p+.145*direction+.065*UP,R),(p+.18*direction+.02*UP,R),
                 (p+.085*direction+.135*UP,R),self.home(0)]
        for target in options:
            try:
                j=self.iksolve(0,target,self.goals[0],tries=15)
                path=route(self,0,j,interarm_margin=.006,max_seconds=2.)
                break
            except IKError:continue
        else:raise IKError("no safe donor parking before loaded descent")
        for waypoint in path:
            self.move('wall_clear',max(1.0,2.2*np.max(np.abs(self.goals[0]-waypoint))),{0:[waypoint]})
        self.event('wall_clear',front=self.site(0),rear=self.site(1),tool_separation_m=np.linalg.norm(self.site(0)[0]-self.site(1)[0]))

    def place_at_wall(self,p,R,layer):
        self.park_donor_at_wall()
        hand=rel(self.brick(),self.site(1))
        if self.vac.face_sign<0:R=R@np.diag([1.,-1.,-1.])
        choices=sorted([R,R@rz(180)],key=lambda r:angle(r@hand[1]@self.site(1)[1].T))
        candidates=[];errors=[]
        for orient in choices:
            g=compose((p,orient),hand);pre=(g[0]+.092*UP,g[1])
            try:
                self.iksolve(1,g,tries=20)
                path=self.held_path("carry",1,pre)
                jump=np.max(np.abs(np.diff(np.vstack([self.goals[1],path]),axis=0)))
                if jump>.7:raise IKError(f"loaded IK branch change {jump}")
                duration=max(3.0,1.9*jump*len(path))
                candidates.append((duration,path,orient,False));break
            except IKError as e:errors.append(str(e))
        if not candidates:
            for orient in choices:
                g=compose((p,orient),hand);pre=(g[0]+.092*UP,g[1])
                for seed in [self.goals[1],self.home_joints[1]]:
                    try:
                        joint=self.iksolve(1,pre,seed,tries=22)
                        self.iksolve(1,g,joint,tries=10)
                        path=route(self,1,joint,carrying=True,max_seconds=7)
                        duration=sum(max(1.,2.2*np.max(np.abs(y-x))) for x,y in zip([self.goals[1]]+path[:-1],path))
                        candidates.append((duration,path,orient,True));break
                    except IKError as e:errors.append(str(e))
                if candidates:break
        if not candidates:raise IKError("no swept pre-place path "+str(errors[-5:]))
        duration,path,R,jointspace=candidates[0]
        self.event("carry_plan",duration=duration,jointspace=jointspace,waypoints=len(path))
        if jointspace:
            for j in path:self.move("carry",max(1.,2.2*np.max(np.abs(j-self.goals[1]))),{1:[j]})
        else:self.move("carry",duration,{1:path})
        self.move("carry", 0.32)
        for z, dur in [
            (0.025, 0.9),
            (0.004, 0.6),
            (0.0004, 0.45),
            (-0.00025, 0.3),
            (-0.0006, 0.3),
            (-0.0012, 0.3),
            (-0.0017, 0.3),
        ]:
            g = compose((p, R), rel(self.brick(), self.site(1)))
            self.cart("lower", 1, (g[0] + z * UP, g[1]), dur)
            self.move("lower", 0.22)
            F, sup = self.load()
            if F >= 0.8 * self.m.body_mass[self.bodyids[self.i]] * 9.81:
                break
        else:
            raise ContactError(f"no support {F} {self.brick()[0]}")
        self.event("supported", force_N=F, objects=sup, brick=self.brick())
        self.vac.vent()
        self.event("vent", active=bool(self.d.eq_active[self.vac.eq]))
        self.move("release", 0.36)
        g = self.site(1)
        self.cart("withdraw", 1, (g[0] + UP * 0.095, g[1]), 0.95)
        self.move("settle", 0.45)
        pp, RR = self.brick()
        err = np.linalg.norm(p - pp)
        ang = angle(R @ RR.T)
        if (
            err > 0.008
            or ang > 5
            or self.load()[0] < 1
            or any(self.group(n) == "rear" for n in self.hits())
        ):
            raise ContactError(f"placement failed {err} {ang}")
        self.event(
            "placement_audit",
            position_error=err,
            angle_error=ang,
            actual=self.brick(),
            support=self.hits(),
        )
        self.accepted.append(self.i)
        self.event("placed", count=len(self.accepted))
        self.cart("return", 1, self.home(1), 2.0, tries=20)
        for arm in [1,0]:
            goal=self.home_joints[arm]
            if np.max(np.abs(goal-self.goals[arm]))<.03:continue
            path=route(self,arm,goal,max_seconds=4)
            for waypoint in path:
                self.move("return",max(.65,1.5*np.max(np.abs(waypoint-self.goals[arm]))),{arm:[waypoint]})
        self.move("return", 0.28)
