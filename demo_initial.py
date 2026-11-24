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
            self.vac.check(phase)
        super().step(phase)
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
        for face in (np.eye(3), np.diag([1.,-1.,-1.])):
            points,off=self.patches(h,face)
            ranked=scan(self,points,off,face)
            if ranked:
                break
        in_don=compose((np.zeros(3),face.T),in_don)
        tried=[]
        for _, _, x, y, tool_yaw in ranked:
            if any(np.hypot(x-u,y-v)<.006 and abs((tool_yaw-w+180)%360-180)<40 for u,v,w in tried):
                continue
            tried.append((x,y,tool_yaw))
            sign = 1 if x - off > 0 else -1
            for yaw in ([-66,-35,78,55,105,-100] if face[2,2]<0 else ([78,55,105] if sign>0 else [-66,-35,-100])):
                for pitch, z in [
                    (0, 0.56),
                    (15, 0.59),
                    (-22, 0.55),
                    (32, 0.52),
                    (0, 0.70),
                ]:
                    B = rz(yaw) @ Rot.from_euler("y", pitch, degrees=True).as_matrix()
                    H = (np.array([0.21, 0.235, z]), B)
                    # Centre the donor TCP in its clear turnover workspace.
                    # The held transform determines where the upper face comes to rest.
                    if face[2,2]<0:
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
        f, top = self.plan(h)
        self.move("present",4.6,{0:self.delivery_path})
        self.move("present", 0.45)
        r = compose(self.brick(), top)
        pre = (r[0] - 0.065 * r[1][:, 2], r[1])
        pre_j=self.iksolve(1,pre,self.planned_pre,tries=6)
        path=route(self,1,pre_j)
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
        self.move("unload", .70, grip={0: 0})
        if any(self.group(n)=="front" for n in self.hits()):
            raise ContactError("donor still contacts brick after unloading")
        p2,R2=self.site(1)
        self.cart("lift_free",1,(p2+.085*UP,R2),1.1)
        self.move("lift_free",.15)
        p, R = self.site(0)
        # Retract opposite the donor approach. After the turn the fingers
        # are below the brick, so an upward withdrawal would strike the pump.
        horizontal=p-r[0]
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
        home_path=route(self,0,self.home_joints[0])
        self.move("clear",1.8+.6*(len(home_path)-1),{0:home_path})
        p, R = self.site(1)
        start = self.brick()[0]
        self.cart("rear_only", 1, (p + [0.012, 0.028, 0.035], R), 0.8)
        self.move("rear_only", 0.2)
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
        hand = rel(self.brick(), self.site(1))
        if self.vac.face_sign < 0:
            R = R @ np.diag([1.,-1.,-1.])
        choices = sorted(
            [R, R @ rz(180)], key=lambda r: angle(r @ hand[1] @ self.site(1)[1].T)
        )
        last = None
        for R in choices:
            g = compose((p, R), hand)
            try:
                q = self.iksolve(1, g, tries=20)
                self.cart("carry", 1, (g[0] + 0.092 * UP, g[1]), 2.5, tries=18)
                break
            except IKError as e:
                last = e
        else:
            raise last
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
        self.move("return", 1.35, {a: [self.home_joints[a]] for a in [0, 1]})
        self.move("return", 0.28)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--count", type=int, default=30)
    ap.add_argument("--offset", type=float)
    args = ap.parse_args()
    assert 1 <= args.count <= len(wall())
    s = Demo(args.out, args.count)
    s.brickq = np.array(
        [s.m.jnt_qposadr[s.m.body_jntadr[b]] + np.arange(7) for b in s.bodyids]
    )
    try:
        s.start()
        for i in range(args.count):
            s.i = i
            h = s.input(
                args.offset if args.offset is not None else [-OFFSET, OFFSET][i % 2]
            )
            s.transfer(h)
            s.place()
        s.event("complete", drops=s.vac.drops)
    except Exception as e:
        s.fwd()
        s.event(
            "blocked",
            error=str(e),
            brick=s.brick(),
            tools=[s.site(a) for a in [0, 1]],
            contacts=s.hits(),
        )
        raise
    finally:
        s.save()


if __name__ == "__main__":
    main()
