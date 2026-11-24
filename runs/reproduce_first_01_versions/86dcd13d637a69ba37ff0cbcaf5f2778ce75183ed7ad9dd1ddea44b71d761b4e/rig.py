from pathlib import Path
import copy, itertools, json, time, xml.etree.ElementTree as ET
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.interpolate import PchipInterpolator

ROOT = Path(__file__).resolve().parent
PREF = ("", "rear_arm__")
REF = json.loads((ROOT / "a_reference.json").read_text())
A = np.array(REF["site_rotation_in_brick"])
L, W, T = REF["size"]
PAD = 0.011
OFFSET = L / 2 - PAD - 0.010
UP = np.array([0.0, 0.0, 1.0])


def rz(deg):
    return Rot.from_euler("z", deg, degrees=True).as_matrix()


def quat(R):
    return np.roll(Rot.from_matrix(R).as_quat(), 1)


def rel(a, b):
    return a[1].T @ (b[0] - a[0]), a[1].T @ b[1]


def compose(a, b):
    return a[0] + a[1] @ b[0], a[1] @ b[1]


def angle(R):
    return np.rad2deg(Rot.from_matrix(R).magnitude())


def clean(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, np.generic):
        return x.item()
    raise TypeError(type(x).__name__)


def model_xml(out, n):
    from scene import build
    assert n==30
    build(out)


class IKError(Exception):
    pass


class ContactError(Exception):
    pass


class Rig:
    def __init__(self, out, n):
        self.out = out
        if not getattr(self,"reopening",False):
            out.mkdir(parents=True, exist_ok=False)
            model_xml(out / "scene.xml", n)
        self.m = m = mj.MjModel.from_xml_path(str(out / "scene.xml"))
        self.d = mj.MjData(m)
        self.ik = mj.MjData(m)
        self.names = [m.geom(i).name for i in range(m.ngeom)]
        self.groups = [self.group(k) for k in self.names]
        joints = np.array(
            [[m.joint(p + "joint_" + str(i)).id for i in range(1, 8)] for p in PREF]
        )
        self.q = m.jnt_qposadr[joints]
        self.v = m.jnt_dofadr[joints]
        self.act = np.array(
            [[m.actuator(p + "joint_" + str(i)).id for i in range(1, 8)] for p in PREF]
        )
        self.sites = [m.site(p + "pinch_site").id for p in PREF]
        self.fingers = [m.actuator("fingers_actuator").id]
        self.rng = np.random.default_rng(87)
        self.frames = []
        self.tags = []
        self.times = []
        self.ids = []
        self.events = []
        self.i = 0
        self.goals = np.zeros((2, 7))
        self.base = m.actuator("fl_native_steer").id
        self.plan_s = 0.0
        self.phys_s = 0.0
        self.accepted = []
        self.transfers = 0
        self.tick = 0
        self.fwd()

    @staticmethod
    def group(n):
        if n.startswith("brick_"):
            return "brick"
        if (
            n.startswith(("fl_", "fr_", "rl_", "rr_", "ranger_"))
            or "installation" in n
            or "mount_plate" in n
        ):
            return "base"
        if n == "ground":
            return "ground"
        return "rear" if n.startswith(PREF[1]) else "front"

    def fwd(self):
        mj.mj_forward(self.m, self.d)

    def site(self, a, d=None):
        d = self.d if d is None else d
        s = d.site(self.sites[a])
        return s.xpos.copy(), s.xmat.reshape(3, 3).copy()

    def brick(self, i=None):
        b = self.d.body(f"brick_{self.i if i is None else i:03d}")
        return b.xpos.copy(), b.xmat.reshape(3, 3).copy()

    def hits(self, i=None):
        b = f"brick_{self.i if i is None else i:03d}_geom"
        hits = set()
        for c in self.d.contact:
            x, y = self.names[c.geom1], self.names[c.geom2]
            if x == b:
                hits.add(y)
            if y == b:
                hits.add(x)
        return sorted(hits)

    def bilateral(self, a):
        return all(
            any(n.startswith(PREF[a] + side + "_pad") for n in self.hits())
            for side in ("left", "right")
        )

    def bad(self, d, arm=None):
        for c in d.contact:
            a, b = self.groups[c.geom1], self.groups[c.geom2]
            x, y = self.names[c.geom1], self.names[c.geom2]
            if arm is not None and ("front", "rear")[arm] not in (a, b):
                continue
            if {a, b} == {"rear", "front"} or (
                {a, b} & {"ground", "base"}
                and {a, b} & {"front", "rear"}
                and not any(
                    k in x + y
                    for k in (
                        "gen3_base_link",
                        "shoulder_link",
                        "mount_plate",
                        "installation",
                    )
                )
            ):
                return x, y, float(c.dist)
            if (
                a == b
                and a in ("front", "rear")
                and c.dist < -0.001
                and not any(
                    k in x + y
                    for k in ("coupler", "driver", "follower", "spring_link", "pad")
                )
            ):
                return x, y, float(c.dist)
        return None

    def event(self, k, **fields):
        row = dict(phase=k,brick_id=self.i,time=self.d.time,host_utc_unix=time.time(),
                   process_cpu_s=time.process_time(),ik_cumulative_s=self.plan_s,
                   actuator_move_cumulative_s=self.phys_s,**fields)
        self.events.append(row)
        with (self.out / "events.jsonl").open("a") as f:
            f.write(json.dumps(row, default=clean) + "\n")

    def iksolve(self, a, pose, seed=None, tries=8, collision=True):
        st = time.perf_counter()
        m = self.m
        d = self.ik
        d.qpos[:] = self.d.qpos
        seed = self.goals[a].copy() if seed is None else seed.copy()
        pe = 999
        why = None
        try:
            for k in range(tries):
                j = (
                    seed.copy()
                    if k == 0
                    else (
                        seed + self.rng.normal(0, 0.65, 7)
                        if k <= 2
                        else self.rng.uniform(-2.8, 2.8, 7)
                    )
                )
                for _ in range(95):
                    d.qpos[self.q[a]] = j
                    mj.mj_kinematics(m, d)
                    mj.mj_comPos(m, d)
                    p, R = self.site(a, d)
                    dp = pose[0] - p
                    dr = Rot.from_matrix(pose[1] @ R.T).as_rotvec()
                    pe = min(pe, np.linalg.norm(dp))
                    if np.linalg.norm(dp) < 0.00022 and np.linalg.norm(dr) < 0.0018:
                        if collision:
                            mj.mj_forward(m, d)
                            why = self.bad(d, a)
                            if why:
                                break
                        j[::2] = (
                            seed[::2]
                            + (j[::2] - seed[::2] + np.pi) % (2 * np.pi)
                            - np.pi
                        )
                        return j
                    jp = np.zeros((3, m.nv))
                    jr = jp.copy()
                    mj.mj_jacSite(m, d, jp, jr, self.sites[a])
                    J = np.vstack([jp[:, self.v[a]], 0.45 * jr[:, self.v[a]]])
                    j += np.clip(
                        J.T
                        @ np.linalg.solve(
                            J @ J.T + 0.00015 * np.eye(6), np.r_[dp, 0.45 * dr]
                        ),
                        -0.22,
                        0.22,
                    )
                    j[[1, 3, 5]] = np.clip(
                        j[[1, 3, 5]], [-2.235, -2.565, -2.085], [2.235, 2.565, 2.085]
                    )
            raise IKError(
                f"arm={a} target={pose[0].round(4)} min_position={pe:.5f} collision={why}"
            )
        finally:
            self.plan_s += time.perf_counter() - st

    def record(self, phase):
        self.frames.append(self.d.qpos.copy())
        self.tags.append(phase)
        self.times.append(self.d.time)
        self.ids.append(self.i)

    def step(self, phase, solved=False):
        m = self.m
        d = self.d
        d.ctrl[self.act] = (
            self.goals + d.qfrc_bias[self.v] / m.actuator_gainprm[self.act, 0]
        )
        if solved:
            mj.mj_implicit(m,d)
        else:
            mj.mj_step(m,d)
        self.tick += 1
        if self.tick % 10 == 0:
            err = self.bad(d)
            if err:
                raise ContactError(f"{phase} collision {err}")
        if self.tick % 40 == 0:
            self.record(phase)

    def move(self, phase, duration, plans=None, grip=None, base=None):
        st = time.perf_counter()
        plans = plans or {}
        starts = self.goals.copy()
        base0 = self.d.ctrl[self.base]
        g0 = self.d.ctrl[self.fingers].copy()
        n = max(1, round(duration / self.m.opt.timestep))
        paths = {
            a: PchipInterpolator(
                np.linspace(0, 1, len(js) + 1), np.vstack([starts[a], js]), axis=0
            )
            for a, js in plans.items()
        }
        for i in range(n):
            u = (i + 1) / n
            u = u * u * (3 - 2 * u)
            for a, path in paths.items():
                self.goals[a] = path(u)
            if base is not None:
                self.d.ctrl[self.base] = base0 + u * (base - base0)
            for a, val in (grip or {}).items():
                self.d.ctrl[self.fingers[a]] = g0[a] + u * (val - g0[a])
            self.step(phase)
        self.phys_s += time.perf_counter() - st
        self.fwd()

    def cart_path(self, phase, a, target, tries=8):
        p, R = self.site(a)
        to = Rot.from_matrix(target[1] @ R.T).as_rotvec()
        count = max(
            1,
            int(
                np.ceil(
                    max(
                        np.linalg.norm(target[0] - p) / 0.055, np.linalg.norm(to) / 0.26
                    )
                )
            ),
        )
        path = []
        seed = self.goals[a]
        for t in np.linspace(1 / count, 1, count):
            seed = self.iksolve(
                a,
                (p + t * (target[0] - p), Rot.from_rotvec(t * to).as_matrix() @ R),
                seed,
                tries,
            )
            if phase=="present" and np.max(abs(seed-(path[-1] if path else self.goals[a])))>1.0:
                raise IKError("discontinuous donor turnover IK")
            path.append(seed)
        if phase in ("carry","present","clear","rear_only"):
            curve = PchipInterpolator(
                np.linspace(0, 1, len(path) + 1), np.vstack([self.goals[a], path])
            )
            for t in np.linspace(0, 1, 121):
                self.ik.qpos[:] = self.d.qpos
                self.ik.qpos[self.q[a]] = curve(t)
                mj.mj_forward(self.m, self.ik)
                bad = self.bad(self.ik, a)
                if bad:
                    raise IKError(f"loaded path sweep {bad}")
        return path

    def cart(self,phase,a,target,duration,tries=8):
        self.move(phase,duration,{a:self.cart_path(phase,a,target,tries)})

    def save(self):
        d = self.d
        m = self.m
        spec = mj.mjtState.mjSTATE_INTEGRATION
        state = np.zeros(mj.mj_stateSize(m, spec))
        mj.mj_getState(m, d, state, spec)
        np.savez_compressed(
            self.out / "next_state.npz",
            integration=state,
            spec=int(spec),
            goals=self.goals,
            accepted=self.accepted,
            brick_id=self.i,
            time=d.time,
        )
        (self.out / "next_state.npz").replace(self.out / "state.npz")
        np.savez_compressed(
            self.out / "next_trajectory.npz",
            qpos=self.frames,
            phase=self.tags,
            time=self.times,
            brick_id=self.ids,
        )
        (self.out / "next_trajectory.npz").replace(self.out / "trajectory.npz")
        (self.out / "summary.json").write_text(
            json.dumps(
                dict(
                    transfers=self.transfers,
                    placements=len(self.accepted),
                    sim_s=d.time,
                    planning_s=self.plan_s,
                    physics_loop_s=self.phys_s,
                    events=self.events[-1:],
                ),
                default=clean,
                indent=2,
            )
        )
