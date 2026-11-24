"""Conservative separation certificates for the receiver hardware vs donor at replay frames."""

from demo import *
from types import SimpleNamespace
from scipy.optimize import minimize
import sys

LIMIT = 0.003


class Surfaces:
    def __init__(self, m):
        self.m = m
        self.verts = {}
        for g in range(m.ngeom):
            if m.geom_type[g] != mj.mjtGeom.mjGEOM_MESH:
                continue
            mesh = m.geom_dataid[g]
            start = m.mesh_vertadr[mesh]
            v = m.mesh_vert[start : start + m.mesh_vertnum[mesh]]
            self.verts[g] = v[ConvexHull(v).vertices].astype(float)
        ids = [k for k in range(m.ngeom) if m.geom_contype[k]]
        front = [k for k in ids if Rig.group(m.geom(k).name) == "front"]
        rear = [
            k
            for k in ids
            if Rig.group(m.geom(k).name) == "rear"
            and any(
                part in m.geom(k).name
                for part in (
                    "lip_",
                    "adapter",
                    "stem",
                    "nozzle",
                    "pump_section",
                    "wrist",
                    "bracelet",
                    "camera",
                )
            )
        ]
        self.a, self.b = np.array(list(itertools.product(rear, front))).T

    def extent(self, g, u):
        m = self.m
        t = m.geom_type[g]
        s = m.geom_size[g]
        if g in self.verts:
            return np.max(self.verts[g] @ u)
        if t == mj.mjtGeom.mjGEOM_BOX:
            return abs(u) @ s
        if t == mj.mjtGeom.mjGEOM_CAPSULE:
            return s[0] + abs(u[2]) * s[1]
        if t == mj.mjtGeom.mjGEOM_CYLINDER:
            return np.linalg.norm(u[:2]) * s[0] + abs(u[2]) * s[1]
        if t == mj.mjtGeom.mjGEOM_SPHERE:
            return s[0]
        raise ValueError(m.geom(g).name)

    def pair(self, d, a, b, limit):
        RA = d.geom_xmat[a].reshape(3, 3)
        RB = d.geom_xmat[b].reshape(3, 3)
        delta = d.geom_xpos[b] - d.geom_xpos[a]

        def gap(n):
            n = n / max(1e-14, np.linalg.norm(n))
            return delta @ n - self.extent(a, RA.T @ n) - self.extent(b, -RB.T @ n)

        best = gap(delta)
        direction = delta
        pts = np.zeros(6)
        for tol in [limit, 0.015, 0.04, 0.09]:
            if best >= limit:
                return best
            di = mj.mj_geomDistance(self.m, d, a, b, tol, pts)
            if di >= tol or np.linalg.norm(pts[3:] - pts[:3]) < 1e-10:
                continue
            n = (pts[3:] - pts[:3]) * np.sign(di)
            value = gap(n)
            if value > best:
                best = value
                direction = n
        if best < 1e-6:
            sol = minimize(
                lambda x: -gap(x),
                direction / np.linalg.norm(direction),
                method="Nelder-Mead",
                options={"maxiter": 100, "xatol": 1e-7, "fatol": 1e-8},
            )
            best = max(best, -sol.fun)
        return best

    def bounds(self, d, limit):
        m = self.m
        lower = (
            np.linalg.norm(d.geom_xpos[self.a] - d.geom_xpos[self.b], axis=1)
            - m.geom_rbound[self.a]
            - m.geom_rbound[self.b]
        )
        result = limit
        who = None
        for k in np.where(lower < limit)[0]:
            a, b = self.a[k], self.b[k]
            gap = self.pair(d, a, b, result)
            if gap < result:
                result = gap
                who = [m.geom(a).name, m.geom(b).name]
        return result, who


def check(run):
    m = mj.MjModel.from_xml_path(str(run / "scene.xml"))
    d = mj.MjData(m)
    s = Surfaces(m)
    with np.load(run / "trajectory.npz") as t:
        q, phase, ids, ts = (t[k] for k in ("qpos", "phase", "brick_id", "time"))
    rows = {}
    for k in np.flatnonzero(
        np.isin(phase, ["rear_pre", "rear_align", "dual", "preload", "unload", "clear"])
    ):
        d.qpos[:] = q[k]
        mj.mj_forward(m, d)
        v, n = s.bounds(d, LIMIT)
        i = int(ids[k])
        if i not in rows or v < rows[i]["separation_lower_bound_m"]:
            rows[i] = dict(
                phase=str(phase[k]), time=ts[k], separation_lower_bound_m=v, entities=n
            )
    (run / "approach_clearance.json").write_text(
        json.dumps(rows, default=clean, indent=2)
    )
    print(min(r["separation_lower_bound_m"] for r in rows.values()))


if __name__ == "__main__":
    check(Path(sys.argv[1]))
