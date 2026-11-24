from rig import *

from build_pump import install, RADIUS, OUTER, LIP

# Conservative effective seal radius. Pressure and friction are simulator assumptions.
EFFECTIVE = .0125
DP = 70000.0
FN = DP * np.pi * EFFECTIVE**2
MU = .65
PEEL = .62 * FN * EFFECTIVE
TWIST = .60 * MU * FN * EFFECTIVE


def install_tool(tree, n):
    wrist = tree.find(".//body[@name='rear_arm__bracelet_link']")
    mount = wrist.find("body[@name='rear_arm__base_mount']")
    mount.remove(mount.find("body[@name='rear_arm__base']"))
    wrist.remove(wrist.find("site[@name='rear_arm__pinch_site']"))
    for tag in ["contact", "equality", "tendon", "actuator"]:
        section = tree.find(tag)
        for node in list(section):
            s = ET.tostring(node, encoding="unicode")
            if any(
                x in s
                for x in [
                    "rear_arm__left_",
                    "rear_arm__right_",
                    "rear_arm__split",
                    "rear_arm__fingers_actuator",
                ]
            ):
                section.remove(node)
    # Axial 33 mm adapter plus 7 mm to the pump mesh origin. The recessed
    # stem clears the lateral connector below the commercial flange.
    ET.SubElement(mount,"geom",name="rear_arm__adapter_stem",type="cylinder",pos="0 0 .0135",size=".015 .0135",mass=".064",rgba=".44 .49 .53 1")
    ET.SubElement(mount,"geom",name="rear_arm__adapter_flange",type="cylinder",pos="0 0 .0305",size=".031 .0035",mass=".048",rgba=".44 .49 .53 1")
    b=ET.SubElement(wrist,"body",name="rear_arm__vacuum",pos="0 0 -.09769039",quat="0 1 0 0")
    install(tree,b)
    tree.find("option").set("jacobian", "sparse")
    for i in range(n):
        ET.SubElement(
            tree.find("equality"),
            "weld",
            name=f"seal_{i:03}",
            active="false",
            body1="rear_arm__vacuum",
            body2=f"brick_{i:03}",
            torquescale=".10",
            solref=".006 1",
            solimp=".95 .995 .001",
        )


class Vacuum:
    def __init__(self, rig):
        self.rig = rig
        self.on = False
        self.samples = []
        self.drops = 0

    def seal(self):
        s = self.rig
        b, B = s.brick()
        p, R = s.site(1)
        sign=1 if (p-b)@B[:,2]>=0 else -1
        B=B@np.diag([1,sign,sign])
        pc = B.T @ (p - b)
        rr = B.T @ R
        # Exact support function for the disk footprint and plane gap, with lip radius.
        extent = OUTER * np.linalg.norm(rr[:, :2], axis=1)
        margin = np.array([L / 2, W / 2]) - abs(pc[:2]) - extent[:2]
        dz = pc[2] - T / 2
        gaps = [dz - extent[2], dz + extent[2]]
        align = np.degrees(np.arccos(np.clip(-rr[2, 2], -1, 1)))
        contacts = [x for x in s.hits() if x.startswith("rear_arm__lip_")]
        valid = bool(
            min(margin) > 0.0003
            and align < 3
            and gaps[0] >= -0.0017
            and gaps[1] <= 0.0008
            and len(contacts) >= 3
        )
        return dict(
            valid=valid, face_sign=sign,
            margin_xy=margin,
            gap_range=gaps,
            tilt_deg=align,
            lip_contacts=contacts,
            center=pc,
        )

    def engage(self):
        s = self.rig
        receipt = self.seal()
        s.event("seal_test", **receipt)
        if not receipt["valid"]:
            return False
        self.face_sign=receipt["face_sign"]
        m, d = s.m, s.d
        self.eq = m.equality(f"seal_{s.i:03}").id
        b, B = s.brick()
        p, _ = s.site(1)
        tool = d.body("rear_arm__vacuum")
        R = tool.xmat.reshape(3, 3)
        self.local_anchor = B.T @ (p - b)
        self.initial_axes = B.T @ s.site(1)[1]
        m.eq_data[self.eq, :3] = self.local_anchor
        m.eq_data[self.eq, 3:6] = R.T @ (p - tool.xpos)
        m.eq_data[self.eq, 6:10] = quat(R.T @ B)
        self.v = m.jnt_dofadr[m.body_jntadr[m.body(f"brick_{s.i:03}").id]]
        d.eq_active[self.eq] = 1
        self.on = True
        s.fwd()
        return True

    def check(self, phase, split=False):
        s = self.rig
        m, d = s.m, s.d
        # Refresh q-dependent gravity compensation, then evaluate the same controls
        # that the physical integrator will use. Trip before integrating an overload.
        if split:
            mj.mj_step1(m,d)
            d.ctrl[s.act]=s.goals+d.qfrc_bias[s.v]/m.actuator_gainprm[s.act,0]
            mj.mj_fwdActuation(m,d);mj.mj_fwdAcceleration(m,d);mj.mj_fwdConstraint(m,d);mj.mj_sensorAcc(m,d)
        else:
            mj.mj_fwdPosition(m,d);mj.mj_fwdVelocity(m,d)
            d.ctrl[s.act]=s.goals+d.qfrc_bias[s.v]/m.actuator_gainprm[s.act,0]
            mj.mj_forward(m,d)
        rows = (d.efc_type == int(mj.mjtConstraint.mjCNSTR_EQUALITY)) & (
            d.efc_id == self.eq
        )
        constraint_forces=np.zeros(d.nefc)
        constraint_forces[rows]=d.efc_force[rows]
        generalized=np.zeros(m.nv)
        mj.mj_mulJacTVec(m,d,generalized,constraint_forces)
        wrench=generalized[self.v:self.v+6]
        B = d.body(f"brick_{s.i:03}").xmat.reshape(3, 3)
        f = B.T @ wrench[:3]
        moment = wrench[3:] - np.cross(self.local_anchor, f)
        f=f*np.array([1,self.face_sign,self.face_sign])
        moment=moment*np.array([1,self.face_sign,self.face_sign])
        levels = np.array(
            [
                max(0.0, f[2]) / FN,
                np.linalg.norm(f[:2]) / (MU * FN),
                np.linalg.norm(moment[:2]) / PEEL,
                abs(moment[2]) / TWIST,
            ]
        )
        if s.tick % 10 == 0:
            self.samples.append([d.time, s.i, *f, *moment, *levels])
        cup, R = s.site(1)
        b, B = s.brick()
        error = B.T @ (cup - b) - self.local_anchor
        tilt_change = angle((B.T @ R) @ self.initial_axes.T)
        geometry_lost = (
            np.linalg.norm(error[:2]) > 0.002
            or abs(error[2]) > 0.002
            or tilt_change > 5
        )
        if max(levels) > 1 or geometry_lost:
            d.eq_active[self.eq] = 0
            self.on = False
            self.drops += 1
            s.fwd()
            s.event(
                "overload_trip",
                stage=phase,
                force=f,
                moment=moment,
                ratios=levels,
                anchor_error=error,
                angle_change=tilt_change,
            )
            raise ContactError("vacuum capacity exceeded: " + str(levels))
        return f, moment

    def vent(self):
        s = self.rig
        s.d.eq_active[self.eq] = 0
        self.on = False
        s.fwd()
