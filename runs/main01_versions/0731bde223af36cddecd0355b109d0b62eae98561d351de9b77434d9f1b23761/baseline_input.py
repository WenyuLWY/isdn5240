from rig import *

from dataclasses import dataclass, asdict
import argparse, traceback


@dataclass
class HeldBrick:
    brick_id: int
    dimensions: np.ndarray
    world_pose: tuple
    donor_joints: np.ndarray
    brick_in_tool: tuple
    feedback: list


def wall():
    # Three shrinking courses preserve a half-brick stagger and full bearing.
    # Coordinates grow along +Y. Headers bridge both parallel rows.
    items = []
    for layer, count in enumerate([5, 4, 3]):
        for col in range(count):
            for row in [0, 1]:
                items.append(
                    (
                        np.array(
                            [
                                0.253 + row * 0.077,
                                0.45 + (col + layer * 0.5) * 0.128,
                                (layer + 0.5) * T,
                            ]
                        ),
                        rz(90),
                        layer,
                    )
                )
    for shift in np.arange(6) - 2.5:
        items.append((np.array([0.2915, 0.706 + shift * 0.062, 3.5 * T]), rz(0), 3))
    return items


class Demo(Rig):
    def start(self):
        for a in [0, 1]:
            j = self.iksolve(a, self.home(a), tries=40)
            self.goals[a] = j
            self.d.qpos[self.q[a]] = j
            self.fwd()
        self.home_joints = self.goals.copy()
        self.move("ready", 0.4)
