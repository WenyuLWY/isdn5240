import time
import numpy as np
import viser
from viser.extras import ViserUrdf
from curobo.content import get_assets_path
server = viser.ViserServer()
urdf_path = get_assets_path() / "robot/franka_description/franka_panda.urdf"
robot = ViserUrdf(server, urdf_or_path=urdf_path, load_meshes=True)
robot.update_cfg(np.array([0.0, -1.3, 0.0, -2.5, 0.0, 1.5, 0.8, 0.04, 0.04]))
while True:
    time.sleep(1.0)
