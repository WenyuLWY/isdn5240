import time
import trimesh
import viser
from curobo.content import get_assets_path
mesh_path = (
    get_assets_path()
    / "robot/franka_description/meshes/visual/hand.dae"
)
hand = trimesh.load_mesh(mesh_path, force="mesh")
server = viser.ViserServer()
server.scene.add_mesh_trimesh("/panda_hand", hand)
while True:
    time.sleep(1)
