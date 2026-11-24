import egl
from rig import *
from PIL import Image
from OpenGL.GL import GL_RENDERER,glGetString
def image(run):
 m=mj.MjModel.from_xml_path(str(run/'scene.xml'));d=mj.MjData(m);mj.mj_forward(m,d);m.vis.global_.offwidth=960;m.vis.global_.offheight=600
 cam=mj.MjvCamera();mj.mjv_defaultCamera(cam);cam.lookat[:]=[.15,.44,.24];cam.distance=2.95;cam.azimuth=160;cam.elevation=-40
 with mj.Renderer(m,600,960) as r:
  r.update_scene(d,camera=cam);Image.fromarray(r.render()).save(run/'scene.jpg');print(egl.gpu,glGetString(GL_RENDERER))
if __name__=='__main__':
 import sys
 image(Path(sys.argv[1]))
