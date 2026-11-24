import egl
from rig import *
from PIL import Image,ImageDraw
p=Path('runs/main01');m=mj.MjModel.from_xml_path(str(p/'scene.xml'));d=mj.MjData(m);z=np.load(next((p/'snapshots').glob('01_*_returned.npz')));mj.mj_setState(m,d,z['integration'],mj.mjtState.mjSTATE_INTEGRATION);mj.mj_forward(m,d)
r=mj.Renderer(m,260,420);tile=Image.new('RGB',(1260,520))
for j,(az,el) in enumerate([(154,-35),(35,-32),(5,-35),(-10,-36),(70,-32),(-35,-34)]):
 c=mj.MjvCamera();mj.mjv_defaultCamera(c);c.lookat[:]=[.16,.52,.28];c.distance=2.82;c.azimuth=az;c.elevation=el;r.update_scene(d,camera=c);im=Image.fromarray(r.render());ImageDraw.Draw(im).text((6,6),str((az,el)),fill='black',stroke_fill='white',stroke_width=2);tile.paste(im,(j%3*420,j//3*260))
tile.save(p/'media/angles.jpg');r.close();print([k for k in dir(mj) if 'onnector' in k])
