"""Render the as-built wall from its exact checkpoint and draw the distinct nominal floor plans."""
import argparse,json
from pathlib import Path
import egl
from rig import *
from PIL import Image,ImageDraw,ImageFont
ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);args=ap.parse_args()
run=args.run;output=run/'media';report=json.loads((run/'audit_live/verification.json').read_text());rows=report['rows'];n=len(rows)
ck=sorted((run/'snapshots').glob('*_complete.npz'))[-1]
m=mj.MjModel.from_xml_path(str(run/'scene.xml'));d=mj.MjData(m);z=np.load(ck);mj.mj_setState(m,d,z['integration'],mj.mjtState.mjSTATE_INTEGRATION);m.eq_data[:]=z['eq_data'];mj.mj_forward(m,d)
font=lambda sz:ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',sz)
bold=lambda sz:ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',sz)
bg='#13212c';fg='#f2f5f7';muted='#aec1cc';along='#196389';across='#663d82'
w,h=800,510;m.vis.global_.offwidth=w;m.vis.global_.offheight=h
tex=m.texture('tile').id;adr=m.tex_adr[tex];tile=m.tex_rgb[adr:adr+m.tex_width[tex]*m.tex_height[tex]*3].reshape(-1,3);dark=tile[:,0]<128;tile[dark]=[156,166,172];tile[~dark]=[174,183,187]
r=mj.Renderer(m,h,w)
def capture(az,el,dist):
 cam=mj.MjvCamera();mj.mjv_defaultCamera(cam);cam.lookat[:]=[.293,1.36,.058];cam.distance=dist;cam.azimuth=az;cam.elevation=el
 r.update_scene(d,camera=cam);im=Image.fromarray(r.render());eye=r.scene.camera[0];f=np.array(eye.forward);up=np.array(eye.up);right=np.cross(f,up);center=(eye.pos+r.scene.camera[1].pos)/2
 def project(p):
  v=np.asarray(p)-center;s=h/(2*np.tan(np.deg2rad(m.vis.global_.fovy)/2));return np.array([w/2+s*(v@right)/(v@f),h/2-s*(v@up)/(v@f)])
 return im,project
def axis(draw,a,b,color):
 v=b-a;v/=np.linalg.norm(v);side=np.array([-v[1],v[0]])
 draw.line([tuple(a),tuple(b)],fill='white',width=8);draw.line([tuple(a),tuple(b)],fill=color,width=5)
 draw.polygon([tuple(b+2*v),tuple(b-10*v+5*side),tuple(b-10*v-5*side)],fill=color)
top,proj=capture(7,-83,.78);slant,_=capture(18,-27,.82)
# Mark long axes on actual brick faces from the measured post-construction poses.
def mark(row,span,color):
 b=d.body(row['brick']);R=b.xmat.reshape(3,3);u=R[:,0].copy();normal=R[:,2]*np.sign(R[2,2])
 center=b.xpos+normal*.014;a=proj(center-span*u);b=proj(center+span*u)
 axis(ImageDraw.Draw(top),a,b,color)
mark(rows[0],.039,along);mark(rows[-3],.044,across)
out=Image.new('RGB',(1600,724),bg);dr=ImageDraw.Draw(out)
dr.text((32,18),f'As-built wall  |  {n} bricks',font=bold(31),fill=fg);dr.text((32,60),'Both views from the saved simulation state',font=font(19),fill=muted)
out.paste(top,(0,100));out.paste(slant,(800,100))
dr.text((30,622),'Stretcher long axis',font=bold(21),fill='#88ceed');dr.text((285,622),'Header long axis',font=bold(21),fill='#e1b8f1')
dr.text((820,622),'Actual bricks per course, bottom to top',font=font(20),fill=muted);dr.text((820,661),'  /  '.join(map(str,report['actual_course_counts'])),font=bold(26),fill=fg)
dr.text((30,664),f'Three stretcher courses, one header course   |   t = {float(z["time"]):.2f} s',font=font(20),fill=fg)
out.save(output/'wall_as_built.jpg',quality=94);r.close()
plan=json.loads((ROOT/'targets_world.json').read_text());im=Image.new('RGB',(1180,690),bg);dr=ImageDraw.Draw(im)
dr.text((30,20),'Planned wall targets',font=bold(31),fill=fg);dr.text((30,68),'World Y runs along the wall. Each outline is one target footprint.',font=font(19),fill=muted)
for course in range(1,5):
 sel=[q for q in plan if q['course']==course]
 left=30+((course-1)%2)*579;top=145+((course-1)//2)*255
 dr.text((left,top),f"Course {course}   |   {len(sel)} "+("headers" if course==4 else "stretchers"),font=bold(23),fill=fg)
 for t in sel:
  x,y,_=t['p'];B=np.array(t['rotation']);half=abs(B[:2,:2])@np.array([L,W])/2
  def pt(x,y):return (left+25+(y-1.03)*780,top+53+(x-.21)*780)
  dr.rectangle([pt(x-half[0],y-half[1]),pt(x+half[0],y+half[1])],fill=('#795087' if course==4 else '#397a94'),outline='#e2eef4',width=2)
dr.text((30,638),'All four drawings use the same metric scale.',font=font(18),fill=muted)
im.save(output/'wall_plan.png')
