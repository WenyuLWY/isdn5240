import egl
from rig import *
from axes import SupportGraph,observe
from graph_view import panel
from PIL import Image,ImageDraw,ImageFont
import argparse,subprocess
from OpenGL.GL import GL_RENDERER,glGetString
from types import SimpleNamespace
a=argparse.ArgumentParser();a.add_argument('run',type=Path);a.add_argument('name');a.add_argument('--first',action='store_true');a.add_argument('--speed',type=int,default=16);a.add_argument('--until',type=int);a.add_argument('--focus-slot',type=int);a.add_argument('--hd',action='store_true');a.add_argument('--wide-only',action='store_true');a.add_argument('--feature-slot',type=int,default=1);args=a.parse_args()
p=args.run;out=p/'media';out.mkdir(exist_ok=True)
ev=list(map(json.loads,(p/'events.jsonl').read_text().splitlines()));placed=[e for e in ev if e['phase']=='placed']
until=placed[(args.until or (1 if args.first else len(placed)))-1]['time'] if placed else ev[-1]['time']
cycles=[e for e in ev if e['phase']=='cycle']
count=(args.until or (1 if args.first else len(placed)))
if count and len(cycles)>=count:until=cycles[count-1]['time']
segments=[f for f in sorted((p/'segments').glob('*.npz')) if not f.name.endswith('.force.npz')]
tracks=[]
for f in segments:
 with np.load(f) as t:tracks += [(float(st),int(i),str(ph),q.copy()) for st,i,ph,q in zip(t['time'],t['brick_id'],t['phase'],t['qpos']) if st<=until+.001]
assert tracks and all(b[0]>=a[0]-1e-8 for a,b in zip(tracks,tracks[1:])), 'discontinuous history'
m=mj.MjModel.from_xml_path(str(p/'scene.xml'));d=mj.MjData(m);scale=1.5 if args.hd else 1.;px=lambda v:int(round(v*scale));w,h=px(960),px(600);m.vis.global_.offwidth=w;m.vis.global_.offheight=h
tid=m.texture('tile').id;adr=m.tex_adr[tid];pixels=m.tex_rgb[adr:adr+m.tex_width[tid]*m.tex_height[tid]*3].reshape(-1,3)
dark=pixels[:,0]<128;pixels[dark]=[118,130,137];pixels[~dark]=[163,173,178]
r=mj.Renderer(m,h,w);cam=mj.MjvCamera();mj.mjv_defaultCamera(cam)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',px(22))
selects={e['brick_id']:e for e in ev if e['phase']=='select'};endtimes=[e['time'] for e in placed]
metadata=dict(gpu=egl.gpu,frame_size=[w,h],renderer=glGetString(GL_RENDERER).decode(),frames=0,source_segments=[x.name for x in segments],last_source_time=until,graph_frames=[],video_markers={})
proc=subprocess.Popen(['ffmpeg','-nostdin','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}','-r','25','-i','-','-c:v','libx264','-threads','2','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(out/(args.name+'.mp4'))],stdin=subprocess.PIPE)
encode_feeding=0.
def emit(im,n=1):
 global encode_feeding
 t_io=time.perf_counter()
 raw=im.tobytes()
 for _ in range(n):proc.stdin.write(raw)
 metadata['frames']+=n;encode_feeding+=time.perf_counter()-t_io
def view(tt,i,ph,graph=None):
 bb=d.body(f'brick_{i:03}');wide=(ph in ('ready','overview','reposition','drive_wall','return_pile','return','growth','park') or ph.startswith(('drive','reposition','return_pile')))
 wide=wide or args.wide_only
 if wide:
  cam.lookat[:]=[.18,.47,.29];cam.distance=2.93;cam.azimuth=154;cam.elevation=-35
 elif ph in ('wall_result','wall_progress'):
  cam.lookat[:]=[.288,1.37,.055];cam.distance=1.10;cam.azimuth=14;cam.elevation=-63
 elif ph in ('lower','withdraw','settle','release','carry'):
  cam.lookat[:]=bb.xpos+[0,0,.065];cam.distance=.92;cam.azimuth=145;cam.elevation=-28
 elif ph in ('present','rear_pre','rear_align','dual','unload','preload','clear','lift_free','rear_only'):
  cam.lookat[:]=bb.xpos+[0,0,.035];cam.distance=.74;cam.azimuth=-138;cam.elevation=-9
 else:
  cam.lookat[:]=[.44,-.30,.32];cam.distance=1.24;cam.azimuth=83;cam.elevation=-34
 g=m.geom(f'brick_{i:03}_geom').id;rgba=m.geom_rgba[g].copy()
 if ph in ('select','approach','align','insert','close','test_lift','extract','push_clearance'):m.geom_rgba[g]=[1,.65,.055,1]
 r.update_scene(d,camera=cam)
 if wide:
  # Floor footprint is a visualization of fixed world targets, with no physical geom.
  corners=np.array([[.208,1.018,.002],[.375,1.018,.002],[.375,1.698,.002],[.208,1.698,.002]])
  for start,end in zip(corners,np.roll(corners,-1,axis=0)):
   geom=r.scene.geoms[r.scene.ngeom];r.scene.ngeom+=1
   mj.mjv_initGeom(geom,mj.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([.13,.48,.56,.85],np.float32))
   mj.mjv_connector(geom,mj.mjtGeom.mjGEOM_CAPSULE,.0035,start,end)
 im=Image.fromarray(r.render());m.geom_rgba[g]=rgba
 if ph in ('select','approach','align','insert','close','test_lift'):
  eye=r.scene.camera[0];fwd=np.array(eye.forward);up=np.array(eye.up);right=np.cross(fwd,up)
  v=bb.xpos-(eye.pos+r.scene.camera[1].pos)/2;scale=h/(2*np.tan(np.deg2rad(m.vis.global_.fovy)/2))
  x=w/2+scale*(v@right)/(v@fwd);y=h/2-scale*(v@up)/(v@fwd)
  mark=ImageDraw.Draw(im);mark.line([(x+px(14),y-px(14)),(x+px(32),y-px(34)),(x+px(66),y-px(34))],fill='#ffcd63',width=px(3))
  mark.text((x+px(34),y-px(60)),f'B{i:03}',font=font,fill='#ffcd63',stroke_width=px(2),stroke_fill='#101e2d')
 if graph is not None:
  chart=panel(graph).resize((px(374),px(440)),Image.Resampling.LANCZOS)
  scene=im.resize((px(688),px(430)),Image.Resampling.LANCZOS);im=Image.new('RGB',(w,h),'#101e2d');im.paste(scene.crop((px(64),0,px(650),px(430))),(0,px(60)));im.paste(chart,(px(586),px(50)))
 labels={'select':'Select an exposed brick','reposition':'Reposition to reach','approach':'Approach','align':'Align','insert':'Grasp','close':'Grasp','test_lift':'Test lift','extract':'Lift clear','present_ready':'Carry to handover','present':'Present the exposed face','rear_pre':'Dual-arm handover','rear_align':'Seal on the upper face','dual':'Dual-arm handover','preload':'Verify suction load','unload':'Release the front gripper','lift_free':'Lift clear','clear':'Retract the front arm','rear_only':'Verify rear-only hold','drive_wall':'Drive to the wall','carry':'Approach wall slot','lower':'Place and release','release':'Vent suction','settle':'Verify support','withdraw':'Retract vertically','return':'Retract','return_pile':'Return to the pile','overview':'Pile and construction area','ready':'Pile and construction area','growth':f'Continuous construction  {args.speed}x','park':'Verified placements','wall_result':('Three stretcher courses and one header course' if sum(t<=tt+.00001 for t in endtimes)==30 else 'Ground wall'),'wall_progress':f'Wall construction  {args.speed}x','reposition_waypoint':'Reposition to reach','return_pile_waypoint':'Return to the pile','push_clearance':'Open a grasp corridor','prepare_clearance':'Approach the blocked pair','approach_clearance':'Approach the short end','withdraw_clearance':'Retract after clearing','settle_clearance':'Reobserve the pile','wall_clear':'Clear the placement path','calibrated_open':'Set grasp aperture','drive_wall_waypoint':'Drive to the wall','seat':'Verify support'}
 count=sum(t<=tt+.00001 for t in endtimes);dr=ImageDraw.Draw(im);dr.rectangle((0,px(546),w,h),fill='#101e2d');dr.text((px(17),px(558)),f'{labels.get(ph,ph)}   |   B{i:03}   |   {count}/30',font=font,fill='white');dr.text((px(777),px(558)),f'{tt:.2f} s',font=font,fill='#c5dbe4')
 return im
def graph_at(i,tag,tt):
 snapshot=next((x for x in sorted((p/'snapshots').glob(f'*_{i:03}_{tag}.npz'))),None)
 if not snapshot:return None
 with np.load(snapshot) as t:
  if abs(float(t['time'])-tt)>.045:return None
  mj.mj_setState(m,d,t['integration'],mj.mjtState.mjSTATE_INTEGRATION);m.eq_data[:]=t['eq_data'];ids=list(t['accepted'].astype(int));tt=float(t['time'])
 mj.mj_forward(m,d)
 if tag=='held':ids.append(i)
 rows=observe(SimpleNamespace(m=m,d=d));g=selects[i]['graph'] if tag=='selection' else SupportGraph(m).update(m,d,rows,[f'brick_{j:03}' for j in ids])
 obj=dict(graph=dict(nodes=[dict(id=x['brick_id'],position_m=x['position_m'],active=True) for x in rows if int(x['brick_id'][6:]) not in ids],edges=[dict(source=x['lower'],target=x['upper'],kind='possible_support_priority') for x in g['edges']]),event=dict(run='A',phase='Target locked' if tag=='selection' else 'After extraction'),target=f'brick_{i:03}')
 im=view(tt,i,'select' if tag=='selection' else 'extract',obj)
 metadata['graph_frames'].append(dict(t=tt,i=i,stage=tag,edges=[(x['lower'],x['upper']) for x in g['edges']]))
 return im
start=time.monotonic();seen=set();first_id=placed[(args.focus_slot or args.feature_slot)-1]['brick_id'] if placed else tracks[0][1];keyframes=set()
for k,(tt,i,ph,q) in enumerate(tracks):
 if args.focus_slot and i!=first_id:continue
 early=(i==first_id) or (not args.focus_slot and ph in ('approach_clearance','push_clearance','withdraw_clearance','settle_clearance')) or (tt<=selects[placed[0]['brick_id']]['time'] if placed and not args.focus_slot else False)
 if not early and k%args.speed:continue
 d.qpos[:]=q;mj.mj_forward(m,d)
 if early and i in selects and not args.wide_only:
  for tag,t0 in [('selection',selects[i]['time']),('held',next((e['time'] for e in ev if e['phase']=='A_input' and e['brick_id']==i),-999))]:
   if (i,tag) not in seen and abs(tt-t0)<.041:
    graph=graph_at(i,tag,t0)
    if graph is not None:
     metadata['video_markers'][tag]=dict(output_s=metadata['frames']/25,sim_s=t0,brick=i);graph.save(out/(args.name+'_'+tag+'.jpg'));emit(graph,35);seen.add((i,tag));d.qpos[:]=q;mj.mj_forward(m,d)
 if not early and ph in ('lower','release','withdraw','settle'):
  label='wall_progress'
 else:
  label=ph if early else 'growth'
 im=view(tt,i,label)
 # Pause only on an actual sample following the certified completion of a course.
 done=sum(t<=tt+.00001 for t in endtimes)
 if not args.focus_slot and not args.first and done in (10,18,24) and ('course',done) not in seen:
  wall_image=view(tt,i,'wall_result');emit(wall_image,30)
  metadata.setdefault('course_freezes',[]).append(dict(count=done,time=tt,frames=30))
  wall_image.save(out/(args.name+f'_course_{done:02}.jpg'));seen.add(('course',done))
 if early and ph in ('test_lift','dual','drive_wall','release','push_clearance') and ph not in keyframes:
  metadata['video_markers'][ph]=dict(output_s=metadata['frames']/25,sim_s=tt,brick=i);im.save(out/(args.name+'_'+ph+'.jpg'));keyframes.add(ph)
 if early and ph.startswith('drive_wall') and 'drive_wall' in metadata['video_markers'] and 'moving' not in metadata['video_markers'] and tt-metadata['video_markers']['drive_wall']['sim_s']>1.0:
  metadata['video_markers']['moving']=dict(output_s=metadata['frames']/25,sim_s=tt,brick=i)
 emit(im)
# Snapshot immediately after accepted placement includes the release/withdraw verification.
last=placed[(args.focus_slot-1) if args.focus_slot else (-1 if not args.first else 0)] if placed else None
if args.until:last=placed[args.until-1]
if last and not args.wide_only:
 path=sorted((p/'snapshots').glob(f"*_{last['brick_id']:03}_returned.npz")) or sorted((p/'snapshots').glob(f"*_{last['brick_id']:03}_placed.npz"))
 if path:
  z=np.load(path[0]);mj.mj_setState(m,d,z['integration'],mj.mjtState.mjSTATE_INTEGRATION);mj.mj_forward(m,d);im=view(float(z['time']),last['brick_id'],'park');emit(im,25);im.save(out/(args.name+'_end.jpg'))
  wall_im=view(float(z['time']),last['brick_id'],'wall_result')
  metadata['video_markers']['wall']=dict(output_s=metadata['frames']/25,sim_s=float(z['time']),brick=last['brick_id']);wall_im.save(out/(args.name+'_wall.jpg'));emit(wall_im,65)
t_io=time.perf_counter();proc.stdin.close();assert proc.wait()==0;encode_feeding+=time.perf_counter()-t_io;r.close();metadata.update(encoder_feed_and_flush_wall_s=encode_feeding,render_and_encode_s=time.monotonic()-start,feature_brick=first_id,other_speed=args.speed,sim_sample_interval_s=.04,fps=25);(out/(args.name+'.json')).write_text(json.dumps(metadata,indent=1))
