"""Cross-check tool/brick transforms, body identity and continuous sample times."""
from rig import *
import sys
def audit(run,out):
 m=mj.MjModel.from_xml_path(str(run/'scene.xml'));d=mj.MjData(m)
 zfiles=sorted((run/'snapshots').glob('*_placed.npz'));candidate=zfiles[-1];complete=sorted((run/'snapshots').glob('*_complete.npz'));z=np.load(complete[-1] if complete else candidate);ids=z['accepted'].astype(int);stop=float(z['time'])
 paths=sorted(f for f in (run/'segments').glob('*.npz') if not f.name.endswith('.force.npz'))
 history=[];labels=[];times=[];ident=[]
 for f in paths:
  with np.load(f) as a:
   k=a['time']<=stop+1e-8
   history.extend(a['qpos'][k]);times.extend(a['time'][k]);ident.extend(a['brick_id'][k]);labels.extend(a['phase'][k])
 history=np.asarray(history);ts=np.array(times);dt=np.diff(ts)
 assert len(ts) and dt.min()>=-1e-8 and dt.max()<=.040001, (dt.min(),dt.max(),ts[dt.argmax()])
 events=list(map(json.loads,(run/'events.jsonl').read_text().splitlines()));seals=[m.equality('seal_%03d'%i).id for i in range(30)]
 seed=json.loads((run/'source.json').read_text())['bricks']
 initial=mj.MjData(m);mj.mj_forward(m,initial)
 pose_error=max(np.linalg.norm(initial.body(row['id']).xpos-row['xyz']) for row in seed)
 q_error=max(1-abs(initial.body(row['id']).xquat@np.array(row['quat'])) for row in seed)
 assert pose_error<1e-9 and q_error<1e-9
 root=m.jnt_qposadr[m.joint('ranger_native_freejoint').id]
 rows=[]
 for i in ids:
  name=f'brick_{i:03d}';body=m.body(name);bidx=m.jnt_qposadr[m.body_jntadr[body.id]]
  def event(kind):return next(e for e in events if e['phase']==kind and e['brick_id']==i)
  def state(tag):
   snap=next((run/'snapshots').glob(f'*_{i:03d}_{tag}.npz'));x=np.load(snap)
   m.eq_data[:]=x['eq_data'];mj.mj_setState(m,d,x['integration'],mj.mjtState.mjSTATE_INTEGRATION);mj.mj_forward(m,d)
   return d
  data=state('held');rec=event('A_input')['payload'];arm=data.site('pinch_site');b=data.body(name)
  p,R=rel((arm.xpos,arm.xmat.reshape(3,3)),(b.xpos,b.xmat.reshape(3,3)))
  assert rec['brick_id']==i and len(rec['donor_joints'])==7
  assert all(any(n.startswith(k+'_pad') for n in rec['feedback']) for k in ('left','right'))
  assert not any(n.startswith('brick_') or n in ('ground','work_table') for n in rec['feedback'])
  delta=np.linalg.norm(p-rec['brick_in_tool'][0]);angle_err=angle(R@np.array(rec['brick_in_tool'][1]).T)
  assert delta<2e-5 and angle_err<.02 and not data.eq_active[seals].any(),(i,delta,angle_err)
  data=state('receiver');assert list(np.where(data.eq_active[seals])[0])==[i] and m.eq_obj2id[seals[i]]==body.id
  data=state('placed');assert not data.eq_active[seals].any()
  pick=event('select');included=[n for n in pick['poses'] if n['brick_id']==name];assert len(included)==1 and name in pick['graph']['allowed']
  tripmask=(np.array(ident)==i)&np.char.startswith(np.array(labels),'drive_wall')
  track=history[tripmask,root:root+2];base_dist=np.linalg.norm(np.diff(track,axis=0),axis=1).sum()
  seal=next(e for e in events if e['phase']=='seal_test' and e['brick_id']==i and e['valid'])
  assert len(seal['lip_contacts'])>=3 and min(seal['margin_xy'])>.0003
  assert base_dist>.1 and all(n.startswith('rear_arm__') for n in event('handover')['contacts'])
  disp=np.linalg.norm(np.diff(history[:,bidx:bidx+3],axis=0),axis=1)
  assert float(disp.max())<.11,(name,'large sample discontinuity',float(disp.max()))
  rows.append(dict(brick=name,body_id=body.id,free_joint=int(m.body_jntadr[body.id]),seal_id=seals[i],initial_pose=included[0],held_translation_error_m=delta,held_rotation_error_deg=angle_err,actual_drive_trace_m=base_dist,max_step_40ms_m=disp.max()))
 result=dict(accepted=len(ids),unique_native_bricks=30,initial_position_error_m=pose_error,initial_quat_inner_error=q_error,frames=len(ts),max_sample_gap_s=dt.max(),continuous=True,rows=rows)
 out.write_text(json.dumps(result,default=clean,indent=1));print('chain verified',len(ids),'frames',len(ts),'max gap',dt.max())
if __name__=='__main__':audit(Path(sys.argv[1]),Path(sys.argv[2]))
