from rig import *
import sys
def audit(run,out):
 m=mj.MjModel.from_xml_path(str(run/'scene.xml'));d=mj.MjData(m)
 def mat(frame):
  T=np.eye(4);T[:3,3]=frame.xpos;T[:3,:3]=frame.xmat.reshape(3,3);return T
 rows=[]
 for f in sorted((run/'snapshots').glob('*.npz')):
  with np.load(f) as z:
   if str(z['stage']) not in ('held','receiver','placed'):continue
   m.eq_data[:]=z['eq_data'];mj.mj_setState(m,d,z['integration'],mj.mjtState.mjSTATE_INTEGRATION)
   mj.mj_forward(m,d);i=int(z['active']);base=mat(d.body('ranger_base_link'));front=mat(d.site('pinch_site'));rear=mat(d.site('rear_arm__pinch_site'));brick=mat(d.body(f'brick_{i:03}'))
   rows.append(dict(time=float(z['time']),stage=str(z['stage']),brick=i,T_world_base=base,T_base_front=np.linalg.solve(base,front),T_base_rear=np.linalg.solve(base,rear),T_front_brick=np.linalg.solve(front,brick),T_rear_brick=np.linalg.solve(rear,brick),T_world_brick=brick))
 out.write_text(''.join(json.dumps(r,default=clean)+'\n' for r in rows))
if __name__=='__main__':audit(Path(sys.argv[1]),Path(sys.argv[2]))
