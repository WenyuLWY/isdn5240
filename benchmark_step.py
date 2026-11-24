from types import SimpleNamespace
from rig import *
run=Path('runs/main01');m=mj.MjModel.from_xml_path(str(run/'scene.xml'));s=np.load(next((run/'snapshots').glob('*028_receiver.npz')))
act=np.array([[m.actuator(p+'joint_'+str(k)).id for k in range(1,8)] for p in PREF])
v=m.jnt_dofadr[np.array([[m.joint(p+'joint_'+str(k)).id for k in range(1,8)] for p in PREF])]
m.eq_data[:]=s['eq_data'];results=[]
for mode in ['baseline','split']:
 d=mj.MjData(m);mj.mj_setState(m,d,s['integration'],mj.mjtState.mjSTATE_INTEGRATION);mj.mj_forward(m,d);t=time.monotonic()
 for k in range(180):
  if mode=='baseline':
   mj.mj_fwdPosition(m,d);mj.mj_fwdVelocity(m,d);d.ctrl[act]=s['goals']+d.qfrc_bias[v]/m.actuator_gainprm[act,0];mj.mj_forward(m,d);d.ctrl[act]=s['goals']+d.qfrc_bias[v]/m.actuator_gainprm[act,0];mj.mj_step(m,d)
  else:
   mj.mj_step1(m,d);d.ctrl[act]=s['goals']+d.qfrc_bias[v]/m.actuator_gainprm[act,0]
   mj.mj_fwdActuation(m,d);mj.mj_fwdAcceleration(m,d);mj.mj_fwdConstraint(m,d);mj.mj_sensorAcc(m,d);mj.mj_implicit(m,d)
 results.append((mode,time.monotonic()-t,d.qpos.copy(),d.qvel.copy()))
a,b=results;rec=dict(steps=180,original_s=a[1],split_s=b[1],max_qpos_difference=float(max(abs(a[2]-b[2]))),max_qvel_difference=float(max(abs(a[3]-b[3]))));print(rec);(run/'step_benchmark.json').write_text(json.dumps(rec,indent=2))
