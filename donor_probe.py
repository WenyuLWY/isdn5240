from inspect_hold import *
p,R=s.site(0);r,_=s.site(1);print('start',p,r,'bad',s.bad(s.d),'goal offset',s.goals[0]-s.d.qpos[s.q[0]],'base',s.base_pose()[0]);v=p-r;v/=np.linalg.norm(v)
for delta in [-.06*R[:,2],.065*v,.035*UP,-.035*UP,.040*R[:,1],-.040*R[:,1],-.04*R[:,2]+.04*v]:
 try:
  j=s.iksolve(0,(p+delta,R),s.d.qpos[s.q[0]],tries=15)
  path=route(s,0,j,max_seconds=1);print('ok',delta,len(path))
 except IKError as e:print('fail',delta,str(e))
for j in [s.home_joints[0]]:
 try:print('home',len(route(s,0,j,max_seconds=2)))
 except IKError as e:print('home',str(e))
