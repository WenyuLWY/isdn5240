from integrated import *
import sys
src=Path(sys.argv[1]);s=object.__new__(Integrated);s.reopening=True
import tempfile
tmp=tempfile.TemporaryDirectory();dst=Path(tmp.name);os.symlink((src/'scene.xml').resolve(),dst/'scene.xml');Integrated.__init__(s,dst)
z=np.load(sys.argv[2]);mj.mj_setState(s.m,s.d,z['integration'],mj.mjtState.mjSTATE_INTEGRATION);s.m.eq_data[:]=z['eq_data'];s.goals=z['goals'];s.home_joints=z['home'];s.i=int(z['active']);s.fwd()
if __name__=='__main__':
 print('tools',s.site(0)[0],s.site(1)[0],'clearance',s.clearance(),'bad',s.bad(s.d),'hits',s.hits(),flush=True)
 for a,j in enumerate(s.home_joints):
  if a:continue
  for margin in [.009,.003,0]:
   try:r=route(s,a,j,interarm_margin=margin);print(margin,'plan',len(r),flush=True);break
   except IKError as e:print(margin,str(e),flush=True)
