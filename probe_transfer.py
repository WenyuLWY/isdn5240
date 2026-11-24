from integrated import *
from dataclasses import asdict
import sys
s=object.__new__(Integrated);s.reopening=True
s.out=Path(sys.argv[1]);from types import MethodType
src=s.out;dest=Path(sys.argv[3]);dest.mkdir(exist_ok=False,parents=True)
os.symlink((src/'scene.xml').resolve(),dest/'scene.xml')
Integrated.__init__(s,dest);z=np.load(sys.argv[2]);mj.mj_setState(s.m,s.d,z['integration'],mj.mjtState.mjSTATE_INTEGRATION);s.m.eq_data[:]=z['eq_data'];s.goals=z['goals'];s.home_joints=z['home'];s.i=int(z['active']);s.fwd()
h=HeldBrick(s.i,2*s.m.geom(f'brick_{s.i:03}_geom').size.copy(),s.brick(),s.d.qpos[s.q[0]].copy(),rel(s.site(0),s.brick()),s.hits());t=time.monotonic();s.transfer(h);print('test handoff',s.i,time.monotonic()-t)
