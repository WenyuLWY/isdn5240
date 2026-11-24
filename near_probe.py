from inspect_hold import *
p=[]
for cidx,c in enumerate(s.d.contact):
 a,b=[s.names[g] for g in [c.geom1,c.geom2]];ga,gb=map(s.group,[a,b])
 if {ga,gb}=={'front','rear'} or 'brick' in [ga,gb] and (a==f'brick_{s.i:03}_geom' or b==f'brick_{s.i:03}_geom') or ga==gb and ga in ('front','rear'):
  f=np.zeros(6);mj.mj_contactForce(s.m,s.d,cidx,f);p.append((round(f[0],2),a,b,round(c.dist*1000,3)))
print(sorted(p,reverse=True)[:26]);print('wrist v',s.d.qvel[s.v[1]],'difference',s.d.qpos[s.q[1]]-s.goals[1])
