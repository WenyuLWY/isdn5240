from inspect_hold import *
active=f'brick_{s.i:03}_geom'
for c in s.d.contact:
 ns=[s.names[c.geom1],s.names[c.geom2]]
 if active in ns and any(n.startswith('brick_') and n!=active for n in ns):
  other=ns[1-ns.index(active)];print(other,c.dist,c.frame[:3]*(1 if ns[1]==active else -1), 'position',c.pos)
print('hold',s.brick()[0],s.site(0)[0],s.d.qpos[s.q[0]]-s.goals[0])
