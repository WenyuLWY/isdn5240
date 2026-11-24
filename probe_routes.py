from inspect_hold import *
r=next(x for x in observe(s) if x['brick_id']==f'brick_{s.i:03d}');print('target',r['position_m'],'base',s.base_pose()[0],'gap',s.margin.gap(s.d))
for xy,yaw in s.stations(r):
 d=s.copy_station(xy,yaw);why=s.drive_clear(xy,yaw)
 if args:=getattr(s,'extra',False):pass
 print('endpoint',xy,round(yaw,2),'gap',round(s.margin.gap(d),3),'static',s.bad(d),'drive',why,flush=True)
d=s.copy_station(s.base_pose()[0][:2],np.pi/2);print('near contacts',[(s.names[c.geom1],s.names[c.geom2]) for c in d.contact if set((s.groups[c.geom1],s.groups[c.geom2]))&{'base','front','rear'} and 'brick' in [s.groups[c.geom1],s.groups[c.geom2]]])
