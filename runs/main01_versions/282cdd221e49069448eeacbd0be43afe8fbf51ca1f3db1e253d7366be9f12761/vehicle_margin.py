"""A conservative 2-D separating-plane clearance for chassis/tire versus table."""
from rig import *
from scipy.spatial import ConvexHull
class Margin:
 def __init__(self,m):
  self.m=m
  phi=np.arange(64)*2*np.pi/64
  self.axes=np.stack((np.cos(phi),np.sin(phi)),axis=0)
  self.geoms=[]
  for i in range(m.ngeom):
   g=m.geom(i)
   if g.name!='ranger_base_collision' and not g.name.endswith('_wheel_collision'):continue
   if m.geom_type[i]==mj.mjtGeom.mjGEOM_MESH:
    a=m.geom_dataid[i];v=m.mesh_vert[m.mesh_vertadr[a]:m.mesh_vertadr[a]+m.mesh_vertnum[a]];v=v[ConvexHull(v).vertices]
   else:v=np.array(list(itertools.product([-1,1],repeat=3)))*[g.size[0],g.size[0],g.size[1]]
   self.geoms.append((i,v))
  self.table=m.geom('work_table').id
 def gap(self,d):
  t=d.geom(self.table);bounds=t.xpos[:2]@self.axes-np.abs(self.axes).T@self.m.geom_size[self.table,:2]
  minimum=10.
  for i,v in self.geoms:
   pts=v@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i]
   if pts[:,2].min()>.22:continue
   sep=np.max(bounds-np.max(pts[:,:2]@self.axes,axis=0));minimum=min(minimum,sep)
  return minimum
