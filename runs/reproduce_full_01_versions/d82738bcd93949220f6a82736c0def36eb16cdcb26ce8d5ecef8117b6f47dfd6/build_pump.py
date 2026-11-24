"""Split the upstream STL along axial planes without scaling geometry.
Use convex hulls per slab instead of bridging the housing to the narrow lip.
"""
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
import xml.etree.ElementTree as ET
import json

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'upstream/cobot_pump.stl'
TIP=.10901301
OUTER=.01396
LIP=.00030
RADIUS=OUTER-LIP

def clipped(tri, lo, hi):
    points=[tri[(tri[:,:,2]>=lo)&(tri[:,:,2]<=hi)]]
    for height in (lo,hi):
        for i in range(3):
            a,b=tri[:,i],tri[:,(i+1)%3]
            cuts=(a[:,2]-height)*(b[:,2]-height)<0
            a,b=a[cuts],b[cuts]
            t=(height-a[:,2])/(b[:,2]-a[:,2])
            points.append(a+t[:,None]*(b-a))
    return np.unique(np.round(np.concatenate(points),8),axis=0)

def install(tree,b):
    raw=np.fromfile(SOURCE,dtype=[('n','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')],offset=84)['v'].astype(float)
    asset=tree.find('asset')
    ET.SubElement(asset,'mesh',name='schmalz_visual',file=str(SOURCE))
    upstream=ET.parse(ROOT/'upstream/fr3v2_cobot_pump.xml').find(".//body[@name='cobot_pump']/inertial")
    b.append(upstream)
    ET.SubElement(b,'geom',name='rear_arm__pump_visual',type='mesh',mesh='schmalz_visual',group='2',contype='0',conaffinity='0',rgba='.17 .25 .32 1')
    zs=[-.031,-.023,-.009,-.006,0,.010,.027,.058,.069,.080,.084,.102,.1068,TIP-.0006]
    for i,(lo,hi) in enumerate(zip(zs,zs[1:])):
        v=clipped(raw,lo,hi)
        v=v[ConvexHull(v).vertices]
        name=f'pump_section_{i:02d}'
        ET.SubElement(asset,'mesh',name=name,vertex=' '.join('%.8g'%x for x in v.ravel()))
        ET.SubElement(b,'geom',name='rear_arm__'+name,type='mesh',mesh=name,group='3',rgba='.2 .32 .4 0',mass='0')
    for i in range(16):
        a=np.array([i,i+1])*np.pi/8
        v=np.column_stack((RADIUS*np.cos(a),RADIUS*np.sin(a),np.full(2,TIP-LIP)))
        ET.SubElement(b,'geom',name=f'rear_arm__lip_{i:02d}',type='capsule',fromto=' '.join(map(str,v.ravel())),size=str(LIP),mass='0',rgba='.04 .08 .09 0',group='3',solref='.010 1',friction='.65 .003 .0001')
    ET.SubElement(b,'site',name='rear_arm__pinch_site',pos=f'0 0 {TIP}',size='.001',rgba='0 0 0 0')
