"""Adapted from v23_topology_display / render_topology.py. Same 3D artist and fixed viewing angle."""
import numpy as np,io,collections
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch
from PIL import Image,ImageDraw,ImageFont
BG='#101e2d';FG='#eaf3fa';BLUE='#63d8ff';ORANGE='#ffcd63';RED='#ff7a79';GREY='#738696'
font=lambda n:ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',n)
class Arrow3D(FancyArrowPatch):
 def __init__(self,a,b,**kw):super().__init__((0,0),(0,0),**kw);self.a=a;self.b=b
 def do_3d_projection(self,renderer=None):
  x,y,z=proj3d.proj_transform(*np.stack([self.a,self.b]).T,self.axes.get_proj());self.set_positions((x[0],y[0]),(x[1],y[1]));return np.min(z)
def panel(v):
 g=v['graph'];p=Image.new('RGB',(720,1080),BG);dr=ImageDraw.Draw(p)
 dr.text((28,22),'SPATIAL NODE-LINK GRAPH',font=font(28),fill=FG)
 dr.text((28,62),'Support graph from simulator poses',font=font(18),fill='#a8c3d8')
 e=v['event'];run=e['run'].replace('contact_loop_v23_','');phase=e['phase']
 dr.text((28,100),run+'  |  '+phase,font=font(18),fill=ORANGE)
 if g is None:
  dr.text((75,360),'Waiting for the first fitted scene',font=font(25),fill=FG)
  dr.text((75,403),'No graph shown before perception finishes.',font=font(19),fill='#a8c3d8')
  return p
 nodes=[n for n in g['nodes'] if 'position_m' in n];loc={n['id']:np.array(n['position_m']) for n in nodes}
 adj=[x for x in g['edges'] if x['kind']=='geometric_adjacency'];sup=[x for x in g['edges'] if x['kind']=='possible_support_priority'];blk=[x for x in g['edges'] if x['kind']=='action_blocking']
 pairs=collections.Counter((x['source'],x['target']) for x in blk)
 fig=plt.figure(figsize=(7.2,5.85),dpi=100,facecolor=BG);ax=fig.add_axes([0,0.03,1,.95],projection='3d');ax.set_facecolor(BG)
 for edge in adj:
  a,b=loc[edge['source']],loc[edge['target']];ax.plot(*np.stack([a,b]).T,c=GREY,ls='--',lw=1.5,alpha=.58)
 for edge in sup:
  a,b=loc[edge['source']],loc[edge['target']];ax.add_artist(Arrow3D(a,b,arrowstyle='-|>',mutation_scale=20,lw=2.4,color=ORANGE,connectionstyle='arc3,rad=.1'))
 for (u,w),n in pairs.items():
  a,b=loc[u],loc[w];a=a+(b-a)*.075;b=b-(b-a)*.075
  ax.add_artist(Arrow3D(a,b,arrowstyle='-|>',mutation_scale=21,lw=2.2,color=RED,connectionstyle='arc3,rad=.13'))
 for n in nodes:
  xyz=loc[n['id']];color=ORANGE if n['id']==v['target'] else BLUE if n.get('active') else GREY
  ax.scatter(*xyz,s=125 if n['id']==v['target'] else 70,c=color,edgecolors=FG,linewidths=1.2,depthshade=False,zorder=8)
 ax.set(xlim=(.18,.72),ylim=(-.57,-.06),zlim=(.18,.44));ax.set_box_aspect((.54,.51,.26));ax.view_init(elev=34,azim=-65)
 for axis in (ax.xaxis,ax.yaxis,ax.zaxis):axis.set_pane_color((.063,.118,.176,1));axis.label.set_color('#a8c3d8');axis._axinfo['grid']['color']=(.3,.4,.5,.18)
 ax.tick_params(colors='#8ba8bf',labelsize=8);ax.set_xlabel('X / m',fontsize=9);ax.set_ylabel('Y / m',fontsize=9);ax.set_zlabel('Z / m',fontsize=9)
 offsets={'brick_000':(-10,-18),'brick_001':(-28,13),'brick_002':(-12,-18),'brick_003':(8,8),'brick_004':(10,12),'brick_005':(9,-19),'brick_006':(-15,13),'brick_007':(-16,18),'brick_008':(10,12)}
 for n in nodes:
  x,y,z=proj3d.proj_transform(*loc[n['id']],ax.get_proj());ax.annotate(n['id'].replace('brick_','B'),xy=(x,y),xycoords='data',xytext=offsets.get(n['id'],(7,7)),textcoords='offset points',color=FG,fontsize=11,fontweight='bold',bbox=dict(boxstyle='round,pad=.16',fc=BG,ec='none',alpha=.85),zorder=20)
 buf=io.BytesIO();fig.savefig(buf,format='png',facecolor=BG);plt.close(fig);buf.seek(0);p.paste(Image.open(buf).convert('RGB'),(0,136));dr=ImageDraw.Draw(p)
 dr.line((26,726,694,726),fill='#38516a',width=1)
 dr.text((28,749),f"{len(nodes)} pile bricks  |  {len(sup)} support edges",font=font(23),fill=FG)
 dr.text((28,793),"Arrows: supporting brick to supported brick",font=font(21),fill=ORANGE)
 return p.crop((0,0,720,845))
