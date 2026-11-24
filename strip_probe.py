from inspect_hold import *
from axes import CAL
print('samples (command, gap mm)',[(q['command'],round(q['opening_m']*1000,1)) for q in CAL['samples'] if .060<q['opening_m']<.09])
rows=observe(s);z=np.load(sys.argv[2]);graph=s.graph.update(s.m,s.d,rows,[f'brick_{i:03d}' for i in z['accepted']]);print('allowed',graph['allowed'])
for row in rows:
 if row['brick_id'] not in graph['allowed']:continue
 R=s.d.body(row['brick_id']).xmat.reshape(3,3);normal=R[:,2].copy();normal*=np.sign(normal[2])
 reasons={}
 for clipped in [False,True]:
  for ap in sorted([x for x in CAL['samples'] if x['opening_m']>.0605],key=lambda x:-x['opening_m']):
   for lean in [0.,.18,-.18,.32,-.32,.52,-.52,.68,-.68,.85,-.85]:
    for _,(a,f) in s.strips.families(s.d,row,dict(wide_axis=2,normal=normal,entry_lean_rad=lean,insertion_aperture=ap,allow_certified_contact_clipping=clipped)):
     if f:
      print(row['brick_id'],round(normal[2],3),'SUCCESS',clipped,ap['command'],round(ap['opening_m']*1000,1),lean,a['common_m'],a.get('pad_face_fraction'),flush=True);break
     reasons[a.get('reason')]=reasons.get(a.get('reason'),0)+1
    if f:break
   if f:break
  if f:break
 else:print('FAIL',row['brick_id'],round(normal[2],3),reasons)
