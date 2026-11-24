from inspect_hold import *
from axes import CAL
rows=observe(s);excluded=np.load(sys.argv[2])['accepted'];graph=s.graph.update(s.m,s.d,rows,[f'brick_{i:03d}' for i in excluded]); print('allowed',graph['allowed'])
for r in rows:
 if r['brick_id'] not in graph['allowed']: continue
 name=r['brick_id'];R=s.d.body(name).xmat.reshape(3,3);sz=np.array(r['size_m']);sol=None
 for broad in sorted(range(3),key=lambda j:-abs(R[2,j])):
  for sign in [np.sign(R[2,broad]) or 1,-np.sign(R[2,broad]) or -1]:
   norm=R[:,broad]*sign
   if norm[2]<-.15:continue
   for closing in sorted((j for j in range(3) if j!=broad),key=lambda j:-sz[j]):
    aps=sorted((a for a in CAL['samples'] if a['opening_m']>sz[closing]+.004),key=lambda a:a['opening_m'])
    if not aps:continue
    for ap in [s.strips.full,aps[0],aps[len(aps)//2]]:
     for lean in [0.,.18,-.18,.40,-.40,.75,-.75]:
      a,f=s.strips.calculate(s.d,r,dict(wide_axis=broad,normal=norm,entry_lean_rad=lean,insertion_aperture=ap,pre_distance_m=.07),closing)
      if name=='brick_006' and broad==2 and lean==0 and ap['command']==0:print('006 full',[(k['finger'],k['obstacle']) for k in a.get('blockers',[])][:24])
      if f:
       sol=(broad,sign,closing,ap['command'],lean,a['common_m']);break
     if sol:break
    if sol:break
   if sol:break
  if sol:break
 print(name,'normal z',R[2],sol,flush=True)
