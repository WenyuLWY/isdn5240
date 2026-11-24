from inspect_hold import *
from exact_strips import NativeStrips
from collections import Counter
rows=observe(s);s.strips.boxes=[k for k in s.strips.boxes if s.names[k] in ('work_table','brick_010_geom','brick_011_geom')];n=NativeStrips(s)
for i in [10,11]:
 row=rows[i];found=False
 for axis in [2,0,1]:
  for sg in ([1] if axis==2 else [-1,1]):
   for roll in [False,True]:
    for cmd in [85,80,75,150,155,165,0]:
     jaw=next(x for x in CAL['samples'] if x['command']==cmd)
     for lean in [0,.4,-.4,.8,-.8,1.10,-1.10,1.35,-1.35]:
      for audit,f in n.candidates(s,row,axis,lean,jaw,sg,abs(lean)>.25,tight=True,roll=roll):
       print(i,axis,sg,roll,cmd,lean,audit['common_m'],flush=True);found=True;break
      if found:break
     if found:break
    if found:break
   if found:break
  if found:break
 if not found:print(i,'no tight grip',flush=True)
