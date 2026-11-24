from inspect_hold import *
from exact_strips import search
rows=observe(s);excluded=list(np.load(sys.argv[2])['accepted'])+[int(i) for i in sys.argv[3:]]
g=SupportGraph(s.m).update(s.m,s.d,rows,[f'brick_{i:03}' for i in excluded])
s.strips.boxes=[k for k in s.strips.boxes if s.names[k]=='work_table' or int(s.names[k][6:9]) not in excluded]
for r in rows:
 if r['brick_id'] not in g['allowed']:continue
 t=time.monotonic();v=search(s,r);print(r['brick_id'],round(time.monotonic()-t,2),v[0]['common_m'] if v else None,v[0]['calibrated_open_command'] if v else None,flush=True)
