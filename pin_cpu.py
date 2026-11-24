"""Keep the serial physics worker on an idle core local to the bulk of its memory."""
import os,time,sys,json,re
from pathlib import Path
pid=int(sys.argv[1]);p=Path('/proc')/str(pid)
allow=os.sched_getaffinity(pid)
def counters():
 return {int(a[0][3:]):(sum(map(int,a[1:9])),int(a[4])+int(a[5])) for a in map(str.split,Path('/proc/stat').read_text().splitlines()) if re.fullmatch('cpu[0-9]+',a[0])}
old=counters();time.sleep(1.5);new=counters()
pages={}
for key,v in re.findall(r'\bN(\d+)=(\d+)',(p/'numa_maps').read_text()):pages[int(key)]=pages.get(int(key),0)+int(v)
node=max(pages,key=pages.get);cpu_root=Path('/sys/devices/system/cpu')
def load(i):
 dt=new[i][0]-old[i][0]
 return 1-(new[i][1]-old[i][1])/max(1,dt)
def siblings(i):return [int(x) for x in (cpu_root/f'cpu{i}/topology/thread_siblings_list').read_text().strip().split(',')]
choices=[i for i in allow if (cpu_root/f'cpu{i}/node{node}').exists()]
cpu=min(choices,key=lambda i:(sum(load(s) for s in siblings(i)),load(i),i))
report=dict(pid=pid,allowed_before=sorted(allow),node=node,pages=pages,cpu=cpu,siblings=siblings(cpu),prior_load=[load(s) for s in siblings(cpu)])
os.sched_setaffinity(pid,{cpu});print(json.dumps(report))
