"""Add the integrated demo link to a clean existing branch landing page."""
from pathlib import Path
import subprocess
repo=Path('/mnt/data/cjx_data/brick-disassembly/code_v52');f=repo/'README.md'
status=subprocess.check_output(['git','-C',str(repo),'status','--porcelain','--',f.name],text=True)
if status:raise SystemExit('README has pre-existing work; leave it intact')
s=f.read_text();anchor='## A+B+C 从原始砖堆到地面墙'
if anchor not in s:
 j=s.find('\n## ');j=j if j>=0 else len(s)
 s=s[:j]+'\n'+anchor+'\n\n[集成主视频](course_demo/abc_integrated/media/main.mp4) · [逐砖结果与复现](course_demo/abc_integrated/) · [实抓至落墙的连续片段](course_demo/abc_integrated/media/mobile_cycle.mp4)\n'+s[j:]
 f.write_text(s)
