"""Retain the controller/configuration bytes used to open a server worker."""
from pathlib import Path
import hashlib,json,sys,shutil,time
root=Path(__file__).resolve().parent;run=Path(sys.argv[1]);mode=sys.argv[2]
src=[root/n for n in ('integrated.py','rig.py','demo.py','restore.py','scene.py','baseline_input.py','axes.py','exact_strips.py','declutter_native.py','joint_path.py','vehicle_margin.py','vacuum.py','scan.py','build_pump.py','clearance_check.py','wall_targets.py','prepare_dependency.py','provenance.json','a_reference.json','ik_seeds.json','prototype.xml','env.sh','run.sh')]
sha=lambda b:hashlib.sha256(b).hexdigest()
manifest={f.name:sha(f.read_bytes()) for f in src};key=sha(json.dumps(manifest,sort_keys=True).encode());base=run.parent/(run.name+'_versions')
target=base/key;target.mkdir(exist_ok=True,parents=True)
for f in src:
 if not (target/f.name).exists():shutil.copy2(f,target/f.name)
(target/'sha256.json').write_text(json.dumps(manifest,indent=1))
with (base/'workers.jsonl').open('a') as o:o.write(json.dumps(dict(time_unix=time.time(),mode=mode,source_sha256=key,arguments=sys.argv[3:]))+'\n')
