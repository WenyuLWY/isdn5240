import hashlib,json,subprocess,sys
from pathlib import Path
import numpy,mujoco,scipy
root=Path(__file__).resolve().parent
repo=Path('/mnt/data/cjx_data/brick-disassembly/code_v52');a='5a27c5132b2d6f6b9801a8f24d145ae7519b7668'
config=Path('/mnt/data/cjx_data/brick-disassembly/runs/ranger_v52/oracle73_config.yaml')
keys=['oracle_support','oracle_strips','oracle_geometry','mobile_ranger','robot_model','robot_planning','core','runtime']
sha=lambda b:hashlib.sha256(b).hexdigest()
manifest=dict(a_commit=a,bc_commit='510e8649fc2d25ff6d6f2fa99753d9514c23d7fd',reference_graph='/home/cjx/disassembly/worktrees/v23-topology-display-publish/results/contact_loop_v23_topology_display_20261005/render_topology.py',config=str(config),config_sha256=sha(config.read_bytes()),A_modules={n:sha(subprocess.check_output(['git','-C',str(repo),'show',f'{a}:disassembly/{n}.py'])) for n in keys},python=sys.version,mujoco=mujoco.__version__,scipy=scipy.__version__,numpy=numpy.__version__,server=subprocess.check_output(['hostname'],text=True).strip())
(root/'provenance.json').write_text(json.dumps(manifest,indent=2))
