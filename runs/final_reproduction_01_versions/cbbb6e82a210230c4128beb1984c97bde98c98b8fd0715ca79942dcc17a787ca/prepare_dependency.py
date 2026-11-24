"""Use the installed A code when it matches the pinned modules; otherwise unpack that revision."""
from pathlib import Path
import subprocess,os,json,hashlib,sys,tarfile,io,tempfile
root=Path(__file__).resolve().parent;manifest=json.loads((root/'provenance.json').read_text());repo=Path('/mnt/data/cjx_data/brick-disassembly/code_v52')
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
if digest(Path(manifest['config']))!=manifest['config_sha256']:
 raise RuntimeError('A configuration changed; use the archived config in the repository under DISASSEMBLY_ROOT')
if all(digest(repo/'disassembly'/(k+'.py'))==v for k,v in manifest['A_modules'].items()):
 print(repo);sys.exit(0)
dep=root/'dependencies'/manifest['a_commit']
if not dep.exists():
 dep.parent.mkdir(exist_ok=True)
 with tempfile.TemporaryDirectory(dir=dep.parent) as tmp:
  tmp=Path(tmp);payload=subprocess.check_output(['git','-C',str(repo),'archive',manifest['a_commit'],'disassembly'])
  with tarfile.open(fileobj=io.BytesIO(payload)) as t:t.extractall(tmp)
  runtime=tmp/'disassembly/runtime.py';src=runtime.read_text()
  orig="CODE_COMMIT=subprocess.check_output(['git','-C',str(SOURCE),'rev-parse','HEAD'],text=True).strip()"
  assert orig in src;runtime.write_text(src.replace(orig,"CODE_COMMIT="+repr(manifest['a_commit'])))
  os.rename(tmp,dep)
print(dep)
