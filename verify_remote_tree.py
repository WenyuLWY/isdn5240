"""Match every packaged byte to the blob identities in the fetched GitHub tree."""
from pathlib import Path
import subprocess,hashlib,json,sys
rev=sys.argv[1];base=Path('/home/cjx/disassembly/abc_course_final');bare=base/'remote_readback';directory='course_demo/abc_integrated'
root=Path('/mnt/data/cjx_data/brick-disassembly/code_v52')/directory
manifest=json.loads((root/'SHA256.json').read_text());manifest['SHA256.json']=hashlib.sha256((root/'SHA256.json').read_bytes()).hexdigest()
records=subprocess.check_output(['git','-C',str(bare),'ls-tree','-rz',rev,'--',directory]).split(b'\0')
tree={};total=0
for r in records:
 if not r:continue
 prefix,path=r.split(b'\t',1);name=path.decode()[len(directory)+1:];tree[name]=prefix.split()[2].decode()
assert set(tree)==set(manifest),(set(manifest)-set(tree),set(tree)-set(manifest))
for name,expected in manifest.items():
 b=(root/name).read_bytes();assert hashlib.sha256(b).hexdigest()==expected,name
 assert hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()==tree[name],name
 total+=len(b)
dest=base/'verified_download'/rev;dest.mkdir(exist_ok=True,parents=True);rec=dict(commit=rev,files=len(tree),bytes=total,all_match=True)
(dest/'tree_attestation.json').write_text(json.dumps(rec,indent=2))
print(rec)
