from pathlib import Path
import copy,hashlib,json,xml.etree.ElementTree as ET
import mujoco as mj
import numpy as np
from disassembly.mobile_ranger import model_from_parked

ROOT=Path(__file__).resolve().parent
SOURCE=Path('/mnt/data/cjx_data/brick-disassembly/runs/ranger_v33/main01/scenes/RANGER33/initial_snapshot/snapshot.json')

def build(out):
    record=json.loads(SOURCE.read_text())
    state=Path(record['state_path'])
    assert hashlib.sha256(state.read_bytes()).hexdigest()==record['state_sha256']
    original=ET.parse(record['xml_path']); old=mj.MjModel.from_xml_path(record['xml_path']);d=mj.MjData(old)
    mj.mj_setState(old,d,np.load(state)['integration'],record['state_spec']);mj.mj_forward(old,d)
    tree=ET.parse(ROOT/'prototype.xml');w=tree.find('worldbody')
    for n in ['pick_table','wall_course']:w.remove(w.find(f"geom[@name='{n}']"))
    for c in original.findall('worldbody/camera'):
        k=w.find(f"camera[@name='{c.get('name')}']")
        if k is not None:w.remove(k)
        w.append(copy.deepcopy(c))
    w.append(copy.deepcopy(original.find("worldbody/geom[@name='work_table']")))
    base=w.find("body[@name='ranger_base_link']");base.remove(base.find("joint[@name='base_slide']"))
    tree.find('actuator').remove(tree.find(".//actuator/position[@name='base_drive']"))
    w.remove(w.find("body[@name='brick_000']"));contact=tree.find('contact')
    pairs=list(contact.findall('pair'))
    for q in pairs:contact.remove(q)
    rows=[]
    for i in range(30):
        name=f'brick_{i:03d}';b=copy.deepcopy(original.find(f".//body[@name='{name}']"));v=d.body(name);xyz=v.xpos.copy();q=v.xquat.copy()
        assert .08<xyz[0]<.8 and -.55<xyz[1]<-.09
        b.set('pos',' '.join(map(str,xyz)));b.set('quat',' '.join(map(str,q)));w.append(b)
        for pair in pairs:
            e=copy.deepcopy(pair);e.set('geom1',name+'_geom');contact.append(e)
        qadr=old.jnt_dofadr[old.body_jntadr[old.body(name).id]]
        rows.append(dict(id=name,xyz=xyz.tolist(),quat=q.tolist(),velocity=d.qvel[qadr:qadr+6].tolist()))
    from vacuum import install_tool
    install_tool(tree,30)
    intermediate=out.with_name('parked.xml');tree.write(intermediate);model_from_parked(intermediate,out)
    (out.parent/'source.json').write_text(json.dumps(dict(snapshot=str(SOURCE),snapshot_id=record['snapshot_id'],state_sha256=record['state_sha256'],scene_xml=record['xml_path'],bricks=rows),indent=1))
