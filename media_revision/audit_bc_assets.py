"""Verify the integrated replay's robot meshes against the published B+C scene."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

COLLIDERS = {f'{side}_wheel_collision' for side in ('fl', 'fr', 'rl', 'rr')}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scene_nodes(path):
    root = ET.parse(path).getroot()
    vehicle = root.find("worldbody/body[@name='ranger_base_link']")
    meshes = {(m.get('name') or Path(m['file']).stem): m for m in (tag.attrib for tag in root.findall('asset/mesh'))}
    bodies = {body.attrib['name']: body.attrib for body in vehicle.iter('body')}
    geoms = {}
    labels = {}
    for body in vehicle.iter('body'):
        body_name = body.attrib['name']
        for index, geom in enumerate(body.findall('geom')):
            key = (body_name, index)
            geoms[key] = geom.attrib
            labels[key] = geom.get('name', f'{body_name}[{index}]')
    meshdir = root.find('compiler').get('meshdir', '')
    if not Path(meshdir).is_absolute():
        meshdir = path.parent / meshdir
    return meshes, bodies, geoms, labels, Path(meshdir)


def compare(base, integrated):
    mb, bb, gb, lb, db = scene_nodes(base)
    mi, bi, gi, li, di = scene_nodes(integrated)
    robot_geoms = sorted(gb)
    assert set(gb) == set(gi) and bb == bi
    assert {lb[key] for key in robot_geoms if gb[key] != gi[key]} == COLLIDERS and lb == li
    assert all(gi[key]['type'] == 'cylinder' for key in robot_geoms if li[key] in COLLIDERS)
    model_bc = mj.MjModel.from_xml_path(str(base))
    model_abc = mj.MjModel.from_xml_path(str(integrated))
    mesh_names = set()
    for key in robot_geoms:
        name = lb[key]
        if name in COLLIDERS:
            continue
        body, index = key
        src = model_bc.geom(model_bc.body(body).geomadr[0]+index)
        dst = model_abc.geom(model_abc.body(body).geomadr[0]+index)
        for item in ('type', 'group', 'rgba', 'pos', 'quat', 'size'):
            assert np.allclose(getattr(src, item), getattr(dst, item), atol=1e-10), (name, item)
        if gb[key].get('mesh'):
            mesh_names.add(gb[key]['mesh'])
    asset_hashes = {}
    for name in sorted(mesh_names):
        source, destination = dict(mb[name]), dict(mi[name])
        a, b = source.pop('file', None), destination.pop('file', None)
        assert source == destination and bool(a) == bool(b), name
        if a:
            hash_ = sha(db / a)
            assert hash_ == sha(di / b), name
            asset_hashes[name] = hash_
    assert {'ranger_base', 'wheel_v3', 'schmalz_visual'} <= asset_hashes.keys()
    assert model_abc.actuator('fingers_actuator').id >= 0
    assert model_abc.body('rear_arm__vacuum').id >= 0
    assert not any('rear_arm__' in n and 'follower' in n for n in lb.values())
    assert model_abc.joint('ranger_native_freejoint').id >= 0
    return dict(reference='510e8649fc2d25ff6d6f2fa99753d9514c23d7fd',
                reference_scene_sha256=sha(base), integrated_scene_sha256=sha(integrated),
                platform='Ranger Mini V3 / 2 × Kinova Gen3',
                front_tool='Robotiq 2F-85', rear_tool='Schmalz Cobot Pump ECBPi 12',
                matching_body_frames=len(bi), matched_robot_geoms=len(gi)-len(COLLIDERS),
                asset_sha256=asset_hashes,
                integrated_changes=dict(hidden_tire_collision_geoms=sorted(COLLIDERS),
                                        free_chassis=True, independently_steered_and_driven_wheels=4))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bc_scene', type=Path)
    parser.add_argument('integrated_scene', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    report = compare(args.bc_scene, args.integrated_scene)
    args.output.write_text(json.dumps(report, indent=2))
    print(report['matching_body_frames'], 'body frames,', report['matched_robot_geoms'],
          'common geometry elements,', len(report['asset_sha256']), 'byte-matched mesh files')
