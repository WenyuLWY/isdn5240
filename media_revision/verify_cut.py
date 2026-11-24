"""Verify coverage, exact selection frames, and decode the complete cut."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def verify(path, run, audit):
    metadata = json.loads(path.with_suffix('.json').read_text())
    hardware = json.loads(audit.read_text())
    assert metadata['scene_sha256'] == hardware['integrated_scene_sha256']
    assert hashlib.sha256((run/'scene.xml').read_bytes()).hexdigest() == metadata['scene_sha256']
    events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    selected = {}
    placed = [e['brick_id'] for e in events if e['phase'] == 'placed']
    for e in events:
        if e['phase'] == 'select':
            selected.setdefault(e['brick_id'], e['time'])
    chapters = metadata['chapters']
    assert [c['brick'] for c in chapters] == placed and len(placed) == 30
    for chapter in chapters:
        brick = chapter['brick']
        assert abs(chapter['selection_frame'] - selected[brick]) < 2e-6
        assert chapter['start_s'] <= chapter['selection_video_s'] < chapter['lift_video_s']
        assert chapter['lift_video_s'] <= chapter['updated_video_s'] < chapter['end_s']
        assert chapter['picked_sim_s'] <= chapter['updated_sim_s']
        for phase in ('close', 'test_lift'):
            assert chapter['phase_frames'][phase] > 5, chapter
    subprocess.run(['ffmpeg', '-nostdin', '-v', 'error', '-xerror', '-i', str(path),
                    '-f', 'null', '-'], check=True)
    details = json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
        '-show_entries', 'stream=codec_name,width,height,pix_fmt,nb_read_frames:format=duration',
        '-of', 'json', str(path),
    ]))
    stream = details['streams'][0]
    assert stream['codec_name'] == 'h264' and stream['pix_fmt'] == 'yuv420p'
    assert [stream['width'], stream['height']] == metadata['resolution']
    assert int(stream['nb_read_frames']) == metadata['frames']
    assert abs(float(details['format']['duration']) - metadata['duration_s']) < .001
    result = dict(frames=metadata['frames'], duration_s=metadata['duration_s'],
                  sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  all_30_grasps_and_graph_updates=True, exact_selection_time_alignment=True,
                  shortest_selection_to_extraction_s=min(x['updated_video_s'] - x['selection_video_s']
                                                          for x in chapters),
                  all_frames_decode=True, model_matches_bc=True)
    path.with_name(path.stem+'_verification.json').write_text(json.dumps(result, indent=2))
    print(result)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', type=Path)
    parser.add_argument('run', type=Path)
    parser.add_argument('asset_audit', type=Path)
    args = parser.parse_args()
    verify(args.video, args.run, args.asset_audit)
