"""Render the 120-second lecture cut from the original trajectory."""
import argparse
from bisect import bisect_right
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import time

import egl
import numpy as np

from topology_cut import (
    CLEARANCE, FPS, H, TRANSFER, W, WALL, Film, Graph, event_list, read_tracks,
)


# The slow demonstration and the last-pair clearance have their own frame budgets.
REGULAR = dict(select=16, approach=6, insert=6, close=8, test_lift=10, extract=13,
               handover=9, travel=6, wall=8, return_=3)
FEATURE = dict(select=29, reposition=48, approach=18, insert=16, close=17, test_lift=17,
               extract=20, handover=65, travel=26, wall=34, return_=8)
CLEAR_PAIR = dict(clearance=68, select=16, approach=8, insert=8, close=8, test_lift=12,
                  extract=14, handover=13, travel=8, wall=15, return_=4)
PAUSES = {10: 12, 18: 12, 24: 12}
INTRO = 40
OUTRO = 72
FEATURE_SLOT = 5


def sample_evenly(samples, count):
    if not samples:
        raise ValueError('required sequence has no source frames')
    return [samples[i] for i in np.linspace(0, len(samples)-1, count).round().astype(int)]


def stage_of(phase, separate_reposition):
    if phase.startswith('reposition') and separate_reposition:
        return 'reposition'
    if phase in ('insert', 'close', 'test_lift'):
        return phase
    if phase in ('extract', 'present_ready'):
        return 'extract'
    if phase in TRANSFER:
        return 'handover'
    if phase.startswith('drive_wall'):
        return 'travel'
    if phase in WALL:
        return 'wall'
    if phase.startswith('return'):
        return 'return_'
    return 'approach'


def portion(samples, group, count, feature):
    if group == 'handover' and not feature:
        tight = [x for x in samples if x[2] in ('dual', 'preload', 'unload', 'lift_free', 'rear_only')]
        if tight:
            samples = tight
    if group == 'wall' and not feature:
        tight = [x for x in samples if x[2] in ('release', 'withdraw', 'settle')]
        if tight:
            samples = tight
    if group == 'extract':
        # Follow the lift at first. Keep the final updated graph on screen long
        # enough to spot the removed brick before cutting to the handover.
        motion = max(1, round(count * .57))
        return sample_evenly(samples, motion) + [samples[-1]]*(count-motion)
    return sample_evenly(samples, count)


def render(run, destination):
    started = time.monotonic()
    events = event_list(run)
    placed = [e for e in events if e['phase'] == 'placed']
    cycles = {e['brick_id']: e['time'] for e in events if e['phase'] == 'cycle'}
    histories = defaultdict(dict)
    for e in events:
        histories[e['brick_id']].setdefault(e['phase'], e)
    tracks = read_tracks(run)
    timestamps = [sample[0] for sample in tracks]
    decisions = {(sample[1], round(sample[0], 6)): sample
                 for sample in tracks if sample[2] == 'select'}
    assert len(placed) == 30
    film = Film(run, destination, placed)
    metadata = dict(
        dataset=str(run.resolve()), source='verified integrated experiment',
        scene_sha256=hashlib.sha256((run/'scene.xml').read_bytes()).hexdigest(),
        fps=FPS, resolution=[W, H], renderer=film.renderer, gpu=egl.gpu, chapters=[],
    )
    film.set_frame(tracks[0])
    film.show(tracks[0], tracks[0][1], 0, None, 'overview', repeat=INTRO)
    previous_return = -1.
    for index, event in enumerate(placed, start=1):
        target = event['brick_id']
        history = histories[target]
        decision = history['select']
        picked, held = (history[name]['time'] for name in ('picked', 'A_input'))
        end = cycles[target]
        feature = index == FEATURE_SLOT
        clearance = 'decluttered' in history
        fractions = FEATURE if feature else CLEAR_PAIR if clearance else REGULAR
        chapter_samples = tracks[bisect_right(timestamps, previous_return + 1e-8):
                                  bisect_right(timestamps, end + 1e-8)]
        by_stage = defaultdict(list)
        for sample in chapter_samples:
            phase = sample[2]
            if sample[0] < decision['time'] - 1e-8:
                if clearance and phase in CLEARANCE:
                    by_stage['clearance'].append(sample)
                continue
            if phase != 'select':
                by_stage[stage_of(phase, feature)].append(sample)
        decision_frame = decisions[target, round(decision['time'], 6)]
        aim = np.array(next(r['position_m'] for r in decision['poses']
                            if r['brick_id'] == decision['target']))
        aim[:2] = .64 * aim[:2] + .36 * np.array([.44, -.32])
        aim[2] = .36
        graph = Graph(film, decision, [f"brick_{r['brick_id']:03}" for r in placed[:index-1]],
                      target, picked, held)
        info = dict(brick=target, slot=index, start_s=film.frame/FPS,
                    selection_sim_s=decision['time'], picked_sim_s=picked, held_sim_s=held,
                    phase_frames={})
        for group, count in fractions.items():
            if group == 'select':
                film.set_frame(decision_frame)
                info.update(selection_video_s=film.frame/FPS, selection_frame=decision_frame[0])
                film.show(decision_frame, target, index, aim, 'select',
                          chart=graph.update(decision_frame[0], 'select', selection=True),
                          repeat=count)
                continue
            selected = portion(by_stage[group], group, count, feature)
            for sample in selected:
                stamp, _, phase, _ = sample
                film.set_frame(sample)
                chart = None
                if group in {'reposition', 'approach', 'insert', 'close', 'test_lift', 'extract'}:
                    chart = graph.update(stamp, phase)
                if group == 'test_lift' and 'lift_video_s' not in info:
                    info['lift_video_s'] = film.frame/FPS
                if chart is not None and graph.removed and 'updated_video_s' not in info:
                    info.update(updated_video_s=film.frame/FPS, updated_sim_s=stamp)
                film.show(sample, target, index, aim, phase, chart=chart)
                info['phase_frames'][phase] = info['phase_frames'].get(phase, 0)+1
        if index in PAUSES:
            sample = chapter_samples[-1]
            film.set_frame(sample)
            film.show(sample, target, index, aim, 'course', repeat=PAUSES[index])
        info['end_s'] = film.frame/FPS
        metadata['chapters'].append(info)
        previous_return = end
    film.set_frame(tracks[-1])
    film.show(tracks[-1], placed[-1]['brick_id'], len(placed), None, 'end', repeat=OUTRO,
              key='topology_2min_wall')
    film.finish()
    assert film.frame == 120 * FPS
    metadata.update(frames=film.frame, duration_s=film.frame/FPS,
                    render_wall_s=time.monotonic()-started)
    destination.with_suffix('.json').write_text(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    render(args.run, args.output)
