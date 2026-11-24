"""Cut the verified ABC trajectory with a synchronized 3D panel for every grasp."""
import argparse
import hashlib
import json
import subprocess
import time
from types import SimpleNamespace
from bisect import bisect_right
from collections import defaultdict
from pathlib import Path

import egl
import mujoco as mj
import numpy as np
from OpenGL.GL import GL_RENDERER, glGetString
from PIL import Image, ImageDraw

from axes import SupportGraph, observe
from topology_panel import BACKGROUND, FOREGROUND, SECONDARY, TARGET, SIZE, font, panel

FPS = 25
HY, W = 810, 1440
LEFT = 928
H = 900
GRASP = {'approach', 'align', 'insert', 'close', 'test_lift', 'extract',
         'present_ready', 'calibrated_open', 'open_for_pick', 'open_retry',
         'retreat_retry', 'settle_retry', 'reposition', 'reposition_waypoint',
         'clear_chassis', 'parking_settle', 'select'}
CLEARANCE = {'approach_clearance', 'push_clearance', 'withdraw_clearance', 'settle_clearance'}
TRANSFER = {'present', 'rear_pre', 'rear_align', 'preload', 'dual', 'unload',
            'lift_free', 'clear', 'rear_only'}
WALL = {'carry', 'lower', 'release', 'withdraw', 'settle', 'wall_clear'}
LABELS = {
    'overview': 'Ranger Mini V3   /   Dual Kinova Gen3',
    'select': 'Select an exposed brick',
    'reposition': 'Move to a reachable pose',
    'align': 'Align to the brick',
    'insert': 'Insert the gripper',
    'close': 'Close the Robotiq 2F-85',
    'test_lift': 'Verify that both fingers hold the brick',
    'extract': 'Lift clear of the pile',
    'present_ready': 'Lift clear of the pile',
    'reposition_clearance': 'Reposition to clear the grasp corridor',
    'prepare_clearance': 'Approach the blocked pair',
    'approach_clearance': 'Approach the blocked pair',
    'push_clearance': 'Open a grasp corridor with the finger pads',
    'withdraw_clearance': 'Withdraw and reobserve the pile',
    'settle_clearance': 'Reobserve the pile',
    'unload': 'Release the front gripper after suction holds',
    'preload': 'Check load on the Schmalz suction tool',
    'rear_align': 'Seat the Schmalz suction cup',
    'rear_only': 'Verify rear-only hold',
    'travel': 'Drive to the wall',
    'release': 'Vent suction after the brick is supported',
    'withdraw': 'Lift the suction tool away',
    'settle': 'Verify the placement',
    'course': 'Completed course',
    'end': 'Three stretcher courses and one header course',
}


def phase_name(phase):
    if phase.startswith('return'):
        return 'Return to the pile'
    if phase.startswith('drive_wall'):
        return LABELS['travel']
    if phase.startswith('reposition') and phase != 'reposition_clearance':
        return LABELS['reposition']
    if phase in LABELS:
        return LABELS[phase]
    if phase in TRANSFER:
        return 'Transfer the same brick between both arms'
    if phase in WALL:
        return 'Place on the supported wall slot'
    return 'Approach the selected brick' if phase in GRASP else 'Prepare the next move'


def playback(phase, feature):
    if phase in CLEARANCE:
        return 1.6
    if phase in {'close', 'test_lift', 'select'}:
        return 1.
    if phase == 'extract':
        return 1.35
    if phase == 'insert':
        return 1.6
    if phase in {'approach', 'align'}:
        return 2.4
    if phase.startswith('reposition') or phase in {'clear_chassis', 'parking_settle'}:
        return 3.0 if feature else 6.
    if phase in GRASP:
        return 5.
    if phase.startswith('drive_wall'):
        return 4.5 if feature else 16.
    if phase.startswith('return'):
        return 20.
    if phase in {'preload', 'unload', 'release', 'withdraw'}:
        return 1.35 if feature else 4.
    if phase in TRANSFER or phase in WALL:
        return 4. if feature else 13.
    return 9.


def event_list(run):
    with (run / 'events.jsonl').open() as log:
        return [json.loads(line) for line in log if line.strip()]


def read_tracks(run):
    segments = [file for file in sorted((run / 'segments').glob('*.npz'))
                if not file.name.endswith('.force.npz')]
    tracks = []
    for file in segments:
        with np.load(file) as part:
            tracks.extend((float(t), int(i), str(phase), q)
                          for t, i, phase, q in zip(part['time'], part['brick_id'],
                                                   part['phase'], part['qpos']))
    assert tracks and all(a[0] <= b[0] + 1e-8 for a, b in zip(tracks, tracks[1:]))
    return tracks


class Film:
    def __init__(self, run, output, placed):
        self.model = mj.MjModel.from_xml_path(str(run / 'scene.xml'))
        self.data = mj.MjData(self.model)
        self.model.vis.global_.offwidth = W
        self.model.vis.global_.offheight = HY
        self.tint_floor()
        self.wide = mj.Renderer(self.model, HY, W)
        self.narrow = mj.Renderer(self.model, HY, LEFT)
        self.camera = mj.MjvCamera()
        mj.mjv_defaultCamera(self.camera)
        self.placed = [event['time'] for event in placed]
        self.output = output
        self.frame = 0
        self.encoder = subprocess.Popen([
            'ffmpeg', '-nostdin', '-v', 'error', '-y', '-f', 'rawvideo',
            '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-',
            '-c:v', 'libx264', '-preset', 'veryfast', '-threads', '3',
            '-crf', '21', '-maxrate', '1900k', '-bufsize', '3400k',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output),
        ], stdin=subprocess.PIPE)
        self.renderer = glGetString(GL_RENDERER).decode()

    def tint_floor(self):
        model = self.model
        texture = model.texture('tile').id
        start = model.tex_adr[texture]
        length = model.tex_width[texture] * model.tex_height[texture] * 3
        pixels = model.tex_rgb[start:start + length].reshape(-1, 3)
        dark = pixels[:, 0] < 128
        pixels[dark] = [143, 156, 163]
        pixels[~dark] = [173, 183, 189]

    def set_frame(self, sample):
        self.data.qpos[:] = sample[3]
        self.data.time = sample[0]
        mj.mj_forward(self.model, self.data)

    def camera_pose(self, phase, target, aim, chart):
        brick = self.data.body(f'brick_{target:03}').xpos
        camera = self.camera
        if phase in ('course', 'end'):
            camera.lookat[:] = [.288, 1.37, .055]
            camera.distance, camera.azimuth, camera.elevation = 1.16, 14, -56
        elif phase in WALL:
            camera.lookat[:] = brick + [0, 0, .060]
            camera.distance, camera.azimuth, camera.elevation = .93, 146, -28
        elif phase in TRANSFER:
            camera.lookat[:] = brick + [0, 0, .060]
            camera.distance, camera.azimuth, camera.elevation = .80, -138, -16
        elif phase in ('overview',) or phase.startswith(('return', 'drive', 'reposition')):
            camera.lookat[:] = [(.25 if chart else .18), (.03 if chart else .47), .27]
            camera.distance, camera.azimuth, camera.elevation = (2.05 if chart else 2.94), 154, -35
        elif chart or phase in CLEARANCE:
            camera.lookat[:] = aim
            camera.distance, camera.azimuth, camera.elevation = 1.13, 82, -40
        else:
            camera.lookat[:] = [.44, -.23, .32]
            camera.distance, camera.azimuth, camera.elevation = 1.48, 81, -39

    def show(self, sample, target, slot, aim, phase, speed=1., chart=None, repeat=1, key=None):
        timestamp = sample[0]
        self.camera_pose(phase, target, aim, chart is not None)
        geom = self.model.geom(f'brick_{target:03}_geom').id
        color = self.model.geom_rgba[geom].copy()
        if chart is not None or phase == 'push_clearance':
            self.model.geom_rgba[geom] = [1., .65, .055, 1.]
        renderer = self.narrow if chart is not None else self.wide
        renderer.update_scene(self.data, camera=self.camera)
        im = Image.new('RGB', (W, H), BACKGROUND)
        im.paste(Image.fromarray(renderer.render()))
        self.model.geom_rgba[geom] = color
        draw = ImageDraw.Draw(im)
        if chart is not None:
            im.paste(chart, (W - SIZE[0], 0))
            draw.line((LEFT + 8, 0, LEFT + 8, HY), fill='#294453', width=2)
            if phase not in {'extract', 'present_ready'}:
                view = renderer.scene.camera[0]
                facing, up = np.asarray(view.forward), np.asarray(view.up)
                eye = (view.pos + renderer.scene.camera[1].pos) / 2
                ray = self.data.body(f'brick_{target:03}').xpos - eye
                depth = ray @ facing
                focal = HY / (2 * np.tan(np.radians(self.model.vis.global_.fovy) / 2))
                if depth > .015:
                    x = LEFT/2 + focal*(ray @ np.cross(facing, up))/depth
                    y = HY/2 - focal*(ray @ up)/depth
                    if 38 < x < LEFT-42 and 70 < y < HY-40:
                        label_x = min(x+29, LEFT-95)
                        draw.line([(x,y),(label_x+4,y-27)], fill=TARGET, width=3)
                        draw.rounded_rectangle((label_x,y-51,label_x+68,y-23), 6, fill=BACKGROUND)
                        draw.text((label_x+7,y-49), f'B{target:03}', font=font(18,True), fill=TARGET)
        done = bisect_right(self.placed, timestamp + 1e-7)
        draw.line((28, HY, W - 28, HY), fill='#385568', width=2)
        text = phase_name(phase)
        if phase == 'course':
            text += f'  /  {done} bricks in place'
        draw.text((28, HY + 11), text, font=font(27, True), fill=FOREGROUND)
        details = f'B{target:03}  /  BRICK {slot:02} OF 30' if slot else 'PILE TO WALL'
        if speed > 1.001:
            details += f'  /  {speed:g}×'
        draw.text((29, HY + 53), details, font=font(16, True), fill=TARGET)
        draw.text((W - 28, HY + 14), f'{done:02}/30 placed', font=font(25, True),
                  fill=FOREGROUND, anchor='ra')
        draw.text((W - 28, HY + 54), f'{timestamp:.2f} s sim', font=font(16),
                  fill=SECONDARY, anchor='ra')
        if key:
            im.save(self.output.with_name(key + '.jpg'), quality=93)
        blob = im.tobytes()
        for _ in range(repeat):
            self.encoder.stdin.write(blob)
        self.frame += repeat

    def finish(self):
        self.encoder.stdin.close()
        assert self.encoder.wait() == 0
        self.narrow.close()
        self.wide.close()


class Graph:
    def __init__(self, film, choice, excluded, target, picked, held):
        self.film, self.choice = film, choice
        self.support = SupportGraph(film.model)
        self.excluded = excluded
        self.name = f'brick_{target:03}'
        self.picked, self.held = picked, held
        self.removed = False
        self.finished = None
        self.rows = None
        self.signature = None
        self.image = None
        self.last = -1e6

    def update(self, timestamp, phase, selection=False):
        film = self.film
        if not self.removed and timestamp >= self.picked:
            name = self.name + '_geom'
            any_support = any(
                name in pair and any(other != name and
                                     (other.startswith('brick_') or other == 'work_table')
                                     for other in pair)
                for pair in ((film.model.geom(c.geom1).name, film.model.geom(c.geom2).name)
                             for c in film.data.contact))
            if (phase in {'extract', 'present_ready'} and not any_support) or timestamp >= self.held:
                self.removed = True
                self.finished = timestamp
        if not selection and self.image is not None and timestamp - self.last < .32 and self.finished != timestamp:
            return self.image
        self.last = timestamp
        if selection:
            # The decision frame is an exact event, not an interpolated support graph.
            source = self.choice['graph']
            rows = self.choice['poses']
        else:
            rows = observe(SimpleNamespace(m=film.model, d=film.data))
            source = self.support.update(film.model, film.data, rows,
                                         self.excluded + ([self.name] if self.removed else []))
        excluded = set(self.excluded)
        if self.removed:
            excluded.add(self.name)
        visible = [row for row in rows if row['brick_id'] not in excluded]
        names = {row['brick_id'] for row in visible}
        edges = sorted((edge['lower'], edge['upper']) for edge in source['edges']
                       if edge['lower'] in names and edge['upper'] in names)
        poses = np.array([row['position_m'] + row['quaternion_wxyz'] for row in visible])
        signature = (tuple(sorted(names)), tuple(edges), self.removed)
        if self.signature != signature or np.max(abs(poses - self.rows), initial=0.) > .0014:
            self.signature = signature
            self.rows = poses
            self.image = panel(visible, edges, source['endpoints'], self.name, self.removed)
        return self.image


def render(run, output, only_slot=None):
    events = event_list(run)
    placed = [event for event in events if event['phase'] == 'placed']
    by_brick = defaultdict(list)
    cycles = {event['brick_id']: event['time'] for event in events if event['phase'] == 'cycle'}
    for event in events:
        by_brick[event['brick_id']].append(event)
    tracks = read_tracks(run)
    ts = [sample[0] for sample in tracks]
    decisions = {(sample[1], round(sample[0], 6)): sample
                 for sample in tracks if sample[2] == 'select'}
    film = Film(run, output, placed)
    movie = dict(dataset=str(run.resolve()), scene_sha256=hashlib.sha256((run/'scene.xml').read_bytes()).hexdigest(),
                 fps=FPS, resolution=[W, H], renderer=film.renderer, gpu=egl.gpu, chapters=[])
    start = time.monotonic()
    if only_slot is None:
        initial = tracks[0]
        film.set_frame(initial)
        film.show(initial, initial[1], 0, None, 'overview', repeat=55)
    previous = -1.
    for index, placed_event in enumerate(placed, start=1):
        target = placed_event['brick_id']
        brick_events = by_brick[target]
        chosen = next(event for event in brick_events if event['phase'] == 'select')
        picked = next(event['time'] for event in brick_events if event['phase'] == 'picked')
        held = next(event['time'] for event in brick_events if event['phase'] == 'A_input')
        last = cycles[target]
        if only_slot is not None and index != only_slot:
            previous = last
            continue
        aim = np.array(next(row['position_m'] for row in chosen['poses']
                            if row['brick_id'] == chosen['target']))
        aim[:2] = .64 * aim[:2] + .36 * np.array([.44, -.32])
        aim[2] = .36
        chapter = dict(brick=target, slot=index, start_s=film.frame/FPS,
                       selection_sim_s=chosen['time'], picked_sim_s=picked,
                       held_sim_s=held, phase_frames={})
        after_key = close_key = False
        budget = .04
        graph = Graph(film, chosen, [f"brick_{e['brick_id']:03}" for e in placed[:index-1]],
                      target, picked, held)
        in_chapter = tracks[bisect_right(ts, previous + 1e-8):bisect_right(ts, last + 1e-8)]
        selection_drawn = False
        last_sample = None
        for sample in in_chapter:
            tt, original_id, phase, _ = sample
            if not selection_drawn and tt + 1e-8 >= chosen['time']:
                selection = decisions[target, round(chosen['time'], 6)]
                film.set_frame(selection)
                chart = graph.update(selection[0], 'select', selection=True)
                hold = 39 if index in (1, 5, 18, 29, 30) else 29
                chapter['selection_video_s'] = film.frame / FPS
                chapter['selection_frame'] = selection[0]
                film.show(selection, target, index, aim, 'select', chart=chart, repeat=hold,
                          key=f'preview_{index:02}_selection' if index in (1, 5, 18, 29, 30) else None)
                selection_drawn = True
            if phase == 'select' or (last_sample is not None and tt - last_sample < 1e-8):
                continue
            last_sample = tt
            speed = playback(phase, index in (1, 5))
            show_graph = selection_drawn and tt <= held + .04 and phase in GRASP
            budget += .04 / speed
            if budget + 1e-8 < 1/FPS:
                continue
            budget -= 1/FPS
            if not selection_drawn and phase not in CLEARANCE and phase not in {'reposition_clearance', 'prepare_clearance'}:
                continue
            film.set_frame(sample)
            chart = graph.update(tt, phase) if show_graph else None
            if selection_drawn and not after_key and tt >= held and chart is None:
                # If an extraction ended between display samples, use this real pose.
                chart = graph.update(tt, 'present_ready')
                phase = 'present_ready'
            if chart is not None and graph.removed and not after_key:
                chapter['updated_video_s'] = film.frame / FPS
                chapter['updated_sim_s'] = tt
                after_key = True
                film.show(sample, target, index, aim, phase, chart=chart, repeat=24,
                          key=f'preview_{index:02}_after' if index in (1, 5, 18, 29, 30) else None)
            else:
                active = target if selection_drawn else original_id
                key = None
                if not close_key and phase == 'test_lift':
                    chapter['lift_video_s'] = film.frame / FPS
                    if index in (1, 5, 18, 29, 30):
                        key = f'preview_{index:02}_lift'
                    close_key = True
                film.show(sample, active, index, aim, phase, speed=speed, chart=chart, key=key)
            chapter['phase_frames'][phase] = chapter['phase_frames'].get(phase, 0) + 1
        assert selection_drawn and after_key and close_key, (index, chapter, graph.finished)
        if index in (10, 18, 24) and only_slot is None:
            sample = in_chapter[-1]
            film.set_frame(sample)
            film.show(sample, target, index, aim, 'course', repeat=33)
        chapter['end_s'] = film.frame/FPS
        movie['chapters'].append(chapter)
        with output.with_suffix('.progress.json').open('w') as log:
            json.dump(dict(slot=index, frames=film.frame, seconds=time.monotonic()-start), log)
        previous = last
    if only_slot is None:
        film.set_frame(tracks[-1])
        film.show(tracks[-1], placed[-1]['brick_id'], len(placed), None, 'end', repeat=86,
                  key='topology_wall_result')
    film.finish()
    movie.update(frames=film.frame, duration_s=film.frame/FPS,
                 render_wall_s=time.monotonic()-start)
    output.with_suffix('.json').write_text(json.dumps(movie, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--only-slot', type=int)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    render(args.run, args.output, args.only_slot)
