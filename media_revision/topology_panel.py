"""Render the support graph as oriented bricks in a fixed world-space view."""
from functools import lru_cache
from itertools import product

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation

SIZE = (494, 810)
BACKGROUND = '#10202c'
FOREGROUND = '#f1f7fa'
SECONDARY = '#acc2ce'
TARGET = '#ffd165'
BRICK = '#74cdde'
GRID = '#294453'
EPSILON = 1e-9
CORNERS = np.array(list(product((-1, 1), repeat=3)))
FACES = [
    (0, 1, 3, 2), (4, 6, 7, 5),
    (0, 4, 5, 1), (2, 3, 7, 6),
    (0, 2, 6, 4), (1, 5, 7, 3),
]
BASE_Z = .20
VIEW_CENTER = np.array([.44, -.32, .31])
azimuth, elevation = np.radians((-65, 34))
EYE = np.array([np.cos(elevation) * np.cos(azimuth),
                np.cos(elevation) * np.sin(azimuth), np.sin(elevation)])
RIGHT = np.array([-np.sin(azimuth), np.cos(azimuth), 0.])
UP = np.cross(EYE, RIGHT)


@lru_cache(maxsize=None)
def font(size, bold=False):
    style = '-Bold' if bold else ''
    return ImageFont.truetype(f'/usr/share/fonts/truetype/dejavu/DejaVuSans{style}.ttf', size)


def project(point):
    relative = np.asarray(point) - VIEW_CENTER
    perspective = 1.52 / (1.52 - relative @ EYE)
    return np.stack((246 + 491 * perspective * (relative @ RIGHT),
                     384 - 491 * perspective * (relative @ UP)), axis=-1)


def line(draw, points, color, width=1):
    draw.line(list(map(tuple, points)), fill=color, width=width)


def arrow(draw, start, end, color, width=2, head=9):
    direction = end - start
    length = np.linalg.norm(direction)
    if length < head + 1:
        return
    direction /= length
    across = np.array([-direction[1], direction[0]])
    before = end - head * direction
    line(draw, [start, before], color, width)
    draw.polygon([tuple(end), tuple(before + .46 * head * across),
                  tuple(before - .46 * head * across)], fill=color)


def grid(draw):
    x0, x1, y0, y1 = .08, .80, -.55, -.09
    for x in np.linspace(x0, x1, 7):
        line(draw, project([[x, y0, BASE_Z], [x, y1, BASE_Z]]), GRID)
    for y in np.linspace(y0, y1, 5):
        line(draw, project([[x0, y, BASE_Z], [x1, y, BASE_Z]]), GRID)
    datum = np.array([x0, y0, BASE_Z])
    origin = project(datum)
    for label, vector in zip('XYZ', np.eye(3)):
        end = project(datum + .074 * vector)
        arrow(draw, origin, end, '#7b9bab', 2, 6)
        draw.text(tuple(end + [-5, -19]), label, fill=SECONDARY, font=font(14, True))


def draw_bricks(draw, rows, target):
    faces = []
    light = np.array([-.25, -.35, .9])
    for row in rows:
        name = row['brick_id']
        center = np.asarray(row['position_m'])
        quaternion = np.asarray(row['quaternion_wxyz'])
        rotation = Rotation.from_quat(np.roll(quaternion, -1)).as_matrix()
        corners = (CORNERS * (.5 * np.asarray(row['size_m']))) @ rotation.T + center
        base_color = np.array([247, 177, 61] if name == target else [40, 81, 103])
        border = '#ffe3a0' if name == target else '#538092'
        for indices in FACES:
            vertices = corners[list(indices)]
            normal = np.cross(vertices[1] - vertices[0], vertices[2] - vertices[0])
            normal /= np.linalg.norm(normal)
            if (EYE * 1.52 + VIEW_CENTER - vertices[0]) @ normal <= 0:
                continue
            shade = .67 + .37 * max(0, normal @ light)
            fill = tuple((base_color * shade).astype(int))
            faces.append((vertices.mean(axis=0) @ EYE, project(vertices), fill, border))
    for _, points, fill, border in sorted(faces, key=lambda face: face[0]):
        draw.polygon(list(map(tuple, points)), fill=fill, outline=border)


def draw_links(draw, origins, edges, target):
    for source, destination in sorted(edges, key=lambda pair: target in pair):
        source_pos, destination_pos = origins[source], origins[destination]
        direction = destination_pos - source_pos
        distance = np.linalg.norm(direction)
        if distance < EPSILON:
            continue
        direction /= distance
        bright = target in (source, destination)
        start = source_pos + min(6, distance * .15) * direction
        end = destination_pos - min(7, distance * .18) * direction
        arrow(draw, start, end, TARGET if bright else '#91b8c9', 3 if bright else 2, 10 if bright else 8)
    for name, (x, y) in origins.items():
        radius = 6 if name == target else 3
        color = TARGET if name == target else BRICK
        draw.ellipse((x-radius, y-radius, x+radius, y+radius), fill=color, outline=BACKGROUND, width=1)
    if target in origins:
        x, y = origins[target]
        label = target.replace('brick_', 'B')
        label_x = min(max(x + 20, 28), 405)
        label_y = y - 43
        line(draw, [(x, y), (label_x + 7, label_y + 26)], TARGET, 2)
        draw.rounded_rectangle((label_x, label_y, label_x + 67, label_y + 30), 6, fill=TARGET)
        draw.text((label_x + 7, label_y + 3), label, fill=BACKGROUND, font=font(17, True))


def panel(rows, edges, goals, target, extracted=False):
    image = Image.new('RGB', SIZE, BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw.text((28, 23), '3D support graph', font=font(32, True), fill=FOREGROUND)
    draw.text((28, 68), 'Support graph from simulator poses', font=font(19), fill=SECONDARY)
    draw.line((28, 111, 466, 111), fill=GRID, width=2)
    draw.text((28, 134), 'TARGET REMOVED' if extracted else 'SELECTED FOR GRASP',
              font=font(17, True), fill=TARGET)
    draw.text((27, 158), target.replace('brick_', 'B'), font=font(40, True), fill=FOREGROUND)
    draw.text((148, 166), 'Pile updated after lift' if extracted
              else ('Exposed endpoint' if target in goals else 'Eligible target'),
              font=font(18), fill=SECONDARY)
    grid(draw)
    draw_bricks(draw, rows, None if extracted else target)
    origins = {row['brick_id']: project(np.asarray(row['position_m']) + [0., 0., .012])
               for row in rows}
    if origins:
        draw_links(draw, origins, edges, None if extracted else target)
    else:
        draw.text((247, 370), 'Pile cleared', font=font(27, True), fill=FOREGROUND, anchor='mm')
    draw.line((28, 605, 466, 605), fill=GRID, width=2)
    draw.text((28, 628), f'{len(origins)} on pile   /   {len(edges)} support links',
              font=font(23, True), fill=FOREGROUND)
    draw.ellipse((28, 685, 40, 697), fill=TARGET)
    draw.text((53, 680), 'Selected brick', font=font(19), fill=SECONDARY)
    draw.ellipse((260, 685, 272, 697), fill=BRICK)
    draw.text((285, 680), 'Other pile bricks', font=font(19), fill=SECONDARY)
    arrow(draw, np.array([28., 739.]), np.array([45., 739.]), '#91b8c9', 2, 7)
    draw.text((53, 725), 'Lower brick supports upper brick', font=font(19), fill=SECONDARY)
    return image
