"""Flood rescue world: metre units, z=0 ground, z=1.5 water surface.

Run directly for the interactive PyBullet viewer, or use --overview for a PNG.
The water is visual-only; this scene does not model fluid forces or run the
legacy QR mission. Static people, trees and buildings have collision geometry.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

import pybullet as p

ARENA_LENGTH_M = 40.0
ARENA_WIDTH_M = 30.0
ARENA_X_MIN, ARENA_X_MAX = 0.0, 40.0
ARENA_Y_MIN, ARENA_Y_MAX = -15.0, 15.0
WATER_LEVEL_M = 1.5
TAKEOFF_PAD_TOP_M = 1.8
DRONE_START_POS = (20.0, -13.0)
DRONE_START_YAW = math.pi / 2
# Left/right are viewed from the launch pad looking into the arena (+Y).
# The second tree moves into the former middle-left building footprint;
# another replaces the second right building. Two remain behind the centre.
TREE_POSITIONS = ((6.0, -2.0), (34.0, -2.0), (20.0, 4.0), (20.0, 10.0))
HUMAN_POSITIONS = tuple((x + side * 3.0, y)
                        for x, y in TREE_POSITIONS for side in (-1, 1))
# x, y, width, depth, height. Move the 8 m left building to the centre.
BUILDING_SPECS = (
    (5.0, -10.0, 6.0, 5.0, 4.0),
    (20.0, -2.0, 7.0, 5.0, 8.0),
    (5.0, 7.0, 6.0, 6.0, 12.0),
    (11.0, 11.0, 3.0, 4.0, 5.5),
    (35.0, -10.0, 6.0, 5.0, 10.0),
    (35.0, 7.0, 6.0, 6.0, 7.5),
    (29.0, 11.0, 3.0, 4.0, 11.0),
)


def _shape(kind, position, color, *, collidable=True, **geometry):
    visual = p.createVisualShape(kind, rgbaColor=color, **geometry)
    collision = p.createCollisionShape(kind, **geometry) if collidable else -1
    return p.createMultiBody(0, collision, visual, basePosition=position)


def _box(position, half, color, *, collidable=True):
    return _shape(p.GEOM_BOX, position, color, halfExtents=half, collidable=collidable)


def _sphere(position, radius, color):
    return _shape(p.GEOM_SPHERE, position, color, radius=radius)


def _cylinder(position, radius, height, color):
    # PyBullet uses length for visuals, height for collision cylinders.
    visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
    collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    return p.createMultiBody(0, collision, visual, basePosition=position)


def build_tree(x, y):
    """A 6.6 m tree with solid trunk, branches and canopy envelopes."""
    brown, green = (.30, .16, .07, 1), (.13, .40, .16, 1)
    return [
        _cylinder((x, y, 2.3), .28, 4.6, brown),
        _box((x - .55, y, 3.8), (.65, .13, .13), brown),
        _box((x + .55, y, 4.2), (.65, .13, .13), brown),
        _sphere((x, y, 4.9), 1.7, green),
        _sphere((x - .95, y, 4.4), 1.15, (.18, .48, .20, 1)),
        _sphere((x + .95, y + .15, 4.7), 1.15, (.20, .44, .13, 1)),
    ]


def build_human(x, y, index):
    """Standing 1.9 m human, feet on ground, head and raised arms above water."""
    skin = (.72, .46, .29, 1)
    shirt = ((.98, .48, .06, 1), (.95, .8, .12, 1), (.7, .12, .6, 1))[index % 3]
    ids = [
        _box((x - .12, y, .43), (.09, .12, .43), (.10, .14, .22, 1)),
        _box((x + .12, y, .43), (.09, .12, .43), (.10, .14, .22, 1)),
        _box((x, y, 1.16), (.24, .15, .32), shirt),
        _cylinder((x, y, 1.51), .075, .12, skin),
        _sphere((x, y, 1.72), .18, skin),
    ]
    for side in (-1, 1):
        ids.append(_box((x + side * .30, y, 1.5), (.07, .085, .32), shirt))
        ids.append(_sphere((x + side * .30, y, 1.87), .085, skin))
    # Small ripples make the partially submerged people easy to locate.
    for dx, dy in ((.55, 0), (-.55, 0), (0, .42), (0, -.42)):
        ids.append(_box((x + dx, y + dy, WATER_LEVEL_M + .008),
                        (.14, .025, .003), (.55, .85, .9, 1), collidable=False))
    return ids


def build_building(spec, index):
    x, y, width, depth, height = spec
    palette = ((.58, .65, .70, 1), (.70, .62, .49, 1), (.57, .61, .54, 1))
    ids = [_box((x, y, height / 2), (width / 2, depth / 2, height / 2), palette[index % 3])]
    ids.append(_box((x, y, height + .10), (width / 2 + .12, depth / 2 + .12, .10), (.30, .34, .37, 1)))
    for z in range(2, math.ceil(height), 2):
        for dx in (-width / 4, width / 4):
            for side in (-1, 1):
                ids.append(_box((x + dx, y + side * (depth / 2 + .012), z + .15),
                                (.38, .018, .42), (.12, .25, .31, 1), collidable=False))
    return ids


def build_arena_only(connect_mode=p.GUI, takeoff_xy=DRONE_START_POS):
    """Return (client_id, collidable_body_ids); call on an independent client."""
    client_id = p.connect(connect_mode)
    if client_id < 0:
        raise RuntimeError('PyBullet could not connect')
    try:
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240)
        _box((20, 0, -.1), (20, 15, .1), (.29, .28, .23, 1))
        # Exact 1.5 m inundation from the original ground datum. Water has no
        # collision shape: aerial ray casts must see the solid obstacles.
        _box((20, 0, WATER_LEVEL_M / 2), (20, 15, WATER_LEVEL_M / 2),
             (.12, .46, .61, .88), collidable=False)
        for y in (-15, 15):
            _box((20, y, 1.52), (20, .045, .015), (.95, .81, .2, 1), collidable=False)
        for x in (0, 40):
            _box((x, 0, 1.52), (.045, 15, .015), (.95, .81, .2, 1), collidable=False)
        # Sparse surface highlights, all visual-only.
        for x in range(2, 40, 3):
            for y in range(-13, 15, 3):
                _box((x, y, 1.504), (.32, .018, .002), (.29, .63, .73, 1), collidable=False)
        tx, ty = takeoff_xy
        _box((tx, ty, TAKEOFF_PAD_TOP_M / 2), (1.25, 1.25, TAKEOFF_PAD_TOP_M / 2), (.23, .25, .28, 1))
        _box((tx, ty, TAKEOFF_PAD_TOP_M + .012), (1.2, 1.2, .012), (.13, .27, .69, 1), collidable=False)
        for dx in (-.4, .4):
            _box((tx + dx, ty, 1.83), (.07, .6, .008), (1, 1, 1, 1), collidable=False)
        _box((tx, ty, 1.83), (.4, .07, .008), (1, 1, 1, 1), collidable=False)
        for position in TREE_POSITIONS:
            build_tree(*position)
        for index, position in enumerate(HUMAN_POSITIONS):
            build_human(*position, index)
        for index, spec in enumerate(BUILDING_SPECS):
            build_building(spec, index)
        collidables = [p.getBodyUniqueId(i) for i in range(p.getNumBodies())
                       if p.getCollisionShapeData(p.getBodyUniqueId(i), -1)]
        return client_id, collidables
    except BaseException:
        p.disconnect(client_id)
        raise


def render_overview(output):
    import numpy as np
    from PIL import Image, ImageDraw

    client_id, _ = build_arena_only(p.DIRECT)
    try:
        view = p.computeViewMatrix((57, -49, 46), (20, 0, 1.5), (0, 0, 1))
        projection = p.computeProjectionMatrixFOV(50, 1.4, .1, 150)
        rgba = p.getCameraImage(1400, 1000, view, projection, renderer=p.ER_TINY_RENDERER)[2]
        image = Image.fromarray(np.asarray(rgba, dtype=np.uint8).reshape(1000, 1400, 4)[:, :, :3])
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, 1400, 65), fill=(20, 33, 43))
        draw.text((24, 14), 'FLOOD RESCUE WORLD | 40 x 30 m | Water: 1.50 m above ground', fill='white', font_size=23)
        draw.text((24, 43), '4 person-tree-person groups | 1 centre building + 6 side buildings | Heights: 4-12 m', fill='white', font_size=16)
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        image.save(output)
        return output
    finally:
        p.disconnect(client_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--overview', type=Path, help='Render PNG without opening a GUI')
    args = parser.parse_args()
    if args.overview:
        print(render_overview(args.overview))
        return
    client_id, collidables = build_arena_only()
    p.resetDebugVisualizerCamera(42, 35, -42, (20, 0, 1.5))
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    print(f'Flood world: water 1.5 m, {len(HUMAN_POSITIONS)} people, {len(TREE_POSITIONS)} trees, {len(BUILDING_SPECS)} buildings; {len(collidables)} solid bodies.')
    print('Scene viewer only. Ctrl+C to close. Existing QR mission is separate.')
    try:
        while p.isConnected(client_id):
            p.stepSimulation()
            time.sleep(1 / 240)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected(client_id):
            p.disconnect(client_id)


if __name__ == '__main__':
    main()
