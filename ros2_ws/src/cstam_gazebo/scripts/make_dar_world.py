#!/usr/bin/env python3
"""Generate the "Dar TuniBot" restaurant world for Gazebo Harmonic.

A Tunisian courtyard house (dar) turned into a romantic, classy fine-dining
restaurant, evening mood:
  - central courtyard with a grand 3-tier ivory marble fountain, gold-trimmed
    columns, soft fairy lights overhead, and an ivory/black marble inlay floor
  - kitchen behind a gold-framed door, Bey's Salon (VIP) behind a narrow
    gold-framed arch topped with a gold medallion
  - red carpet with gold edges from the entrance to the courtyard
  - fine-dining round tables: floor-length linen, high-back velvet chairs, full
    place settings, a low rose centrepiece with a glass candle, and a two-tier
    crystal chandelier overhead (champagne chairs; burgundy + gold in the VIP)

Everything is built from simple shapes (boxes / cylinders / spheres), so no mesh
files are needed. To change the layout, edit the numbers in the LAYOUT section and run:

    python3 ros2_ws/src/cstam_gazebo/scripts/make_dar_world.py

It writes ros2_ws/src/cstam_gazebo/worlds/dar_tunibot.world.
All coordinates are in metres, Gazebo world frame. The building spans x -11..11, y -8..8.
"""
import math
import os

# ============================== LAYOUT ==============================
WALL_H = 3.0          # wall height
WALL_T = 0.2          # wall thickness

# Wall segments (x1, y1, x2, y2). Gaps in the lines are the doors.
WALLS = [
    # outer walls, with a 2 m entrance in the south wall (x -1..1)
    (-11, 8, 11, 8), (-11, -8, -11, 8), (11, -8, 11, 8),
    (-11, -8, -1, -8), (1, -8, 11, -8),
    # kitchen (x -11..-6, y 2..8), door on its east wall at y 4.6..6.0 (1.4 m)
    (-11, 2, -6, 2), (-6, 2, -6, 4.6), (-6, 6.0, -6, 8),
    # Bey's Salon / VIP (x 6..11, y 3..8), narrow arched door at x 6.9..8.0 (1.1 m)
    (6, 3, 6, 8), (6, 3, 6.9, 3), (8.0, 3, 11, 3),
]
# Doors: (name, x, y, width, along 'x' or 'y') -> blue frame + lintel above
DOORS = [
    ('kitchen_door', -6.0, 5.3, 1.4, 'y'),
    ('salon_arch', 7.45, 3.0, 1.1, 'x'),
    ('entrance', 0.0, -8.0, 2.0, 'x'),
]

FOUNTAIN = (0.0, 1.0)                     # centre of the courtyard fountain
COLUMNS = [(-4, -2), (0, -2), (4, -2), (-4, 4), (0, 4), (4, 4), (-4, 1), (4, 1)]
COURTYARD = (-4, -2, 4, 4)                # x0, y0, x1, y1 (marble inlay floor)
TERRACE = (-11, -8, 11, -4.4)

# Round tables: name -> (x, y). Cloth radius 0.55, four poufs around it.
TABLES = {
    'table_1': (-2.3, 6.4), 'table_2': (2.3, 6.4),        # north arcade
    'table_3': (8.3, 1.0), 'table_4': (8.3, -2.3),        # east arcade
    'table_5': (-7.2, -6.3), 'table_6': (-3.6, -6.3),     # jasmine terrace
    'table_7': (3.6, -6.3), 'table_8': (7.2, -6.3),
    'table_9': (-7.6, 0.0), 'table_10': (-7.6, -2.9),     # west arcade
    'vip_1': (9.3, 6.4), 'vip_2': (9.3, 4.2),             # Bey's salon
}
# Boxes: name -> (cx, cy, sx, sy, height, colour)
BOXES = {
    'pickup_counter': (-8.5, 7.55, 4.0, 0.6, 0.95, 'steel'),
    'oven': (-10.6, 4.5, 0.6, 3.0, 1.0, 'steel'),
    'mint_tea_bar': (10.65, -1.0, 0.5, 4.5, 1.1, 'floor'),
    'planter_west': (-7.25, -4.4, 4.5, 0.4, 0.6, 'terracotta'),
    'planter_east': (7.25, -4.4, 4.5, 0.4, 0.6, 'terracotta'),
}
DOCK = (-10.8, -1.45)                     # charging station against the west wall

# ============================== DETAIL SETTINGS ==============================
# Lower these if Gazebo gets slow (software rendering).
FOUNTAIN_JETS = 8                         # thin glowing water jets in the lower basin
HEDGE_ROSE_SPACING = 0.30                 # metres between roses along each of 2 rows per planter
FAIRY_LIGHTS = True                       # bulbs strung across the courtyard
FAIRY_ROWS_Y = (-2.0, 4.0)                # one string along each arcade edge
FAIRY_BULBS_PER_ROW = 7
FAIRY_GLOW = (0.55, 0.48, 0.36)           # soft warm white
PLACE_SETTINGS = True                     # plate, charger, napkin, cutlery, 2 glasses per chair
CHAIR_LEGS = True                         # thin gold chair legs (4 per chair)
CENTREPIECE_ROSES = 6                     # roses around the candle in each table's gold bowl
CHAND_CRYSTALS_OUTER = 8                  # chandelier crystals, lower/outer tier
CHAND_CRYSTALS_INNER = 5                  # chandelier crystals, upper/inner tier

# ============================== COLOURS ==============================
# Romantic / classy "Dar TuniBot" palette: cream + gold trim, champagne floor,
# black-and-white marble courtyard, burgundy velvet for regular chairs/cloths.
C = {
    'white': (0.93, 0.87, 0.76),        # cream walls
    'ivory': (0.95, 0.92, 0.84),        # ivory marble (fountain, courtyard floor, column shafts)
    'champagne': (0.86, 0.78, 0.62),    # champagne marble (floor medallion)
    'black_marble': (0.06, 0.06, 0.07),
    'teal': (0.01, 0.13, 0.15),         # deep, dark teal fountain water (see-through)
    'sand': (0.93, 0.88, 0.78),
    'terracotta': (0.55, 0.30, 0.17),
    'leaf': (0.07, 0.24, 0.09),         # dark boxwood terrace hedges
    'dark_wood': (0.20, 0.12, 0.07),    # skirting at the base of the walls
    'steel': (0.55, 0.58, 0.62),
    'linen': (0.95, 0.93, 0.87),        # ivory linen cloth (regular tables)
    'vip_cloth': (0.42, 0.04, 0.10),    # burgundy cloth (VIP tables, red carpet)
    'champagne_velvet': (0.78, 0.68, 0.50),  # regular chairs
    'burgundy_velvet': (0.36, 0.04, 0.09),   # VIP chairs
    'porcelain': (0.98, 0.98, 0.97),    # dinner plates, napkins, candles
    'crystal': (0.90, 0.92, 0.96),      # chandelier crystals, glassware
    'rose_white': (0.97, 0.95, 0.91),
    'blush': (0.93, 0.70, 0.72),
    'gold': (0.80, 0.63, 0.21),         # trim, chair frames, door frames, columns
    'dock': (0.10, 0.65, 0.30),
    'floor': (0.90, 0.83, 0.69),        # champagne floor
    'terrace_floor': (0.84, 0.76, 0.61),
    'glass': (0.85, 0.90, 0.92),        # vase glass
    'rose': (0.70, 0.05, 0.10),         # rose blooms / petals
}


def mat(c, emissive=None):
    r, g, b = C[c]
    e = f'<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>' if emissive else ''
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular>{e}</material>')


def point_light(name, pos, diffuse=(1.0, 0.8, 0.5)):
    """A warm point light with no shadows (evening mood, cheap on software rendering)."""
    x, y, z = pos
    r, g, b = diffuse
    return (f'<light name="{name}" type="point"><pose>{x} {y} {z} 0 0 0</pose>'
            f'<diffuse>{r} {g} {b} 1</diffuse><specular>0.2 0.2 0.2 1</specular>'
            f'<attenuation><range>9</range><constant>0.3</constant><linear>0.2</linear>'
            f'<quadratic>0.02</quadratic></attenuation><cast_shadows>false</cast_shadows></light>\n')


def geom(shape, size):
    if shape == 'box':
        return f'<box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box>'
    if shape == 'cyl':
        return f'<cylinder><radius>{size[0]:.3f}</radius><length>{size[1]:.3f}</length></cylinder>'
    return f'<sphere><radius>{size[0]:.3f}</radius></sphere>'


class Part(str):
    """The XML for one visual(+collision) element. Behaves like a plain string
    everywhere it's used (str.join(), etc.), but also carries its raw shape/size/
    pose so model() can log collision shapes for the navigability checker."""
    def __new__(cls, xml, shape=None, size=None, pose=None, collide=False):
        obj = str.__new__(cls, xml)
        obj.shape, obj.size, obj.pose, obj.collide = shape, size, pose, collide
        return obj


# Filled by model(): (shape, size, world_pose) for every collision=True part built.
# Read by scripts/check_dar_world_navigability.py via get_collisions().
COLLISIONS = []


def part(name, shape, size, pose, colour, collide=True, emissive=None, rp=(0, 0), transparency=0,
         visible=True):
    """One visual (+ collision) element inside a link. pose = (x, y, z, yaw).
    rp = (roll, pitch) tilts the part; only allowed on visual-only parts, since
    the navigability checker assumes collision shapes are upright.
    visible=False writes only the collision (an invisible solid block)."""
    x, y, z, yaw = pose
    assert rp == (0, 0) or not collide, f'{name}: tilted parts must be visual only'
    assert visible or collide, f'{name}: a part must be visible or solid'
    p = f'<pose>{x:.3f} {y:.3f} {z:.3f} {rp[0]:.4f} {rp[1]:.4f} {yaw:.4f}</pose>'
    g = geom(shape, size)
    t = f'<transparency>{transparency}</transparency>' if transparency else ''
    s = f'<visual name="{name}_v">{p}<geometry>{g}</geometry>{t}{mat(colour, emissive)}</visual>' if visible else ''
    if collide:
        s += f'<collision name="{name}_c">{p}<geometry>{g}</geometry></collision>'
    return Part(s, shape, size, pose, collide)


def model(name, parts, pose=(0, 0, 0, 0)):
    x, y, z, yaw = pose
    for pt in parts:
        if getattr(pt, 'collide', False):
            lx, ly, lz, lyaw = pt.pose
            wx = x + lx * math.cos(yaw) - ly * math.sin(yaw)
            wy = y + lx * math.sin(yaw) + ly * math.cos(yaw)
            COLLISIONS.append((pt.shape, pt.size, (wx, wy, z + lz, yaw + lyaw)))
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 {yaw}</pose>'
            f'<link name="link">{"".join(parts)}</link></model>\n')


def build():
    COLLISIONS.clear()
    out = []
    # ---------- floors (visual only) ----------
    fl = [part('floor', 'box', (22, 16, 0.01), (0, 0, 0.005, 0), 'floor', collide=False)]
    tx0, ty0, tx1, ty1 = TERRACE
    fl.append(part('terrace', 'box', (tx1 - tx0, ty1 - ty0, 0.01),
                   ((tx0 + tx1) / 2, (ty0 + ty1) / 2, 0.007, 0), 'terrace_floor', collide=False))
    # Courtyard marble inlay. Each layer is 2 mm thick and sits 2 mm above the
    # one below it, so overlapping layers never share a plane (no flicker).
    x0, y0, x1, y1 = COURTYARD
    fx, fy = FOUNTAIN
    cxm, cym, w, d = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    z = [0.011 + 0.002 * i for i in range(9)]   # z[0] = lowest inlay layer centre
    fl.append(part('court_ivory', 'box', (w, d, 0.002), (cxm, cym, z[0], 0), 'ivory', collide=False))
    band = 0.25                                  # black-marble border band
    for n, (bx, by, bsx, bsy) in enumerate([(cxm, y1 - band / 2, w, band), (cxm, y0 + band / 2, w, band),
                                            (x0 + band / 2, cym, band, d), (x1 - band / 2, cym, band, d)]):
        fl.append(part(f'court_border{n}', 'box', (bsx, bsy, 0.002), (bx, by, z[1], 0), 'black_marble', collide=False))
    # thin gold lines from the medallion out to the border on all 4 sides
    for n, (ex, ey) in enumerate([(x1 - band, fy), (x0 + band, fy), (fx, y1 - band), (fx, y0 + band)]):
        L = math.hypot(ex - fx, ey - fy)
        fl.append(part(f'court_ray{n}', 'box', (L, 0.04, 0.002), ((fx + ex) / 2, (fy + ey) / 2, z[1], math.atan2(ey - fy, ex - fx)),
                       'gold', collide=False))
    # medallion: concentric discs, largest at the bottom
    for n, (r, col) in enumerate([(2.30, 'gold'), (2.26, 'black_marble'), (2.02, 'gold'), (1.99, 'champagne')]):
        fl.append(part(f'medallion{n}', 'cyl', (r, 0.002), (fx, fy, z[2 + n], 0), col, collide=False))
    # 8-point star = two squares rotated 45 deg (gold outline under black marble)
    for n, (side, col) in enumerate([(2.80, 'gold'), (2.70, 'black_marble')]):
        for k, yaw in enumerate((0, math.pi / 4)):
            fl.append(part(f'star{n}_{k}', 'box', (side, side, 0.002), (fx, fy, z[6 + n], yaw), col, collide=False))
    fl.append(part('medallion_gold_ring', 'cyl', (1.53, 0.002), (fx, fy, z[8], 0), 'gold', collide=False))
    fl.append(part('medallion_centre', 'cyl', (1.50, 0.002), (fx, fy, z[8] + 0.002, 0), 'ivory', collide=False))
    # red carpet with gold edges, entrance (0,-8) to courtyard (0,-2), 1.4 m wide
    cy0, cy1, cw = -8.0, -2.0, 1.4
    fl.append(part('carpet', 'box', (cw, cy1 - cy0, 0.02), (0, (cy0 + cy1) / 2, 0.02, 0), 'vip_cloth', collide=False))
    for side in (-1, 1):
        fl.append(part(f'carpet_edge_{side}', 'box', (0.08, cy1 - cy0, 0.02),
                       (side * (cw / 2 - 0.04), (cy0 + cy1) / 2, 0.031, 0), 'gold', collide=False))
    out.append(model('floors', fl))

    # ---------- walls: cream, with dark-wood skirting, a gold rail at 1 m, ----------
    # ---------- and gold crown moulding near the ceiling ----------
    wp = []
    for n, (x1, y1, x2, y2) in enumerate(WALLS):
        L = math.hypot(x2 - x1, y2 - y1)
        yaw = math.atan2(y2 - y1, x2 - x1)
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        wp.append(part(f'wall{n}', 'box', (L + WALL_T, WALL_T, WALL_H), (cx, cy, WALL_H / 2, yaw), 'white'))
        wp.append(part(f'skirting{n}', 'box', (L + WALL_T + 0.01, WALL_T + 0.03, 0.15),
                       (cx, cy, 0.075, yaw), 'dark_wood', collide=False))
        wp.append(part(f'rail{n}', 'box', (L + WALL_T + 0.01, WALL_T + 0.02, 0.10),
                       (cx, cy, 1.0, yaw), 'gold', collide=False))
        wp.append(part(f'crown{n}', 'box', (L + WALL_T + 0.01, WALL_T + 0.03, 0.12),
                       (cx, cy, WALL_H - 0.10, yaw), 'gold', collide=False))
    out.append(model('dar_walls', wp))

    # ---------- doors: gold frames + lintel (lintel starts at 2.2 m, above the robot) ----------
    for name, x, y, w, axis in DOORS:
        yaw = 0.0 if axis == 'x' else math.pi / 2
        dx, dy = (w / 2 * math.cos(yaw), w / 2 * math.sin(yaw))
        ps = [part('post_a', 'box', (0.12, WALL_T + 0.06, 2.2), (-w / 2 - 0.06, 0, 1.1, 0), 'gold', collide=False),
              part('post_b', 'box', (0.12, WALL_T + 0.06, 2.2), (w / 2 + 0.06, 0, 1.1, 0), 'gold', collide=False),
              part('lintel', 'box', (w + 0.24, WALL_T, WALL_H - 2.2), (0, 0, 2.2 + (WALL_H - 2.2) / 2, 0), 'white')]
        if name == 'salon_arch':  # gold medallion standing on the wall above the VIP door.
            # part() only writes a yaw rotation; this needs roll=pi/2 (cylinder axis
            # tipped from vertical to horizontal) so it faces out of the wall instead
            # of lying flat like a disc, so its XML is built directly here.
            ps.append(
                f'<visual name="medallion_v"><pose>0 0 2.6 {math.pi / 2:.4f} 0 0</pose>'
                f'<geometry>{geom("cyl", (0.28, 0.05))}</geometry>{mat("gold")}</visual>')
        out.append(model(name, ps, (x, y, 0, yaw)))

    # ---------- grand 3-tier fountain: ivory marble, gold trim, black-marble step ----------
    # Solid parts all stay inside radius 1.0 m. The step ring is wider but flat,
    # visual only, and inside the 1.6 m no-go circle.
    def octagon(name, side, h, zc, colour, collide=False):
        """Two squares rotated 45 deg = an octagon (circumradius side/sqrt(2))."""
        return [part(f'{name}_{k}', 'box', (side, side, h), (0, 0, zc, k * math.pi / 4), colour, collide)
                for k in (0, 1)]

    def bowl(name, r, z_top, depth, stem_r):
        """Ivory bowl with a tapered underside, gold trim and teal water on top."""
        return [
            part(f'{name}_under', 'cyl', (stem_r, depth), (0, 0, z_top - 1.5 * depth, 0), 'ivory', collide=False),
            part(f'{name}', 'cyl', (r, depth), (0, 0, z_top - depth / 2, 0), 'ivory'),
            part(f'{name}_trim', 'cyl', (r + 0.02, 0.03), (0, 0, z_top - 0.012, 0), 'gold', collide=False),
            part(f'{name}_water', 'cyl', (r - 0.05, 0.008), (0, 0, z_top + 0.007, 0), 'teal',
                 collide=False, emissive=(0.01, 0.05, 0.06), transparency=0.6),
        ]

    fx, fy = FOUNTAIN
    fp = []
    fp += octagon('step_trim', 2.06, 0.05, 0.055, 'gold')                     # r 1.46, 3-8 cm high
    fp += octagon('step', 2.00, 0.07, 0.065, 'black_marble')                  # r 1.41, 3-10 cm high
    fp += octagon('basin', 1.36, 0.50, 0.25, 'ivory', collide=True)           # r 0.96, solid
    fp += octagon('basin_foot', 1.40, 0.04, 0.12, 'gold')                     # r 0.99, gold band low down
    fp += octagon('basin_rim', 1.40, 0.055, 0.4775, 'gold')                   # r 0.99, top 0.505
    fp += [
        # basin water: warm glow spots on the ivory basin floor, under a see-through deep teal surface
        part('water', 'cyl', (0.84, 0.008), (0, 0, 0.516, 0), 'teal', collide=False,
             emissive=(0.01, 0.05, 0.06), transparency=0.6),
        # tier 1 pedestal + middle bowl
        part('pedestal1', 'cyl', (0.17, 0.50), (0, 0, 0.75, 0), 'ivory'),
        part('pedestal1_collar', 'cyl', (0.20, 0.03), (0, 0, 0.53, 0), 'gold', collide=False),
        # tier 2 pedestal + upper bowl
        part('pedestal2', 'cyl', (0.11, 0.40), (0, 0, 1.26, 0), 'ivory'),
        part('pedestal2_collar', 'cyl', (0.13, 0.03), (0, 0, 1.09, 0), 'gold', collide=False),
        # tier 3 pedestal + crown bowl
        part('pedestal3', 'cyl', (0.07, 0.28), (0, 0, 1.67, 0), 'ivory'),
        part('pedestal3_collar', 'cyl', (0.09, 0.025), (0, 0, 1.55, 0), 'gold', collide=False),
        # gold finial
        part('finial_stem', 'cyl', (0.03, 0.10), (0, 0, 1.90, 0), 'gold', collide=False),
        part('finial_ball', 'sphere', (0.06,), (0, 0, 1.99, 0), 'gold', collide=False),
        part('finial_spike', 'cyl', (0.015, 0.10), (0, 0, 2.08, 0), 'gold', collide=False),
        part('finial_tip', 'sphere', (0.03,), (0, 0, 2.14, 0), 'gold', collide=False),
    ]
    fp += bowl('bowl1', 0.58, 1.06, 0.08, 0.32)
    fp += bowl('bowl2', 0.38, 1.535, 0.07, 0.22)
    fp += bowl('bowl3', 0.20, 1.845, 0.05, 0.12)
    # warm underwater glow (emissive only, no real light) under the see-through water
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        fp.append(part(f'glow{k}', 'cyl', (0.10, 0.002), (0.45 * math.cos(a), 0.45 * math.sin(a), 0.510, 0),
                       'gold', collide=False, emissive=(0.55, 0.38, 0.18)))
    # thin glowing water: jets rising from the basin + streams falling from the middle bowl
    for k in range(FOUNTAIN_JETS):
        a = 2 * math.pi * k / FOUNTAIN_JETS
        tilt, L, r0 = 0.25, 0.35, 0.74        # jets lean slightly inward
        r_mid, z_mid = r0 - math.sin(tilt) * L / 2, 0.52 + math.cos(tilt) * L / 2
        fp.append(part(f'jet{k}', 'cyl', (0.008, L), (r_mid * math.cos(a), r_mid * math.sin(a), z_mid, a),
                       'teal', collide=False, emissive=(0.12, 0.26, 0.27), rp=(0, -tilt), transparency=0.6))
        b = a + math.pi / FOUNTAIN_JETS
        fp.append(part(f'fall{k}', 'cyl', (0.006, 0.52), (0.61 * math.cos(b), 0.61 * math.sin(b), 0.78, 0),
                       'teal', collide=False, emissive=(0.12, 0.26, 0.27), transparency=0.65))
    out.append(model('fountain', fp, (fx, fy, 0, 0)))

    # ---------- arcade columns ----------
    for n, (x, y) in enumerate(COLUMNS):
        out.append(model(f'column_{n}', [
            part('base', 'cyl', (0.26, 0.15), (0, 0, 0.075, 0), 'gold'),
            part('shaft', 'cyl', (0.2, WALL_H), (0, 0, WALL_H / 2, 0), 'ivory'),
            part('capital', 'box', (0.5, 0.5, 0.15), (0, 0, WALL_H - 0.3, 0), 'gold', collide=False),
        ], (x, y, 0, 0)))

    # ---------- fine-dining round tables ----------
    # Solid parts (unchanged): cloth cylinder r=0.55 to the floor, and a 0.40 x 0.40
    # seat box from the floor at radius 0.82 (now invisible). Everything else is visual.
    TOP = 0.77                                    # height of the cloth-covered table top
    for name, (x, y) in TABLES.items():
        is_vip = name.startswith('vip')
        cloth = 'vip_cloth' if is_vip else 'linen'
        velvet = 'burgundy_velvet' if is_vip else 'champagne_velvet'
        ps = [part('cloth', 'cyl', (0.55, 0.74), (0, 0, 0.37, 0), cloth),
              part('drape', 'cyl', (0.58, 0.04), (0, 0, TOP - 0.02, 0), cloth, collide=False)]
        if is_vip:  # gold trim at the hem and along the top edge
            ps.append(part('hem_trim', 'cyl', (0.557, 0.04), (0, 0, 0.03, 0), 'gold', collide=False))
            ps.append(part('edge_trim', 'cyl', (0.585, 0.012), (0, 0, TOP - 0.035, 0), 'gold', collide=False))

        # centrepiece: low gold bowl of white/blush roses around a glass candle holder
        ps += [part('bowl', 'cyl', (0.12, 0.05), (0, 0, TOP + 0.025, 0), 'gold', collide=False),
               part('bowl_foot', 'cyl', (0.06, 0.02), (0, 0, TOP + 0.005, 0), 'gold', collide=False),
               part('candle', 'cyl', (0.022, 0.10), (0, 0, TOP + 0.10, 0), 'porcelain', collide=False),
               part('flame', 'sphere', (0.014,), (0, 0, TOP + 0.165, 0), 'gold',
                    collide=False, emissive=(1.0, 0.60, 0.20)),
               part('hurricane', 'cyl', (0.045, 0.24), (0, 0, TOP + 0.17, 0), 'crystal',
                    collide=False, transparency=0.75)]
        for r in range(CENTREPIECE_ROSES):
            ang = 2 * math.pi * r / max(CENTREPIECE_ROSES, 1)
            ps.append(part(f'rose{r}', 'sphere', (0.035,), (0.085 * math.cos(ang), 0.085 * math.sin(ang), TOP + 0.065, 0),
                           'rose_white' if r % 2 else 'blush', collide=False))

        def at(ang, radial, side, z):
            """Point in the table frame: 'radial' outward along ang, 'side' to the diner's right (+) or left (-)."""
            return (radial * math.cos(ang) - side * math.sin(ang), radial * math.sin(ang) + side * math.cos(ang), z, ang)

        for a in range(4):
            ang = math.pi / 4 + a * math.pi / 2
            # --- chair: invisible solid seat block + upholstered high-back chair facing the table ---
            sx, sy = 0.82 * math.cos(ang), 0.82 * math.sin(ang)
            ps.append(part(f'seat{a}', 'box', (0.40, 0.40, 0.45), (sx, sy, 0.225, 0), velvet, visible=False))
            ps += [part(f'seat_frame{a}', 'box', (0.37, 0.37, 0.03), at(ang, 0.82, 0, 0.43), 'gold', collide=False),
                   part(f'cushion{a}', 'box', (0.36, 0.36, 0.07), at(ang, 0.82, 0, 0.48), velvet, collide=False),
                   part(f'back_frame{a}', 'box', (0.025, 0.40, 0.62), at(ang, 1.0, 0, 0.78), 'gold', collide=False),
                   part(f'back{a}', 'box', (0.05, 0.36, 0.58), at(ang, 0.985, 0, 0.78), velvet, collide=False),
                   part(f'back_top{a}', 'cyl', (0.02, 0.40), at(ang, 1.0, 0, 1.09), 'gold', collide=False,
                        rp=(math.pi / 2, 0))]
            if CHAIR_LEGS:  # legs stay inside the solid seat footprint (<= 0.184 m from its centre)
                for k, (lr, ls) in enumerate([(-0.13, -0.13), (-0.13, 0.13), (0.13, -0.13), (0.13, 0.13)]):
                    ps.append(part(f'leg{a}_{k}', 'cyl', (0.012, 0.42), at(ang, 0.82 + lr, ls, 0.21), 'gold', collide=False))
            # --- place setting in front of the chair ---
            if PLACE_SETTINGS:
                ps += [part(f'charger{a}', 'cyl', (0.15, 0.006), at(ang, 0.38, 0, TOP + 0.003), 'gold', collide=False),
                       part(f'plate{a}', 'cyl', (0.12, 0.010), at(ang, 0.38, 0, TOP + 0.011), 'porcelain', collide=False),
                       part(f'napkin{a}', 'box', (0.05, 0.11, 0.035), at(ang, 0.38, 0, TOP + 0.033), 'porcelain',
                            collide=False, rp=(0.35, 0)),
                       part(f'fork{a}', 'box', (0.17, 0.014, 0.004), at(ang, 0.38, -0.18, TOP + 0.002), 'gold', collide=False),
                       part(f'knife{a}', 'box', (0.18, 0.012, 0.004), at(ang, 0.38, 0.18, TOP + 0.002), 'gold', collide=False),
                       # wine glass (stem + see-through bowl) and water tumbler, top right of the plate
                       part(f'wine_stem{a}', 'cyl', (0.004, 0.10), at(ang, 0.23, 0.10, TOP + 0.05), 'crystal', collide=False),
                       part(f'wine_bowl{a}', 'cyl', (0.035, 0.07), at(ang, 0.23, 0.10, TOP + 0.135), 'crystal',
                            collide=False, transparency=0.6),
                       part(f'water{a}', 'cyl', (0.03, 0.10), at(ang, 0.26, 0.18, TOP + 0.05), 'crystal',
                            collide=False, transparency=0.6)]

        # two-tier crystal chandelier on a gold rod from the ceiling
        ps += [part('chand_rod', 'cyl', (0.01, 0.50), (0, 0, WALL_H - 0.25, 0), 'gold', collide=False),
               part('chand_ring', 'cyl', (0.32, 0.03), (0, 0, 2.68, 0), 'gold', collide=False),
               part('chand_ring_top', 'cyl', (0.18, 0.025), (0, 0, 2.85, 0), 'gold', collide=False),
               part('chand_drop', 'sphere', (0.05,), (0, 0, 2.52, 0), 'crystal', collide=False,
                    emissive=(0.55, 0.52, 0.45))]
        for tier, (n, rr, zz, size) in enumerate([(CHAND_CRYSTALS_OUTER, 0.30, 2.60, 0.035),
                                                   (CHAND_CRYSTALS_INNER, 0.16, 2.77, 0.03)]):
            for r in range(n):
                ang = 2 * math.pi * r / n + tier * math.pi / max(n, 1)
                ps.append(part(f'crystal{tier}_{r}', 'sphere', (size,),
                               (rr * math.cos(ang), rr * math.sin(ang), zz, 0), 'crystal',
                               collide=False, emissive=(0.55, 0.52, 0.45)))
        out.append(model(name, ps, (x, y, 0, 0)))

    # ---------- counters, champagne bar, rose-hedge planters ----------
    for name, (cx, cy, sx, sy, h, col) in BOXES.items():
        ps = [part('body', 'box', (sx, sy, h), (0, 0, h / 2, 0), col)]
        if name.startswith('planter'):
            ps.append(part('hedge', 'box', (sx - 0.1, sy - 0.1, 0.25), (0, 0, h + 0.12, 0), 'leaf', collide=False))
            # two staggered rows of roses along the hedge top: mostly red, some blush
            n = max(2, int((sx - 0.4) / HEDGE_ROSE_SPACING) + 1)
            for row, by in enumerate((-0.07, 0.07)):
                for r in range(n):
                    bx = -sx / 2 + 0.2 + (r + 0.5 * row) * (sx - 0.4 - 0.5 * HEDGE_ROSE_SPACING) / (n - 1)
                    ps.append(part(f'bloom{row}_{r}', 'sphere', (0.045,), (bx, by, h + 0.25, 0),
                                   'blush' if (r + row) % 3 == 0 else 'rose', collide=False))
        if name == 'mint_tea_bar':
            ps.append(part('top', 'box', (sx + 0.06, sy + 0.06, 0.05), (0, 0, h + 0.025, 0), 'gold', collide=False))
            for r in range(5):
                gy = -sy / 2 + 0.5 + r * (sy - 1.0) / 4
                ps.append(part(f'glass{r}', 'cyl', (0.04, 0.14), (0, gy, h + 0.12, 0), 'glass', collide=False))
        out.append(model(name, ps, (cx, cy, 0, 0)))

    # ---------- charging dock ----------
    dx, dy = DOCK
    out.append(model('charging_dock', [
        part('body', 'box', (0.2, 0.7, 0.4), (0, 0, 0.2, 0), 'dock'),
        part('light', 'box', (0.05, 0.4, 0.08), (0.11, 0, 0.3, 0), 'dock', collide=False, emissive=(0.1, 0.9, 0.3)),
    ], (dx, dy, 0, 0)))

    # ---------- fairy lights strung across the courtyard, ~2.7 m with a slight sag ----------
    if FAIRY_LIGHTS:
        fairy_parts = []
        x0, _, x1, _ = COURTYARD
        n = FAIRY_BULBS_PER_ROW
        for ri, row_y in enumerate(FAIRY_ROWS_Y):
            for k in range(n):
                t = k / (n - 1) if n > 1 else 0.5       # 0..1 along the string
                sag = 0.15 * (1 - (2 * t - 1) ** 2)
                fairy_parts.append(part(f'bulb_{ri}_{k}', 'sphere', (0.025,),
                                         (x0 + t * (x1 - x0), row_y, 2.7 - sag, 0), 'ivory',
                                         collide=False, emissive=FAIRY_GLOW))
        out.append(model('fairy_lights', fairy_parts))

    # ---------- evening mood: 3 warm point lights, no shadows ----------
    out.append(point_light('courtyard_light', (0.0, 1.0, 2.6)))
    out.append(point_light('salon_light', (9.0, 5.5, 2.6)))
    out.append(point_light('terrace_light', (0.0, -6.0, 2.6)))
    return ''.join(out)


def get_collisions():
    """Run the layout builder and return the (shape, size, world_pose) list of
    every collision shape it produced. Used by check_dar_world_navigability.py
    so the checker always matches the current layout, without duplicating it."""
    build()
    return list(COLLISIONS)


HEADER = """<?xml version="1.0" ?>
<!-- GENERATED by scripts/make_dar_world.py - edit the script, not this file. -->
<sdf version="1.9">
  <world name="dar_tunibot">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.30 0.27 0.30 1</ambient>
      <background>0.04 0.06 0.14 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.45 0.38 0.32 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><ambient>0.55 0.6 0.5 1</ambient><diffuse>0.55 0.6 0.5 1</diffuse></material></visual>
      </link>
    </model>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 -16 17 0 0.85 1.5708</pose>
      </camera>
    </gui>
"""
FOOTER = "  </world>\n</sdf>\n"

if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, '..', 'worlds', 'dar_tunibot.world')
    with open(path, 'w') as f:
        f.write(HEADER + build() + FOOTER)
    print('Wrote', os.path.normpath(path))
