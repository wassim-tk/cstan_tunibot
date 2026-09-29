#!/usr/bin/env python3
"""Paint the ORIGINAL artworks hung on the walls of Dar TuniBot.

Every picture is drawn procedurally from simple shapes (gradients, polygons,
ellipses, random brush dabs) with a fixed random seed, so running this again
gives exactly the same PNGs. Nothing is copied from existing paintings.

Output: ros2_ws/src/cstam_gazebo/models/dar_art/materials/textures/<name>.png
The world generator (make_dar_world.py) hangs them with
    <albedo_map>model://dar_art/materials/textures/<name>.png</albedo_map>

Needs Pillow. It is installed in the project venv, so run:
    venv/bin/python ros2_ws/src/cstam_gazebo/scripts/make_dar_art.py
It prints the average colour of each picture; make_dar_world.py uses those as
plain canvas colours when PAINTING_TEXTURES = False.
"""
import math
import os
import random

from PIL import Image, ImageChops, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, '..', 'models', 'dar_art', 'materials', 'textures')
LAND = (768, 576)     # 4:3 landscape canvas (1.2 x 0.9 m in the world)
PORT = (512, 640)     # 4:5 portrait canvas  (0.8 x 1.0 m in the world)


# ============================== HELPERS ==============================
def lerp(a, b, t):
    return tuple(int(round(a[i] + (b[i] - a[i]) * t)) for i in range(3))


def gradient(img, box, stops):
    """Vertical gradient inside box=(x0, y0, x1, y1). stops = [(t, colour), ...] with t in 0..1."""
    x0, y0, x1, y1 = box
    d = ImageDraw.Draw(img)
    for y in range(y0, y1):
        t = (y - y0) / max(1, y1 - y0 - 1)
        for (ta, ca), (tb, cb) in zip(stops, stops[1:]):
            if ta <= t <= tb:
                d.line([(x0, y), (x1 - 1, y)], fill=lerp(ca, cb, (t - ta) / max(tb - ta, 1e-6)))
                break


def glow(img, centre, radius, colour, blur):
    """Soft additive glow (sun, moon, candle light)."""
    layer = Image.new('RGB', img.size, (0, 0, 0))
    cx, cy = centre
    ImageDraw.Draw(layer).ellipse([cx - radius, cy - radius, cx + radius, cy + radius], fill=colour)
    return ImageChops.add(img, layer.filter(ImageFilter.GaussianBlur(blur)))


def dabs(img, rng, box, colours, n, size, alpha=90, horizontal=False):
    """Scatter n semi-transparent brush dabs of the given colours inside box."""
    over = Image.new('RGBA', img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(over)
    x0, y0, x1, y1 = box
    for _ in range(n):
        x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
        w = rng.uniform(0.5, 1.0) * size
        h = w * (0.25 if horizontal else rng.uniform(0.5, 1.0))
        c = rng.choice(colours)
        d.ellipse([x - w, y - h, x + w, y + h], fill=c + (alpha,))
    img.paste(Image.alpha_composite(img.convert('RGBA'), over).convert('RGB'))


def canvas_finish(img, rng, strength=10):
    """Soften slightly and add a fine canvas-weave grain so it reads as paint, not vector art."""
    img = img.filter(ImageFilter.GaussianBlur(0.8))
    w, h = img.size
    grain = Image.effect_noise((w, h), 40).convert('RGB')
    weave = Image.new('RGB', (w, h))
    wd = ImageDraw.Draw(weave)
    for x in range(0, w, 3):
        wd.line([(x, 0), (x, h)], fill=(12, 12, 12))
    for y in range(0, h, 3):
        wd.line([(0, y), (w, y)], fill=(12, 12, 12))
    img = Image.blend(img, ImageChops.multiply(img, ImageChops.invert(weave)), 0.5)
    return ImageChops.add(Image.blend(img, grain, strength / 255), Image.new('RGB', (w, h), (0, 0, 0)))


def rot(points, centre, ang):
    cx, cy = centre
    c, s = math.cos(ang), math.sin(ang)
    return [(cx + (x - cx) * c - (y - cy) * s, cy + (x - cx) * s + (y - cy) * c) for x, y in points]


def leaf(d, centre, length, width, ang, colour):
    cx, cy = centre
    pts = []
    for k in range(13):
        t = k / 12
        pts.append((cx + t * length, cy - math.sin(t * math.pi) * width))
    for k in range(12, -1, -1):
        t = k / 12
        pts.append((cx + t * length, cy + math.sin(t * math.pi) * width * 0.7))
    d.polygon(rot(pts, centre, ang), fill=colour)


def rose(d, rng, centre, r, dark, mid, light):
    """A rose bloom seen from above: rings of petals spiralling inwards."""
    cx, cy = centre
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=dark)
    rings = 5
    for k in range(rings):
        rr = r * (1 - k / rings)
        n = 5 + k
        off = rng.uniform(0, 2 * math.pi)
        for p in range(n):
            a = off + 2 * math.pi * p / n
            px, py = cx + 0.35 * rr * math.cos(a), cy + 0.35 * rr * math.sin(a)
            pr = rr * 0.62
            d.ellipse([px - pr, py - pr * 0.8, px + pr, py + pr * 0.8],
                      fill=lerp(mid, light, k / rings) if p % 2 else lerp(dark, mid, k / rings))
        d.arc([cx - rr * 0.8, cy - rr * 0.8, cx + rr * 0.8, cy + rr * 0.8],
              start=off * 57.3, end=off * 57.3 + 200, fill=dark, width=max(1, int(r / 12)))
    d.ellipse([cx - r * 0.12, cy - r * 0.12, cx + r * 0.12, cy + r * 0.12], fill=dark)


# ============================== THE ARTWORKS ==============================
def sidi_bou_said_sunset(rng):
    """Whitewashed cliff-top village with blue doors against a violet-orange sunset over the gulf."""
    w, h = LAND
    img = Image.new('RGB', LAND)
    hz = int(h * 0.58)
    gradient(img, (0, 0, w, hz), [(0, (48, 32, 92)), (0.45, (148, 62, 118)),
                                  (0.8, (236, 118, 82)), (1, (252, 186, 112))])
    gradient(img, (0, hz, w, h), [(0, (120, 70, 118)), (1, (28, 30, 72))])
    sun = (int(w * 0.66), hz - 38)
    img = glow(img, sun, 110, (120, 60, 20), 60)
    d = ImageDraw.Draw(img)
    d.ellipse([sun[0] - 42, sun[1] - 42, sun[0] + 42, sun[1] + 42], fill=(255, 222, 160))
    for k in range(40):  # sun path shimmering on the water
        y = hz + 4 + k * (h - hz) / 42
        half = 12 + k * 1.6 + rng.uniform(-6, 6)
        x = sun[0] + rng.uniform(-8, 8)
        d.line([(x - half, y), (x + half, y)], fill=lerp((255, 205, 130), (140, 80, 110), k / 40), width=2)
    dabs(img, rng, (0, 0, w, hz - 60), [(170, 80, 130), (210, 110, 110), (90, 50, 110)], 90, 40, 40, True)
    d = ImageDraw.Draw(img)
    # cliff + village on the left
    cliff = [(0, int(h * 0.30))]
    for k in range(1, 21):
        x = k * w * 0.5 / 20
        cliff.append((x, h * 0.30 + (k / 20) ** 2.2 * h * 0.52))
    cliff += [(w * 0.5, h), (0, h)]
    d.polygon(cliff, fill=(78, 48, 72))
    houses = []
    for k in range(14):
        t = rng.uniform(0.0, 0.85)
        x = t * w * 0.5
        base = h * 0.30 + t ** 2.2 * h * 0.52 + rng.uniform(4, 24)
        hw, hh = rng.uniform(26, 48), rng.uniform(26, 50)
        houses.append((base, x, hw, hh))
    for base, x, hw, hh in sorted(houses):
        wall = lerp((238, 214, 222), (206, 170, 196), rng.random())
        d.rectangle([x, base - hh, x + hw, base + 30], fill=wall)
        d.rectangle([x, base - hh, x + hw * 0.18, base + 30], fill=lerp(wall, (120, 80, 120), 0.35))
        dw = hw * 0.26
        d.rectangle([x + hw * 0.45, base - hh * 0.55, x + hw * 0.45 + dw, base + 2], fill=(28, 84, 168))
        d.ellipse([x + hw * 0.45, base - hh * 0.55 - dw / 2, x + hw * 0.45 + dw, base - hh * 0.55 + dw / 2],
                  fill=(28, 84, 168))
        if rng.random() < 0.3:
            d.pieslice([x + hw * 0.2, base - hh - hw * 0.35, x + hw * 0.8, base - hh + hw * 0.35], 180, 360,
                       fill=wall)
    mx, my = w * 0.16, h * 0.30 + 0.32 ** 2.2 * h * 0.52 - 40  # minaret
    d.rectangle([mx, my - 90, mx + 18, my + 20], fill=(236, 206, 214))
    d.rectangle([mx - 4, my - 94, mx + 22, my - 86], fill=(200, 160, 90))
    d.polygon([(mx - 2, my - 94), (mx + 9, my - 120), (mx + 20, my - 94)], fill=(28, 84, 168))
    return canvas_finish(img, rng)


def roses_still_life(rng):
    """Red and blush roses in a gold vase on a dark velvet ground, a few fallen petals."""
    w, h = PORT
    img = Image.new('RGB', PORT)
    gradient(img, (0, 0, w, h), [(0, (46, 8, 18)), (0.75, (22, 4, 10)), (0.76, (70, 36, 22)), (1, (40, 20, 12))])
    img = glow(img, (int(w * 0.35), int(h * 0.3)), 180, (40, 18, 14), 90)
    d = ImageDraw.Draw(img)
    vx, vy = w // 2, int(h * 0.62)
    stems = []
    for k in range(9):
        a = -math.pi / 2 + rng.uniform(-0.85, 0.85)
        L = rng.uniform(150, 250)
        stems.append((vx + L * math.cos(a), vy - 30 + L * math.sin(a)))
    for sx, sy in stems:
        d.line([(vx + rng.uniform(-15, 15), vy), (sx, sy)], fill=(38, 72, 30), width=4)
        for _ in range(2):
            t = rng.uniform(0.3, 0.7)
            px, py = vx + (sx - vx) * t, vy + (sy - vy) * t
            leaf(d, (px, py), rng.uniform(30, 45), rng.uniform(9, 13), rng.uniform(0, 2 * math.pi),
                 rng.choice([(40, 86, 36), (56, 100, 44), (30, 64, 30)]))
    for k, (sx, sy) in enumerate(sorted(stems, key=lambda p: p[1])):
        if k % 3 == 1:
            rose(d, rng, (sx, sy), rng.uniform(26, 34), (170, 96, 104), (226, 150, 158), (248, 206, 206))
        else:
            rose(d, rng, (sx, sy), rng.uniform(28, 38), (96, 6, 20), (170, 18, 40), (222, 70, 84))
    # gold vase
    vase = [(vx - 40, vy - 20), (vx + 40, vy - 20), (vx + 30, vy + 5), (vx + 62, vy + 70),
            (vx + 46, vy + 120), (vx - 46, vy + 120), (vx - 62, vy + 70), (vx - 30, vy + 5)]
    d.polygon(vase, fill=(176, 132, 46))
    d.polygon([(vx - 22, vy + 8), (vx - 12, vy + 8), (vx - 34, vy + 110), (vx - 44, vy + 110)], fill=(236, 204, 120))
    d.polygon([(vx + 30, vy + 5), (vx + 62, vy + 70), (vx + 46, vy + 120), (vx + 30, vy + 120)], fill=(130, 92, 30))
    d.ellipse([vx - 42, vy - 28, vx + 42, vy - 12], fill=(210, 168, 72))
    d.ellipse([vx - 30, vy - 24, vx + 30, vy - 15], fill=(40, 16, 10))
    for _ in range(4):  # fallen petals
        px, py = rng.uniform(w * 0.15, w * 0.85), rng.uniform(h * 0.83, h * 0.93)
        d.ellipse([px - 11, py - 5, px + 11, py + 5], fill=rng.choice([(160, 16, 36), (220, 140, 150)]))
    return canvas_finish(img, rng)


def mediterranean_seascape(rng):
    """Midday sea: turquoise shallows, deep blue open water, ochre rocks and a small white sail."""
    w, h = LAND
    img = Image.new('RGB', LAND)
    hz = int(h * 0.42)
    gradient(img, (0, 0, w, hz), [(0, (96, 150, 204)), (1, (220, 226, 222))])
    gradient(img, (0, hz, w, h), [(0, (32, 88, 148)), (0.55, (24, 110, 150)), (1, (40, 160, 160))])
    dabs(img, rng, (0, 10, w, hz - 30), [(250, 250, 250), (230, 236, 242)], 40, 50, 70, True)
    d = ImageDraw.Draw(img)
    for _ in range(420):  # wave strokes, longer and brighter towards the viewer
        y = rng.uniform(hz + 2, h)
        t = (y - hz) / (h - hz)
        x = rng.uniform(0, w)
        L = 6 + t * 40
        c = lerp((70, 130, 180), (190, 236, 232), rng.uniform(0.2, 1.0) * (0.4 + 0.6 * t))
        d.line([(x, y), (x + L, y + rng.uniform(-1, 1))], fill=c, width=1 + int(t * 3))
    # headland: a rounded, jagged coastline from the bottom edge up to the right edge
    coast = []
    for k in range(41):
        t = k / 40
        a = t * math.pi / 2
        coast.append((w - w * 0.40 * math.cos(a) + rng.uniform(-10, 10) * (1 - t),
                      h - h * 0.56 * math.sin(a) + rng.uniform(-12, 12) * t * (1 - t) * 4))
    mask = Image.new('L', LAND, 0)
    ImageDraw.Draw(mask).polygon(coast + [(w, h)], fill=255)
    rock = Image.new('RGB', LAND, (140, 98, 60))
    rd = ImageDraw.Draw(rock)
    for _ in range(160):
        x, y, r = rng.uniform(w * 0.55, w), rng.uniform(h * 0.4, h), rng.uniform(8, 30)
        rd.ellipse([x - r, y - r * 0.6, x + r, y + r * 0.6],
                   fill=rng.choice([(176, 128, 78), (120, 82, 50), (196, 150, 96), (98, 66, 44)]))
    img.paste(rock, mask=mask)
    d = ImageDraw.Draw(img)
    for x, y in coast:  # foam where the sea meets the rocks
        for _ in range(3):
            fx, fy, r = x + rng.uniform(-14, 4), y + rng.uniform(-6, 6), rng.uniform(3, 7)
            d.ellipse([fx - r, fy - r * 0.6, fx + r, fy + r * 0.6], fill=(236, 244, 240))
    bx, by = w * 0.28, hz + 30  # sailboat
    d.polygon([(bx - 26, by), (bx + 26, by), (bx + 18, by + 8), (bx - 18, by + 8)], fill=(120, 60, 40))
    d.polygon([(bx, by - 2), (bx, by - 60), (bx + 26, by - 4)], fill=(250, 248, 240))
    d.polygon([(bx - 2, by - 2), (bx - 2, by - 46), (bx - 22, by - 4)], fill=(232, 228, 216))
    for gx, gy in ((w * 0.45, h * 0.14), (w * 0.5, h * 0.2), (w * 0.4, h * 0.22)):
        d.line([(gx - 9, gy - 4), (gx, gy), (gx + 9, gy - 4)], fill=(60, 70, 90), width=2)
    return canvas_finish(img, rng)


def desert_dunes_night(rng):
    """Moonlit Saharan dunes (Douz) under a starry indigo sky, two date palms."""
    w, h = LAND
    img = Image.new('RGB', LAND)
    gradient(img, (0, 0, w, h), [(0, (8, 12, 40)), (0.6, (48, 44, 96)), (1, (96, 70, 110))])
    d = ImageDraw.Draw(img)
    for _ in range(260):
        x, y = rng.uniform(0, w), rng.uniform(0, h * 0.55)
        s = rng.choice([1, 1, 1, 2])
        c = lerp((150, 150, 190), (255, 250, 230), rng.random())
        d.ellipse([x - s / 2, y - s / 2, x + s / 2, y + s / 2], fill=c)
    mx, my = int(w * 0.76), int(h * 0.2)
    img = glow(img, (mx, my), 70, (60, 60, 50), 45)
    d = ImageDraw.Draw(img)
    crescent = Image.new('L', LAND, 0)
    cd = ImageDraw.Draw(crescent)
    cd.ellipse([mx - 34, my - 34, mx + 34, my + 34], fill=255)
    cd.ellipse([mx - 20, my - 40, mx + 44, my + 26], fill=0)
    img.paste((250, 242, 212), mask=crescent)
    layers = [(0.52, (150, 98, 70), (92, 56, 52)), (0.62, (184, 124, 76), (110, 66, 50)),
              (0.74, (210, 150, 90), (126, 76, 52)), (0.88, (226, 172, 104), (150, 92, 58))]
    for n, (base, lit, shade) in enumerate(layers):
        ph, amp, per = rng.uniform(0, 6.28), rng.uniform(22, 40), rng.uniform(1.2, 2.2)
        ridge = [(x, h * base - amp * math.sin(ph + per * math.pi * x / w) ** 2) for x in range(0, w + 8, 8)]
        d.polygon(ridge + [(w, h), (0, h)], fill=shade)
        # moonlit side of each dune: shift the ridge line right and down a little
        lit_poly = ridge + [(x + 30, y + 26) for x, y in reversed(ridge)]
        d.polygon(lit_poly, fill=lit)
        d.line(ridge, fill=lerp(lit, (255, 236, 200), 0.4), width=2)
    for px, py, s in ((w * 0.16, h * 0.63, 1.0), (w * 0.23, h * 0.66, 0.8)):  # palms
        d.line([(px, py), (px + 8 * s, py - 90 * s)], fill=(30, 22, 30), width=int(7 * s))
        tx, ty = px + 8 * s, py - 90 * s
        for a in range(8):
            ang = -math.pi + a * math.pi / 7 + rng.uniform(-0.1, 0.1)
            leaf(d, (tx, ty), 55 * s, 7 * s, ang + 0.25, (30, 22, 30))
    return canvas_finish(img, rng)


def zellige_arch(rng):
    """A horseshoe arch filled with an 8-point-star zellige tile pattern, set in a lime-plaster wall."""
    w, h = LAND
    img = Image.new('RGB', LAND, (232, 222, 200))
    dabs(img, rng, (0, 0, w, h), [(222, 208, 182), (240, 232, 214), (214, 198, 170)], 300, 40, 50)
    tiles = Image.new('RGB', LAND, (238, 234, 222))
    d = ImageDraw.Draw(tiles)
    cell = 64
    pal = [(28, 72, 150), (22, 112, 96), (196, 150, 58), (140, 36, 44)]
    for j in range(-1, h // cell + 2):
        for i in range(-1, w // cell + 2):
            cx, cy = i * cell + (cell / 2 if j % 2 else 0), j * cell
            col = pal[(i + 2 * j) % len(pal)]
            s = cell * 0.36
            sq = [(cx - s, cy - s), (cx + s, cy - s), (cx + s, cy + s), (cx - s, cy + s)]
            d.polygon(sq, fill=col, outline=(40, 34, 30))
            d.polygon(rot(sq, (cx, cy), math.pi / 4), fill=col, outline=(40, 34, 30))
            c = cell * 0.13
            d.ellipse([cx - c, cy - c, cx + c, cy + c], fill=(238, 234, 222), outline=(40, 34, 30))
            c2 = cell * 0.05
            d.ellipse([cx - c2, cy - c2, cx + c2, cy + c2], fill=col)
    # horseshoe-arch mask: a circle wider than the opening sitting on two jambs
    mask = Image.new('L', LAND, 0)
    md = ImageDraw.Draw(mask)
    ax, aw = w // 2, int(w * 0.36)
    top_c, r = int(h * 0.40), int(aw * 0.62)
    md.ellipse([ax - r, top_c - r, ax + r, top_c + r], fill=255)
    md.rectangle([ax - aw // 2, top_c, ax + aw // 2, h - 40], fill=255)
    md.rectangle([ax - r, top_c + int(r * 0.55), ax - aw // 2, h], fill=0)
    md.rectangle([ax + aw // 2, top_c + int(r * 0.55), ax + r, h], fill=0)
    # gold border just outside the arch
    border = mask.filter(ImageFilter.MaxFilter(13))
    img.paste((190, 146, 60), mask=border)
    img.paste(tiles, mask=mask)
    d = ImageDraw.Draw(img)
    d.rectangle([0, h - 40, w, h], fill=(170, 148, 118))
    d.line([(0, h - 40), (w, h - 40)], fill=(120, 100, 80), width=3)
    return canvas_finish(img, rng)


def gold_burgundy_bands(rng):
    """Abstract: soft-edged burgundy, rose and black bands with a low gold sun and gold-leaf flecks."""
    w, h = LAND
    img = Image.new('RGB', LAND)
    gradient(img, (0, 0, w, h), [(0, (92, 10, 30)), (0.35, (122, 20, 42)), (0.36, (28, 10, 14)),
                                 (0.52, (40, 12, 18)), (0.53, (164, 60, 70)), (0.78, (120, 24, 40)),
                                 (0.79, (190, 142, 60)), (1, (120, 80, 30))])
    img = img.filter(ImageFilter.GaussianBlur(14))
    dabs(img, rng, (0, 0, w, h), [(150, 30, 50), (70, 8, 22), (200, 90, 90)], 220, 34, 45, True)
    img = glow(img, (int(w * 0.32), int(h * 0.45)), 120, (90, 60, 10), 30)
    d = ImageDraw.Draw(img)
    d.ellipse([w * 0.32 - 92, h * 0.45 - 92, w * 0.32 + 92, h * 0.45 + 92], outline=(214, 170, 80), width=5)
    for _ in range(45):
        x, y, s = rng.uniform(0, w), rng.uniform(0, h), rng.uniform(1.5, 4)
        d.polygon(rot([(x - s, y - s / 2), (x + s, y - s / 2), (x + s, y + s / 2), (x - s, y + s / 2)],
                      (x, y), rng.uniform(0, 3)), fill=rng.choice([(226, 184, 92), (246, 214, 130), (180, 136, 56)]))
    d.line([(w * 0.58, 0), (w * 0.72, h)], fill=(214, 170, 80), width=3)
    return canvas_finish(img, rng)


def gold_burgundy_rings(rng):
    """Abstract portrait: gold rings drifting over a deep burgundy field, crossed by a black diagonal."""
    w, h = PORT
    img = Image.new('RGB', PORT)
    gradient(img, (0, 0, w, h), [(0, (64, 6, 22)), (1, (120, 18, 40))])
    dabs(img, rng, (0, 0, w, h), [(90, 8, 28), (140, 26, 50), (50, 4, 16)], 260, 36, 50)
    d = ImageDraw.Draw(img)
    d.polygon([(0, h * 0.72), (w, h * 0.28), (w, h * 0.36), (0, h * 0.80)], fill=(20, 8, 12))
    for cx, cy, r0, n in ((w * 0.38, h * 0.36, 30, 7), (w * 0.68, h * 0.70, 20, 5)):
        for k in range(n):
            r = r0 + k * 18
            c = lerp((250, 214, 130), (150, 104, 36), k / n)
            d.arc([cx - r, cy - r, cx + r, cy + r], start=rng.uniform(0, 90), end=rng.uniform(250, 340),
                  fill=c, width=4 if k % 2 else 2)
    img = glow(img, (int(w * 0.38), int(h * 0.36)), 26, (120, 90, 30), 14)
    return canvas_finish(img, rng)


def blue_door(rng):
    """A studded blue Tunisian door in a whitewashed wall, bougainvillea spilling over the top."""
    w, h = PORT
    img = Image.new('RGB', PORT, (238, 236, 228))
    dabs(img, rng, (0, 0, w, h), [(228, 226, 216), (246, 244, 238), (220, 214, 204)], 260, 34, 60)
    d = ImageDraw.Draw(img)
    dx0, dx1, dtop, dbot = w * 0.22, w * 0.78, h * 0.30, h * 0.90
    r = (dx1 - dx0) / 2
    d.rectangle([dx0 - 16, dtop, dx1 + 16, dbot], fill=(196, 170, 120))                     # stone frame
    d.pieslice([dx0 - 16, dtop - r - 16, dx1 + 16, dtop + r + 16], 180, 360, fill=(196, 170, 120))
    d.rectangle([dx0, dtop, dx1, dbot], fill=(22, 82, 170))
    d.pieslice([dx0, dtop - r, dx1, dtop + r], 180, 360, fill=(22, 82, 170))
    d.line([(w / 2, dtop - r), (w / 2, dbot)], fill=(14, 56, 120), width=3)                  # two leaves
    stud = lambda x, y: d.ellipse([x - 4, y - 4, x + 4, y + 4], fill=(24, 24, 30))
    for k in range(25):                                    # studs following the arch
        a = math.pi + k * math.pi / 24
        stud(w / 2 + (r - 18) * math.cos(a), dtop + (r - 18) * math.sin(a))
    for y in range(int(dtop), int(dbot - 20), 22):         # studs down both edges
        stud(dx0 + 18, y)
        stud(dx1 - 18, y)
    for cy in (h * 0.50, h * 0.72):                        # a studded rosette on each leaf
        for cx in (w * 0.36, w * 0.64):
            stud(cx, cy)
            for k in range(8):
                a = k * math.pi / 4
                stud(cx + 26 * math.cos(a), cy + 26 * math.sin(a))
    d.ellipse([w / 2 - 22, h * 0.60, w / 2 - 6, h * 0.60 + 16], outline=(40, 40, 46), width=3)  # knocker
    d.rectangle([dx0 - 30, dbot, dx1 + 30, dbot + 14], fill=(170, 150, 120))                  # step
    d.rectangle([0, dbot + 14, w, h], fill=(206, 196, 176))
    for _ in range(26):                                    # bougainvillea
        x = rng.uniform(0, w * 0.6) if rng.random() < 0.8 else rng.uniform(w * 0.6, w)
        y = rng.uniform(0, h * 0.24 + (1 - x / w) * h * 0.2)
        leaf(d, (x, y), rng.uniform(20, 30), rng.uniform(6, 9), rng.uniform(0, 6.28),
             rng.choice([(46, 100, 40), (60, 120, 50)]))
    for _ in range(170):
        x = rng.gauss(w * 0.25, w * 0.22)
        y = abs(rng.gauss(0, h * 0.14))
        s = rng.uniform(5, 10)
        d.ellipse([x - s, y - s, x + s, y + s], fill=rng.choice([(206, 30, 130), (230, 60, 150), (170, 20, 110)]))
    return canvas_finish(img, rng)


def olive_grove(rng):
    """Olive grove on rolling hills (Sahel) in golden late-afternoon light."""
    w, h = LAND
    img = Image.new('RGB', LAND)
    gradient(img, (0, 0, w, h), [(0, (246, 210, 142)), (0.45, (238, 176, 112)), (1, (238, 176, 112))])
    img = glow(img, (int(w * 0.8), int(h * 0.22)), 80, (60, 40, 10), 50)
    d = ImageDraw.Draw(img)
    hills = [(0.42, (176, 150, 96)), (0.52, (150, 142, 84)), (0.66, (170, 132, 80)), (0.82, (190, 140, 84))]
    ridges = []
    for base, col in hills:
        ph = rng.uniform(0, 6.28)
        ridge = [(x, h * base + 16 * math.sin(ph + x / w * 5) + 8 * math.sin(ph * 2 + x / w * 13))
                 for x in range(0, w + 8, 8)]
        d.polygon(ridge + [(w, h), (0, h)], fill=col)
        ridges.append(ridge)
    for n, ridge in enumerate(ridges[1:], 1):              # rows of trees, bigger in front
        s = 0.35 + 0.35 * n
        for x in range(int(rng.uniform(0, 60)), w, int(90 * s)):
            y = ridge[min(len(ridge) - 1, x // 8)][1] + 6 * s
            d.ellipse([x - 22 * s, y - 2 * s, x + 22 * s, y + 6 * s], fill=(110, 90, 60))  # shadow
            d.line([(x, y), (x - 4 * s, y - 20 * s), (x + 2 * s, y - 34 * s)], fill=(76, 56, 40), width=int(6 * s))
            for _ in range(int(16 * s) + 6):
                bx, by = x + rng.gauss(0, 16 * s), y - 44 * s + rng.gauss(0, 11 * s)
                r = rng.uniform(6, 11) * s
                d.ellipse([bx - r, by - r * 0.7, bx + r, by + r * 0.7],
                          fill=rng.choice([(104, 118, 80), (136, 146, 100), (84, 98, 66), (160, 164, 118)]))
    return canvas_finish(img, rng)


ARTWORKS = {
    'sidi_bou_said_sunset': sidi_bou_said_sunset,
    'roses_still_life': roses_still_life,
    'mediterranean_seascape': mediterranean_seascape,
    'desert_dunes_night': desert_dunes_night,
    'zellige_arch': zellige_arch,
    'gold_burgundy_bands': gold_burgundy_bands,
    'gold_burgundy_rings': gold_burgundy_rings,
    'blue_door': blue_door,
    'olive_grove': olive_grove,
}

if __name__ == '__main__':
    os.makedirs(OUT_DIR, exist_ok=True)
    for n, (name, paint) in enumerate(ARTWORKS.items()):
        img = paint(random.Random(1000 + n))
        path = os.path.join(OUT_DIR, name + '.png')
        img.save(path, optimize=True)
        avg = img.resize((1, 1), Image.Resampling.BOX).getpixel((0, 0))
        print(f"{name:24s} {img.size[0]}x{img.size[1]}  avg colour ({avg[0] / 255:.2f}, "
              f"{avg[1] / 255:.2f}, {avg[2] / 255:.2f})")
    print('Wrote', os.path.normpath(OUT_DIR))
