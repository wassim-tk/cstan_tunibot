#!/usr/bin/env python3
"""Check that the robot can reach every stop in Dar TuniBot without a real
Gazebo run.

How it works:
  1. It imports make_dar_world.py and calls get_collisions() to get the exact
     list of collision shapes the generator builds (so this check can never
     drift out of sync with the world file).
  2. It ignores any shape that starts above 1.3 m (the robot's height) -
     the robot can drive under door lintels, for example.
  3. It rasterises the remaining shapes onto a 5 cm grid, growing each one by
     the robot's radius + a safety margin (0.30 + 0.10 = 0.40 m), plus a
     no-go circle around the fountain.
  4. It flood-fills (BFS) from the start position and reports which named
     stops are reachable.
  5. It checks the robot can drive a full loop around the fountain (every
     point on a ring just outside the no-go circle must be reachable).
  6. It saves a PNG map of the result (no external libraries needed).

Run it with:
    python3 ros2_ws/src/cstam_gazebo/scripts/check_dar_world_navigability.py
"""
import math
import os
import struct
import sys
import zlib
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_dar_world as W

# ============================== SETTINGS ==============================
ROBOT_R = 0.30 + 0.10          # robot radius + safety margin
ROBOT_H = 1.3                  # robot height: ignore shapes starting above this
CELL = 0.05                    # grid resolution, metres
NO_GO_CENTER = W.FOUNTAIN
NO_GO_R = 1.6
LOOP_R = NO_GO_R + 0.35        # ring around the fountain the robot must be able to drive

START = (-9.0, -1.45)
STOPS = {
    'Dock': (-10.2, -1.45), 'Kitchen': (-8.5, 6.55),
    'T1': (-2.3, 4.95), 'T2': (2.3, 4.95), 'T3': (6.8, 1.0), 'T4': (6.8, -2.3),
    'T5': (-5.4, -6.3), 'T6': (-1.85, -6.3), 'T7': (1.85, -6.3), 'T8': (5.4, -6.3),
    'T9': (-6.1, 0.0), 'T10': (-6.1, -2.9),
    'VIP1': (7.8, 6.4), 'VIP2': (7.8, 4.2),
}

# Grid bounds: building is x -11..11, y -8..8; add a bit of margin.
X0, X1 = -11.5, 11.5
Y0, Y1 = -8.5, 8.5
NX = int(round((X1 - X0) / CELL))
NY = int(round((Y1 - Y0) / CELL))

OUT_PNG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dar_tunibot_nav_check.png')


def world_to_cell(x, y):
    return int((x - X0) / CELL), int((y - Y0) / CELL)


def cell_center(i, j):
    return X0 + (i + 0.5) * CELL, Y0 + (j + 0.5) * CELL


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def mark_circle(grid, cx, cy, r):
    if r <= 0:
        return
    i0, j0 = world_to_cell(cx - r, cy - r)
    i1, j1 = world_to_cell(cx + r, cy + r)
    for j in range(max(0, j0), min(NY, j1 + 1)):
        for i in range(max(0, i0), min(NX, i1 + 1)):
            px, py = cell_center(i, j)
            if (px - cx) ** 2 + (py - cy) ** 2 <= r * r:
                grid[j][i] = 1


def mark_rounded_box(grid, cx, cy, sx, sy, yaw, inflate):
    """A box footprint grown by `inflate` in every direction (Minkowski sum
    of a rotated rectangle with a disk = rounded rectangle)."""
    hx, hy = sx / 2, sy / 2
    reach = math.hypot(hx, hy) + inflate
    i0, j0 = world_to_cell(cx - reach, cy - reach)
    i1, j1 = world_to_cell(cx + reach, cy + reach)
    cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
    for j in range(max(0, j0), min(NY, j1 + 1)):
        for i in range(max(0, i0), min(NX, i1 + 1)):
            px, py = cell_center(i, j)
            dx, dy = px - cx, py - cy
            lx = dx * cos_y - dy * sin_y
            ly = dx * sin_y + dy * cos_y
            nx = lx - clamp(lx, -hx, hx)
            ny = ly - clamp(ly, -hy, hy)
            if nx * nx + ny * ny <= inflate * inflate:
                grid[j][i] = 1


def build_grid():
    grid = [[0] * NX for _ in range(NY)]
    collisions = W.get_collisions()
    used = 0
    for shape, size, pose in collisions:
        x, y, z, yaw = pose
        if shape == 'box':
            sx, sy, sz = size
            bottom = z - sz / 2
        elif shape == 'cyl':
            r, length = size
            bottom = z - length / 2
        else:  # sphere
            (r,) = size
            bottom = z - r
        if bottom >= ROBOT_H:
            continue  # robot can drive under this
        used += 1
        if shape == 'box':
            mark_rounded_box(grid, x, y, sx, sy, yaw, ROBOT_R)
        else:
            mark_circle(grid, x, y, r + ROBOT_R)
    print(f'{len(collisions)} collision shapes total, {used} below {ROBOT_H} m used as obstacles')

    # no-go circle around the fountain (already generous, not further inflated).
    # Tracked on its own grid so the map can show it in a different colour.
    nogo = [[0] * NX for _ in range(NY)]
    mark_circle(nogo, NO_GO_CENTER[0], NO_GO_CENTER[1], NO_GO_R)
    return grid, nogo


def bfs(grid, nogo, start_xy):
    def blocked(i, j):
        return grid[j][i] or nogo[j][i]

    si, sj = world_to_cell(*start_xy)
    if not (0 <= si < NX and 0 <= sj < NY) or blocked(si, sj):
        raise SystemExit(f'Start point {start_xy} is inside an obstacle or off the grid!')
    reached = [[False] * NX for _ in range(NY)]
    reached[sj][si] = True
    q = deque([(si, sj)])
    neigh = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]
    while q:
        i, j = q.popleft()
        for di, dj in neigh:
            ni, nj = i + di, j + dj
            if 0 <= ni < NX and 0 <= nj < NY and not reached[nj][ni] and not blocked(ni, nj):
                reached[nj][ni] = True
                q.append((ni, nj))
    return reached


def ring_points(r=None, n=720):
    # 720 samples on a ~2 m ring are ~1.7 cm apart, less than one 5 cm cell,
    # so consecutive samples are in the same or neighbouring cells: all of
    # them reachable = one unbroken drivable loop.
    r = LOOP_R if r is None else r
    cx, cy = NO_GO_CENTER
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def fountain_loop_ok(reached):
    bad = []
    for x, y in ring_points():
        i, j = world_to_cell(x, y)
        if not reached[j][i]:
            bad.append((round(x, 2), round(y, 2)))
    return bad


def write_png(path, grid, nogo, reached, stops_status, loop_ok=True):
    # RGB image, 1 pixel per grid cell. Flip vertically so +y (north) is up.
    rows = []
    for j in range(NY - 1, -1, -1):
        row = bytearray()
        for i in range(NX):
            if grid[j][i]:
                rgb = (40, 40, 40)               # real obstacle
            elif nogo[j][i]:
                rgb = (235, 150, 40)             # fountain no-go buffer
            elif reached[j][i]:
                rgb = (200, 235, 200)            # reachable free space
            else:
                rgb = (235, 235, 235)            # free but NOT reachable
            row += bytes(rgb)
        rows.append(row)

    def stamp(x, y, rgb, radius_px=3):
        ci, cj = world_to_cell(x, y)
        row_idx = (NY - 1) - cj
        for dj in range(-radius_px, radius_px + 1):
            for di in range(-radius_px, radius_px + 1):
                if di * di + dj * dj > radius_px * radius_px:
                    continue
                ri, rr = ci + di, row_idx + dj
                if 0 <= rr < NY and 0 <= ri < NX:
                    rows[rr][ri * 3:ri * 3 + 3] = bytes(rgb)

    for x, y in ring_points(n=240):   # fountain loop: teal if drivable, red if not
        stamp(x, y, (20, 150, 150) if loop_ok else (210, 30, 30), 0)
    stamp(*START, (30, 100, 220), 4)  # start = blue
    for name, (x, y) in STOPS.items():
        stamp(x, y, (30, 160, 40) if stops_status[name] else (210, 30, 30), 4)

    raw = bytearray()
    for row in rows:
        raw += b'\x00' + row  # filter type 0 (none) per scanline

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data +
                struct.pack('>I', zlib.crc32(tag + data)))

    ihdr = struct.pack('>IIBBBBB', NX, NY, 8, 2, 0, 0, 0)
    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) + \
        chunk(b'IDAT', zlib.compress(bytes(raw), 9)) + chunk(b'IEND', b'')
    with open(path, 'wb') as f:
        f.write(png)


def main():
    grid, nogo = build_grid()
    reached = bfs(grid, nogo, START)

    status = {}
    print(f'\nStart: {START}\n')
    all_ok = True
    for name, (x, y) in STOPS.items():
        i, j = world_to_cell(x, y)
        ok = 0 <= i < NX and 0 <= j < NY and reached[j][i]
        status[name] = ok
        all_ok &= ok
        print(f'  {name:8s} {str((x, y)):18s} {"OK" if ok else "UNREACHABLE"}')

    bad = fountain_loop_ok(reached)
    if bad:
        print(f'\n  Fountain loop (r={LOOP_R:.2f} m): BLOCKED at {len(bad)} points, e.g. {bad[:3]}')
    else:
        print(f'\n  Fountain loop (r={LOOP_R:.2f} m): OK, the robot can drive all the way around')

    write_png(OUT_PNG, grid, nogo, reached, status, loop_ok=not bad)
    print(f'\nMap saved to {OUT_PNG}')
    print('\nALL STOPS REACHABLE' if all_ok else '\nSOME STOPS ARE NOT REACHABLE - see the map')
    return 0 if all_ok and not bad else 1


if __name__ == '__main__':
    raise SystemExit(main())
