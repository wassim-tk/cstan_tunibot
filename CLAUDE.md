# CSTAM TuniBot

A restaurant service robot (BellaBot-style) simulated with **ROS 2 Jazzy + Gazebo Harmonic**
(gz-sim 8). The robot delivers orders from the kitchen to tables in "Dar TuniBot", a Tunisian
courtyard house turned into a fine-dining restaurant, and returns to its charging dock.

## Rules for working in this repo (Maram's)

- Work on the **`my-fixes`** branch only.
- **One step at a time.** After each step, stop and wait for Maram's Gazebo screenshot before
  starting the next one.
- **Do not commit or push.** Maram does all commits and pushes.
- Do not modify Wassim's (git author `spexii`) files unless explicitly asked, e.g.
  `cstam_web_bridge/`, `cstam_core/`, `docs/`, `Dockerfile`, `requirements.txt`,
  `cstam_gazebo/CMakeLists.txt`, `cstam_gazebo/launch/spawn_cstam_robot.launch.py`,
  `cstam_gazebo/models/{building,path,robot,table}`.
- After every world change, run both checks (all 14 stops + the fountain loop must pass):
  ```bash
  cd ros2_ws/src/cstam_gazebo
  python3 scripts/make_dar_world.py            # regenerates worlds/dar_tunibot.world
  gz sdf --check worlds/dar_tunibot.world
  python3 scripts/check_dar_world_navigability.py
  ```
- `worlds/dar_tunibot.world` is generated: edit `scripts/make_dar_world.py`, never the world.
- GPU: NVIDIA GTX 1650, driver 595 (installed 2026-09-29, before that it was software
  rendering). Still keep real lights few (3) and shadows off on point lights, and put detail
  counts / on-off toggles as settings at the top of the generator.

## Dar TuniBot layout (x -11..11, y -8..8, metres; walls 3 m high)

- **Courtyard** (x -4..4, y -2..4): grand 3-tier marble fountain at (0, 1) - solid inside
  r = 1.0 m, 1.6 m no-go circle for the robot. 8 arcade columns around it, marble inlay floor.
- **Kitchen** (x -11..-6, y 2..8): door on its east wall at x = -6, y 4.6..6.0; pickup counter
  along the north wall.
- **Bey's Salon / VIP** (x 6..11, y 3..8): narrow gold arch at y = 3, x 6.9..8.0.
- **Arcades** around the courtyard: tables 1-2 north, 3-4 east, 9-10 west; mint-tea bar on the
  east wall (x 10.65, y -1).
- **Jasmine terrace** (y -8..-4.4): tables 5-8, planters with rose hedges at y = -4.4,
  2 m entrance in the south wall (x -1..1), red carpet from the entrance to the courtyard.
- **Charging dock** at (-10.8, -1.45) on the west wall. Robot start: (-9.0, -1.45).
- Tables: cloth cylinder r = 0.55 (solid to the floor) + 4 chair seats (solid 0.40 x 0.40 box)
  at r = 0.82; everything else (place settings, chandeliers, chair backs) is visual only.

## The 14 robot stops (x, y)

| Stop | Position | Stop | Position |
|---|---|---|---|
| Dock | (-10.2, -1.45) | T6 | (-1.85, -6.3) |
| Kitchen | (-8.5, 6.55) | T7 | (1.85, -6.3) |
| T1 | (-2.3, 4.95) | T8 | (5.4, -6.3) |
| T2 | (2.3, 4.95) | T9 | (-6.1, 0.0) |
| T3 | (6.8, 1.0) | T10 | (-6.1, -2.9) |
| T4 | (6.8, -2.3) | VIP1 | (7.8, 6.4) |
| T5 | (-5.4, -6.3) | VIP2 | (7.8, 4.2) |

(Defined in `scripts/check_dar_world_navigability.py`.)

## Done / next

**Done**
- Robot spawns inside the restaurant by default.
- Dar TuniBot world, step A (fountain, marble floor, courtyard) and step B (fine-dining tables).
- Step C: 9 original paintings (`scripts/make_dar_art.py` -> `models/dar_art/`, run it with
  `venv/bin/python`, which has Pillow).

**Next**
1. Step D - grand piano in the courtyard.
2. Make `dar_tunibot.world` the default world in the launch files.
3. Robot fixes: frictionless casters, `/joint_states` bridge, SLAM autostart + `slam_params.yaml`.
4. Mapping the new world.
5. Waypoints for the 14 stops.
6. Jazzy-compatible `nav2_params.yaml`.
7. Delivery brain (order -> kitchen -> table -> dock).
