# CSTAM TuniBot

A restaurant service robot (BellaBot-style) simulated with **ROS 2 Jazzy + Gazebo Harmonic**
(gz-sim 8). The robot delivers orders from the kitchen to tables in "Dar TuniBot", a Tunisian
courtyard house turned into a fine-dining restaurant, and returns to its charging dock.

**Decision:** Dar TuniBot is our Phase 1 world. Maram works independently on `my-fixes` and
borrows useful parts of Wassim's `main`. His `restaurant.world`, his 24-table
`waypoints.yaml` and `cstam_map` are **not used**.

## How to launch

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch cstam_gazebo dar_sim.launch.py
```
This is THE way to launch. `dar_sim.launch.py` (ours) includes Wassim's
`spawn_cstam_robot.launch.py` with world `dar_tunibot.world` and spawn (-9.0, -1.45), yaw 0.0,
and adds a `joint_state_bridge` for `/joint_states`.

## Rules for working in this repo (Maram's)

- Work on the **`my-fixes`** branch only.
- **One step at a time.** After each step, stop and wait for Maram's Gazebo screenshot before
  starting the next one.
- **Do not commit or push.** Maram does all commits and pushes.
- Do not modify Wassim's (git author `spexii`) files unless explicitly asked, e.g.
  `cstam_web_bridge/`, `cstam_core/`, `docs/`, `Dockerfile`, `requirements.txt`,
  `cstam_gazebo/CMakeLists.txt`, `cstam_gazebo/launch/spawn_cstam_robot.launch.py`,
  `cstam_gazebo/models/{building,path,robot,table}`.
- Wassim's `main` was merged into `my-fixes` (commit 5b259e5), keeping **his**
  `spawn_cstam_robot.launch.py` (defaults x 7.06, y -12.0, yaw 1.57; has `spawn_yaw`).
  **Never edit it** - pass launch arguments to it instead (as `dar_sim.launch.py` does).
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
- Grand piano at (3.18, 3.17), yaw 0.6, in the courtyard's NE corner (one invisible solid box
  1.6 x 1.5 x 1.0 m + a solid bench box). Any closer to the fountain blocks the fountain loop.

## Robot / Wassim's code notes

- URDF (`cstam_robot.urdf.xacro`): casters `caster_front` / `caster_rear` are frictionless
  (mu1 = mu2 = 0) so they don't drag and spoil the odometry.
- Wassim's `nav2_params.yaml` already uses Jazzy plugin names and has `collision_monitor`,
  `velocity_smoother` and `docking_server`: reuse it in step 5, don't rebuild it from scratch.
- His `delivery_task_manager.py` still never sends Nav2 goals (so step 7 is still needed).

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

**World (done):** steps A-D - fountain + marble floor, fine-dining tables, 9 original paintings
(`scripts/make_dar_art.py` -> `models/dar_art/`, run with `venv/bin/python`, which has Pillow),
grand piano.

**Plan status**
- Step 0 - merge Wassim's `main` into `my-fixes`: **done**.
- Step 1 - `dar_sim.launch.py` (Dar TuniBot world + start pose): **done**.
- Step 2 - robot fixes: frictionless casters **done**; `/joint_states` bridge **written, not yet
  tested**.
- Step 3 - SLAM autostart + `slam_params.yaml`: **NEXT**.
- Step 4 - mapping the Dar TuniBot world.
- Step 5 - Nav2: reuse Wassim's Jazzy `nav2_params.yaml`.
- Step 6 - waypoints for the 14 stops.
- Step 7 - delivery brain (order -> kitchen -> table -> dock), sending real Nav2 goals.

## Known issues

- Driving feels laggy: check Gazebo's real-time factor (RTF) before recording the mapping video.
- libEGL `failed to create dri2 screen` warnings are harmless (NVIDIA rendering verified with
  `glxinfo`).
