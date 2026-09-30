# CSTAM-TUNIBOT: Autonomous Indoor Delivery Robot (Phase 1 MVP)

ROS 2 **Jazzy** + Gazebo **Harmonic** + Nav2 stack for a BellaBot-style
restaurant delivery robot. This branch contains **only Phase 1**; the web
dashboard, battery simulator and other extras live on the `extra` branch.

| Phase 1 requirement | Points | Where |
| :--- | :---: | :--- |
| Mapping & Localization (SLAM) | 15 | `slam_toolbox` (`cstam_navigation/launch/slam.launch.py`, `config/slam_params.yaml`), `mapping_tour`, `save_map`, AMCL on the saved map |
| Autonomous Navigation | 15 | Nav2 (`cstam_navigation/config/nav2_params.yaml`): Smac 2D planner, MPPI controller, costmaps with 0.38 m footprint, collision monitor, recovery behaviours |
| Delivery Task Management | 10 | `cstam_core/delivery_task_manager.py`: FIFO queue, kitchen → table cycle, retries, cancel, operator confirmation |
| Auto-Docking Behavior | 10 | `cstam_core/docking_controller.py` + Nav2 `opennav_docking`: idle / manual / low-battery triggers, return-to-dock and undock sequences |

---

## 1. Install

Ubuntu 24.04 + ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
                    ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
                    python3-numpy python3-yaml
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 2. Run the delivery system (uses the committed map)

```bash
ros2 launch cstam_core cstam_system.launch.py          # Gazebo + RViz + Nav2 + task manager + docking
```

The robot starts **on the charging dock**. Order deliveries from another terminal:

```bash
ros2 run cstam_core request_delivery 3 "Espresso"        # table 3
ros2 run cstam_core request_delivery "Table 14" "Juice"  # names: 0..23, "Table 7", "table_7", "T7"
```

For each order the robot undocks if needed, then drives to the kitchen pass-through
window, waits for loading, drives to the table and waits for the customer, then
starts the next order. After `idle_timeout` seconds (30 s by default) with nothing to do,
it returns to the dock on its own.

| Action | Command |
| :--- | :--- |
| Confirm tray loaded / taken (skip the wait) | `ros2 topic pub --once /delivery/confirm std_msgs/msg/Empty` |
| Cancel everything | `ros2 service call /delivery/cancel_all std_srvs/srv/Trigger` |
| Send robot to dock and keep it there | `ros2 topic pub --once /dock/command std_msgs/msg/String "{data: dock}"` |
| Release the manual dock hold | `ros2 topic pub --once /dock/command std_msgs/msg/String "{data: resume}"` |
| Simulate low battery / recharge | `ros2 topic pub --times 3 /battery_state sensor_msgs/msg/BatteryState "{percentage: 0.15}"` (then `0.95`) |
| Watch state | `ros2 topic echo /delivery/status` and `ros2 topic echo /dock/status` |

Launch arguments: `headless:=true` (no Gazebo GUI), `rviz:=false`, `load_time:=10.0`,
`unload_time:=10.0`, `idle_timeout:=30.0`, `map:=/path/to/map.yaml`.

## 3. Build a new map with SLAM

```bash
ros2 launch cstam_navigation slam.launch.py     # Gazebo + slam_toolbox + Nav2 (no AMCL)
ros2 run cstam_core mapping_tour                 # autonomous tour of every aisle (~8 min)
                                                 # (or drive with teleop / RViz "Nav2 Goal")
ros2 run cstam_core save_map                     # -> ros2_ws/src/cstam_navigation/maps/cstam_map.{pgm,yaml}
```

Then rebuild (`colcon build`) or pass `map:=...`, and run section 2.

**Frames.** The robot starts on the dock facing +X, so slam_toolbox's `map` frame is the
world frame shifted by the dock position. `save_map` adds that offset to the map origin,
so the saved map, `waypoints.yaml` and the docking-server dock pose are all in **world
coordinates**. During SLAM a static `world → map` transform does the same for goals.

## 4. Tests

```bash
cd ros2_ws/src/cstam_core && python3 -m pytest test -q
```

The 25 tests cover the delivery state machine, docking triggers, waypoint parsing, the map
writer, and a check that every waypoint lies in free space on the committed map.

System-level check in simulation (headless): the robot undocked, delivered to tables 3,
14, 20 and 5 via the kitchen, then docked in three situations: after going idle, on low
battery (holding the next order until the recharge), and on a manual `dock` command. It
docked within 6 cm of the pad centre, and a cancelled order stopped the robot mid-route.

---

## Design notes

- **World** (`cstam_gazebo/worlds/restaurant.world`): 25 × 17.7 m restaurant, kitchen with a
  pass-through window, restrooms, 24 tables with chairs, and a charging dock at (7.85, −12.79).
  The table tops (0.73 m) sit above the 2D LiDAR's scan plane, so each table has a solid
  base the LiDAR can see. Without it the robot would plan paths under the tables.
- **Robot** (`cstam_gazebo/urdf/cstam_robot.urdf.xacro`): differential drive, 360° LiDAR at
  0.19 m. The casters are frictionless and sit 5 mm above the floor, and the wheels use
  sphere contacts. This keeps wheel odometry within about 1% of ground truth, which is
  what makes the SLAM map and AMCL reliable.
- **Physics step** 4 ms: real time on a laptop CPU, plenty for a 0.5 m/s robot.
- **Waypoints** (`cstam_navigation/config/waypoints.yaml`): `dock`, `kitchen`, and
  `table_0..23`. Each table waypoint is a serving spot in the aisle at the end of the
  table, facing it.
- **Task manager / docking handshake**: the docking controller publishes `/dock/status`
  (`state`, `hold`). The task manager only starts an order when the robot is `UNDOCKED`
  and there is no hold. Otherwise it asks for `undock` on `/dock/command`. The docking
  controller never interrupts an order that is in progress: a manual or low-battery dock
  request sets `hold` and docks once the current order is finished.

See [architecture-diagram.md](architecture-diagram.md) for the node graph and interfaces.

## Assumptions / limitations

- The battery is not simulated in Phase 1 (it's on the `extra` branch). The low-battery
  trigger listens to the standard `/battery_state` topic, so a real battery driver or
  `ros2 topic pub` drives it.
- Docking uses the known dock pose from the map (no fiducial detection). The docking
  server reports "charging" as soon as the robot is on the pad.
