# CSTAM-TUNIBOT Phase 1 Architecture

## 1. Node graph

```mermaid
graph TD
    subgraph Sim["Gazebo Harmonic (cstam_gazebo)"]
        World["restaurant.world<br/>tables, kitchen, dock"]
        Robot["cstam_robot<br/>diff-drive + 360° LiDAR"]
        Bridge["ros_gz_bridge<br/>/clock /cmd_vel /odom /scan /tf"]
        Robot --- World
        Robot --- Bridge
    end

    subgraph Mapping["Mapping (slam.launch.py)"]
        SLAM["slam_toolbox<br/>online async"]
        Tour["mapping_tour"]
        Saver["save_map"]
        SLAM -->|/map| Saver
    end

    subgraph Nav2["Nav2 (navigation.launch.py)"]
        MapSrv["map_server<br/>cstam_map.yaml"]
        AMCL["amcl"]
        BT["bt_navigator"]
        Planner["planner_server<br/>Smac 2D"]
        Ctrl["controller_server<br/>MPPI"]
        Beh["behavior_server<br/>spin / backup / wait"]
        Smooth["velocity_smoother"]
        CM["collision_monitor"]
        DockSrv["docking_server<br/>opennav_docking"]
        MapSrv --> AMCL
        BT --> Planner
        BT --> Ctrl
        BT --> Beh
        Ctrl -->|cmd_vel_nav| Smooth -->|cmd_vel_smoothed| CM
        DockSrv -->|navigate_to_pose| BT
    end

    subgraph Core["cstam_core"]
        TM["delivery_task_manager"]
        DC["docking_controller"]
        CLI["request_delivery"]
    end

    CLI -->|/delivery/request| TM
    TM -->|navigate_to_pose| BT
    TM -->|/delivery/status| DC
    DC -->|/dock/status| TM
    TM -->|/dock/command undock| DC
    DC -->|dock_robot / undock_robot| DockSrv
    CM -->|/cmd_vel| Bridge
    Bridge -->|/scan /odom /tf| AMCL
    Bridge -->|/scan /odom /tf| SLAM
    Tour -->|navigate_to_pose| BT
```

## 2. Interfaces

| Node | Interface | Type | Description |
| :--- | :--- | :--- | :--- |
| delivery_task_manager | `/delivery/request` (sub) | `std_msgs/String` | `"3"`, `"Table 3"` or `{"table": "Table 3", "item": "Espresso"}` |
| delivery_task_manager | `/delivery/confirm` (sub) | `std_msgs/Empty` | Operator confirms loading / unloading |
| delivery_task_manager | `/delivery/cancel_all` (srv) | `std_srvs/Trigger` | Drop the queue and the current order |
| delivery_task_manager | `/delivery/status` (pub) | `std_msgs/String` (JSON) | State, current order, queue, counters |
| delivery_task_manager | `navigate_to_pose` (client) | `nav2_msgs/NavigateToPose` | Kitchen and table goals |
| docking_controller | `/dock/command` (sub) | `std_msgs/String` | `dock`, `undock`, `resume` |
| docking_controller | `/battery_state` (sub) | `sensor_msgs/BatteryState` | Optional low-battery trigger |
| docking_controller | `/dock/status` (pub) | `std_msgs/String` (JSON) | `UNDOCKED / DOCKING / DOCKED / UNDOCKING`, reason, hold |
| docking_controller | `dock_robot`, `undock_robot` (clients) | `nav2_msgs` | opennav_docking server |

## 3. Delivery state machine

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> IDLE: queue empty / hold / waiting for undock
    IDLE --> TO_KITCHEN: next order (FIFO), robot undocked
    TO_KITCHEN --> LOADING: arrived
    LOADING --> TO_TABLE: confirm or load_time elapsed
    TO_TABLE --> UNLOADING: arrived
    UNLOADING --> IDLE: confirm or unload_time elapsed (delivered)
    TO_KITCHEN --> TO_KITCHEN: nav failure, retry (max_retries)
    TO_TABLE --> TO_TABLE: nav failure, retry
    TO_KITCHEN --> IDLE: retries exhausted (failed) / cancel
    TO_TABLE --> IDLE: retries exhausted (failed) / cancel
```

## 4. Docking state machine

```mermaid
stateDiagram-v2
    [*] --> DOCKED: robot boots on the dock
    DOCKED --> UNDOCKING: undock requested and no hold
    UNDOCKING --> UNDOCKED: backed out to staging pose
    UNDOCKED --> DOCKING: idle timeout / manual dock / low battery (once the order is finished)
    DOCKING --> DOCKED: docked on the pad
    DOCKING --> UNDOCKED: cancelled by a new order (idle return only) / failure, retried later
```
