#!/usr/bin/env python3
"""Autonomous mapping tour: drives the robot through every aisle of the
restaurant while slam_toolbox builds the map (launched by slam.launch.py).

Goals are expressed in the `world` frame; slam.launch.py publishes the static
world -> map transform (the dock pose) so they reach Nav2 correctly.

    ros2 run cstam_core mapping_tour
    ros2 run cstam_core save_map
"""

# (x, y) in world coordinates: the east aisle, then every east-west aisle.
# Consecutive points are at most ~7 m apart so each goal lies inside the part
# of the map that is already known.
DEFAULT_TOUR = [
    6.9, -12.2, 6.9, -8.4,
    1.0, -8.4, -5.0, -8.4, -9.9, -8.4,
    -9.9, -4.5, -3.0, -4.5, 3.0, -4.5, 6.9, -4.5,
    6.9, -0.4, 1.0, -0.4, -5.0, -0.4, -9.9, -0.4,
    -9.9, 3.3, -3.0, 3.3, 3.0, 3.3, 6.9, 3.3,
    6.9, -4.5, 6.9, -12.2,
    1.0, -12.2, -5.0, -12.2, -9.9, -12.2,
    -3.0, -12.2, 3.0, -12.2, 6.9, -12.2,
]


def main(args=None):
    import math

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose

    rclpy.init(args=args)
    node = Node('mapping_tour')
    node.declare_parameter('frame_id', 'world')
    node.declare_parameter('waypoints', DEFAULT_TOUR)
    frame = node.get_parameter('frame_id').value
    flat = list(node.get_parameter('waypoints').value)
    points = list(zip(flat[0::2], flat[1::2]))
    log = node.get_logger()

    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    log.info('Waiting for Nav2...')
    client.wait_for_server()

    reached = 0
    for i, (x, y) in enumerate(points):
        nx, ny = points[i + 1] if i + 1 < len(points) else (x, y)
        yaw = math.atan2(ny - y, nx - x) if (nx, ny) != (x, y) else 0.0
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        log.info(f'[{i + 1}/{len(points)}] heading to ({x:.1f}, {y:.1f})')
        for attempt in range(2):
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future)
            handle = future.result()
            if not handle.accepted:
                continue
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            if result_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                reached += 1
                break
        else:
            log.warn(f'could not reach ({x:.1f}, {y:.1f}), continuing')

    log.info(f'Mapping tour finished ({reached}/{len(points)} waypoints reached). '
             'Save the map with: ros2 run cstam_core save_map')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
