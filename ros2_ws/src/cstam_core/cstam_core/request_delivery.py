#!/usr/bin/env python3
"""Command-line helper to order a delivery.

    ros2 run cstam_core request_delivery 3 "Espresso"
    ros2 run cstam_core request_delivery table_12 "Orange juice"
"""
import json
import sys


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    argv = sys.argv[1:] if argv is None else argv
    args = [a for a in rclpy.utilities.remove_ros_args(argv)]
    if not args:
        print('usage: request_delivery <table> [item]')
        return 2
    payload = json.dumps({'table': args[0], 'item': ' '.join(args[1:])})

    rclpy.init()
    node = Node('request_delivery')
    pub = node.create_publisher(String, '/delivery/request', 10)
    # Wait (max 5 s) for the task manager to be discovered before publishing.
    for _ in range(50):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('warning: no delivery_task_manager is listening on /delivery/request')
    pub.publish(String(data=payload))
    rclpy.spin_once(node, timeout_sec=0.3)
    print(f'sent {payload}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
