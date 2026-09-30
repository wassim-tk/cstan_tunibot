#!/usr/bin/env python3
"""Save the SLAM map (/map) as <name>.pgm + <name>.yaml for Nav2's map_server.

slam_toolbox builds the map in the frame of the robot's *starting* pose (the
docking station). `origin_offset` (default: the dock pose, heading 0) is added
to the map origin so the saved map is expressed in world coordinates, the same
frame as waypoints.yaml.

    ros2 run cstam_core save_map                       # -> cstam_navigation/maps/cstam_map
    ros2 run cstam_core save_map --ros-args -p output:=/tmp/my_map
"""
import os

import numpy as np

OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.25


def occupancy_to_pgm(data, width, height):
    """ROS OccupancyGrid data (row 0 = bottom) -> trinary PGM pixels (row 0 = top)."""
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    img = np.full(grid.shape, 205, dtype=np.uint8)            # unknown
    img[(grid >= 0) & (grid <= FREE_THRESH * 100)] = 254       # free
    img[grid >= OCCUPIED_THRESH * 100] = 0                     # occupied
    return np.flipud(img)


def write_map(path_no_ext, img, resolution, origin_x, origin_y):
    height, width = img.shape
    pgm_path = path_no_ext + '.pgm'
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n# CREATOR: cstam_core save_map {resolution:.3f} m/pix\n'
                f'{width} {height}\n255\n'.encode('ascii'))
        f.write(img.tobytes())
    with open(path_no_ext + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {os.path.basename(pgm_path)}\n'
                'mode: trinary\n'
                f'resolution: {resolution:.6f}\n'
                f'origin: [{origin_x:.6f}, {origin_y:.6f}, 0.000000]\n'
                'negate: 0\n'
                f'occupied_thresh: {OCCUPIED_THRESH}\n'
                f'free_thresh: {FREE_THRESH}\n')
    return pgm_path, path_no_ext + '.yaml'


def default_output():
    """Source-tree maps/ folder when found (so the map can be committed)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('cstam_navigation')
        src = os.path.normpath(os.path.join(share, '..', '..', '..', '..', 'src',
                                            'cstam_navigation', 'maps'))
        folder = src if os.path.isdir(src) else os.path.join(share, 'maps')
    except Exception:
        folder = os.getcwd()
    return os.path.join(folder, 'cstam_map')


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid

    rclpy.init(args=args)
    node = Node('cstam_map_saver')
    node.declare_parameter('output', default_output())
    node.declare_parameter('map_topic', '/map')
    node.declare_parameter('origin_offset', [7.85, -12.79])
    node.declare_parameter('timeout', 10.0)

    output = os.path.expanduser(node.get_parameter('output').value)
    offset = list(node.get_parameter('origin_offset').value)
    received = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, node.get_parameter('map_topic').value,
                             received.append, qos)

    deadline = node.get_clock().now().nanoseconds * 1e-9 + node.get_parameter('timeout').value
    while rclpy.ok() and not received and node.get_clock().now().nanoseconds * 1e-9 < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    code = 0
    if not received:
        node.get_logger().error('No map received - is slam_toolbox running?')
        code = 1
    else:
        msg = received[-1]
        info = msg.info
        img = occupancy_to_pgm(msg.data, info.width, info.height)
        os.makedirs(os.path.dirname(output) or '.', exist_ok=True)
        pgm, yml = write_map(output, img, info.resolution,
                             info.origin.position.x + offset[0],
                             info.origin.position.y + offset[1])
        node.get_logger().info(f'Map {info.width}x{info.height} saved to {pgm} and {yml}')
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
