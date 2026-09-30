"""Named locations (dock, kitchen, tables) shared by the CSTAM nodes.

Waypoints live in ``cstam_navigation/config/waypoints.yaml``::

    frame_id: map
    locations:
      dock:    {x: 7.85, y: -12.79, yaw: 0.0}
      table_3: {x: 0.86, y: 1.41, yaw: 3.1416}
"""
import math
import os
import re

import yaml


class Waypoint:
    __slots__ = ('name', 'x', 'y', 'yaw')

    def __init__(self, name, x, y, yaw=0.0):
        self.name = name
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)

    def quaternion(self):
        """(x, y, z, w) quaternion for a planar heading."""
        return (0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0))

    def as_dict(self):
        return {'name': self.name, 'x': self.x, 'y': self.y, 'yaw': self.yaw}

    def __repr__(self):
        return f'Waypoint({self.name}, x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f})'


def normalize_name(name):
    """Map user input to a waypoint key.

    'Table 3', 'table-3', 'T3', '3' -> 'table_3';  'Kitchen' -> 'kitchen'.
    """
    text = str(name).strip().lower()
    if re.fullmatch(r'\d+', text):
        return f'table_{int(text)}'
    match = re.fullmatch(r'(?:table|t)[\s_\-]*(\d+)', text)
    if match:
        return f'table_{int(match.group(1))}'
    return re.sub(r'[\s\-]+', '_', text)


class WaypointBook:
    def __init__(self, waypoints, frame_id='map'):
        self.frame_id = frame_id
        self._waypoints = {wp.name: wp for wp in waypoints}

    @classmethod
    def from_dict(cls, data):
        locations = data.get('locations', {})
        waypoints = [Waypoint(name, v['x'], v['y'], v.get('yaw', 0.0))
                     for name, v in locations.items()]
        return cls(waypoints, data.get('frame_id', 'map'))

    @classmethod
    def from_yaml(cls, path):
        with open(path, 'r', encoding='utf-8') as f:
            return cls.from_dict(yaml.safe_load(f) or {})

    def get(self, name):
        """Return the waypoint for user input `name`, or None."""
        return self._waypoints.get(normalize_name(name))

    def names(self):
        return list(self._waypoints)

    def tables(self):
        return [n for n in self._waypoints if n.startswith('table_')]

    def __contains__(self, name):
        return self.get(name) is not None

    def __len__(self):
        return len(self._waypoints)


def default_waypoints_file():
    """waypoints.yaml installed by cstam_navigation (or the source tree)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('cstam_navigation'),
                            'config', 'waypoints.yaml')
        if os.path.exists(path):
            return path
    except Exception:  # ament index not available (plain unit tests)
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(
        here, '..', '..', 'cstam_navigation', 'config', 'waypoints.yaml'))
