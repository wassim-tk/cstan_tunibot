"""Unit tests for the ROS-independent Phase 1 logic (no ROS runtime needed).

    cd ros2_ws/src/cstam_core && python3 -m pytest test -q
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from cstam_core.delivery_task_manager import (  # noqa: E402
    IDLE, LOADING, TO_KITCHEN, TO_TABLE, UNLOADING, DeliveryManager)
from cstam_core.docking_controller import (  # noqa: E402
    DOCKED, DOCKING, UNDOCKED, UNDOCKING, DockingLogic)
from cstam_core.map_saver import occupancy_to_pgm, write_map  # noqa: E402
from cstam_core.waypoints import (  # noqa: E402
    WaypointBook, default_waypoints_file, normalize_name)


@pytest.fixture
def book():
    return WaypointBook.from_yaml(default_waypoints_file())


def make_manager(book, **kw):
    kw.setdefault('load_time', 5.0)
    kw.setdefault('unload_time', 5.0)
    return DeliveryManager(book, **kw)


def nav_cmds(cmds):
    return [c for c in cmds if c[0] == 'navigate']


# ------------------------------------------------------------------ waypoints
def test_waypoints_file_has_dock_kitchen_and_tables(book):
    assert book.frame_id == 'map'
    assert book.get('dock') is not None
    assert book.get('kitchen') is not None
    assert len(book.tables()) == 24


@pytest.mark.parametrize('text,expected', [
    ('Table 3', 'table_3'), ('table-3', 'table_3'), ('T3', 'table_3'),
    ('3', 'table_3'), ('table_12', 'table_12'), ('Kitchen', 'kitchen'),
])
def test_normalize_name(text, expected):
    assert normalize_name(text) == expected


def test_waypoint_quaternion_is_unit(book):
    for name in book.names():
        q = book.get(name).quaternion()
        assert abs(sum(c * c for c in q) - 1.0) < 1e-9


def test_dock_waypoint_matches_docking_server_config(book):
    import yaml
    params_file = os.path.join(os.path.dirname(default_waypoints_file()), 'nav2_params.yaml')
    with open(params_file) as f:
        params = yaml.safe_load(f)
    dock = params['docking_server']['ros__parameters']['home_dock']['pose']
    wp = book.get('dock')
    assert (wp.x, wp.y, wp.yaw) == pytest.approx(tuple(dock))


# -------------------------------------------------------------- task manager
def test_rejects_unknown_or_non_table_targets(book):
    m = make_manager(book)
    ok, err = m.add_order('Table 99')
    assert not ok and 'unknown table' in err
    ok, _ = m.add_order('kitchen')
    assert not ok
    assert len(m.queue) == 0


def test_full_delivery_cycle(book):
    m = make_manager(book)
    ok, order = m.add_order('Table 3', 'Espresso', now=0.0)
    assert ok and order['table'] == 'table_3'

    cmds = m.tick(0.0)
    assert m.state == TO_KITCHEN
    (_, seq, wp), = nav_cmds(cmds)
    assert wp.name == 'kitchen'

    m.on_nav_result(seq, True, now=10.0)
    assert m.state == LOADING
    assert nav_cmds(m.tick(12.0)) == []           # still loading
    cmds = m.tick(15.0)                           # load_time elapsed
    assert m.state == TO_TABLE
    (_, seq, wp), = nav_cmds(cmds)
    assert wp.name == 'table_3'

    m.on_nav_result(seq, True, now=30.0)
    assert m.state == UNLOADING
    m.tick(35.0)
    assert m.state == IDLE
    assert m.completed == 1 and m.history[-1]['status'] == 'delivered'


def test_fifo_order_and_operator_confirmation(book):
    m = make_manager(book, load_time=1e9, unload_time=1e9)
    for t in ('1', '2', '3'):
        m.add_order(t)
    delivered = []
    now = 0.0
    while len(delivered) < 3:
        now += 1.0
        for cmd in nav_cmds(m.tick(now)):
            m.on_nav_result(cmd[1], True, now)
        if m.state in (LOADING, UNLOADING):
            assert m.confirm()
        if m.history and m.history[-1]['table'] not in delivered:
            delivered.append(m.history[-1]['table'])
    assert delivered == ['table_1', 'table_2', 'table_3']


def test_navigation_retry_then_failure(book):
    m = make_manager(book, max_retries=2)
    m.add_order('5')
    m.add_order('6')
    (_, seq, _), = nav_cmds(m.tick(0.0))
    for attempt in range(2):
        m.on_nav_result(seq, False, 1.0)
        assert m.state == TO_KITCHEN
        (_, seq, _), = nav_cmds(m.tick(1.0))      # goal re-sent
    m.on_nav_result(seq, False, 2.0)
    assert m.state == IDLE and m.failed == 1
    m.tick(3.0)                                   # next order starts
    assert m.state == TO_KITCHEN and m.current['table'] == 'table_6'


def test_stale_navigation_results_are_ignored(book):
    m = make_manager(book)
    m.add_order('1')
    (_, seq, _), = nav_cmds(m.tick(0.0))
    m.on_nav_result(seq + 7, True, 1.0)
    assert m.state == TO_KITCHEN


def test_waits_for_undock_and_respects_hold(book):
    m = make_manager(book)
    m.add_order('4')
    cmds = m.tick(0.0, dock_ready=False)
    assert ('request_undock',) in cmds and m.state == IDLE
    assert m.tick(1.0, dock_ready=True, hold=True) == [] and m.state == IDLE
    m.tick(2.0, dock_ready=True, hold=False)
    assert m.state == TO_KITCHEN


def test_cancel_all(book):
    m = make_manager(book)
    for t in ('1', '2', '3'):
        m.add_order(t)
    m.tick(0.0)
    assert m.cancel_all() == 3
    assert ('cancel_nav',) in m.tick(1.0)
    assert m.state == IDLE and not m.queue
    status = m.status()
    assert status['queue_length'] == 0 and status['current'] is None


# ---------------------------------------------------------- docking logic
def test_idle_timeout_triggers_docking():
    d = DockingLogic(idle_timeout=10.0, start_docked=False)
    assert d.tick(0.0) == []
    assert d.tick(5.0) == []
    assert d.tick(10.0) == ['dock']
    assert d.state == DOCKING and d.reason == 'idle'
    d.on_dock_result(True)
    assert d.state == DOCKED


def test_busy_robot_does_not_dock():
    d = DockingLogic(idle_timeout=10.0, start_docked=False)
    for t in range(0, 100, 5):
        assert d.tick(float(t), task_idle=False, queue_empty=False) == []


def test_new_order_cancels_idle_docking_and_undocks():
    d = DockingLogic(idle_timeout=1.0, start_docked=False)
    d.tick(0.0)
    assert d.tick(1.0) == ['dock']
    d.command('undock')
    assert d.tick(2.0) == ['cancel_dock']
    d.on_dock_result(False, cancelled=True)
    assert d.state == UNDOCKED and d.dock_failures == 0


def test_undock_from_dock_on_request():
    d = DockingLogic(start_docked=True)
    assert d.state == DOCKED
    d.command('undock')
    assert d.tick(0.0) == ['undock'] and d.state == UNDOCKING
    d.on_undock_result(True)
    assert d.state == UNDOCKED


def test_manual_dock_holds_until_resume():
    d = DockingLogic(idle_timeout=1e9, start_docked=False)
    d.command('dock')
    assert d.hold
    assert d.tick(0.0, task_idle=False) == []      # finish current order first
    assert d.tick(1.0, task_idle=True, queue_empty=False) == ['dock']
    assert d.reason == 'manual'
    d.on_dock_result(True)
    d.command('undock')
    assert d.tick(2.0) == [] and d.state == DOCKED  # held
    d.command('resume')
    d.command('undock')
    assert d.tick(3.0) == ['undock']


def test_low_battery_docks_and_waits_for_recharge():
    d = DockingLogic(idle_timeout=1e9, low_battery_threshold=20.0,
                     resume_battery_threshold=80.0, start_docked=False)
    d.update_battery(50.0)
    assert not d.hold
    d.update_battery(15.0)
    assert d.hold
    assert d.tick(0.0, task_idle=True, queue_empty=False) == ['dock']
    assert d.reason == 'low_battery'
    d.on_dock_result(True)
    d.update_battery(60.0)
    d.command('undock')
    assert d.tick(1.0) == [] and d.hold
    d.update_battery(85.0)
    d.command('undock')
    assert d.tick(2.0) == ['undock']


def test_failed_docking_retries_after_idle_period():
    d = DockingLogic(idle_timeout=5.0, start_docked=False)
    d.tick(0.0)
    assert d.tick(5.0) == ['dock']
    d.on_dock_result(False, now=6.0)
    assert d.state == UNDOCKED and d.dock_failures == 1
    assert d.tick(8.0) == []
    assert d.tick(11.0) == ['dock']


# ---------------------------------------------------------------- map saver
def test_occupancy_to_pgm_and_yaml(tmp_path):
    # 2x3 grid, row 0 = bottom: [free, occupied], [unknown, free], [occupied, 50]
    data = [0, 100, -1, 10, 100, 50]
    img = occupancy_to_pgm(data, 2, 3)
    assert img.tolist() == [[0, 205], [205, 254], [254, 0]]
    pgm, yml = write_map(str(tmp_path / 'm'), img, 0.05, 1.5, -2.0)
    text = open(yml).read()
    assert 'origin: [1.500000, -2.000000, 0.000000]' in text
    raw = open(pgm, 'rb').read()
    assert raw.startswith(b'P5') and raw.endswith(img.tobytes())


def test_committed_map_is_valid_and_waypoints_are_free(book):
    import yaml
    map_yaml = os.path.join(os.path.dirname(default_waypoints_file()), '..', 'maps', 'cstam_map.yaml')
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(os.path.dirname(map_yaml), meta['image']), 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = map(int, line.split())
        f.readline()
        img = np.frombuffer(f.read(), dtype=np.uint8).reshape(height, width)
    res = meta['resolution']
    ox, oy = meta['origin'][:2]
    for name in book.names():
        wp = book.get(name)
        col = int((wp.x - ox) / res)
        row = height - 1 - int((wp.y - oy) / res)
        # the robot footprint (0.3 m) around every waypoint must be mapped free
        r = int(0.3 / res)
        patch = img[row - r:row + r + 1, col - r:col + r + 1]
        assert patch.size and (patch == 254).mean() > 0.9, f'{name} is not in free space'
