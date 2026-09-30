#!/usr/bin/env python3
"""Delivery task manager.

Receives food / beverage delivery requests, keeps them in a FIFO queue and
executes them one after the other with Nav2:

    IDLE -> TO_KITCHEN -> LOADING -> TO_TABLE -> UNLOADING -> IDLE (next order)

ROS interface
-------------
Sub  /delivery/request  std_msgs/String   "table_3", "Table 3", "3" or
                                          '{"table": "Table 3", "item": "Espresso"}'
Sub  /delivery/confirm  std_msgs/Empty    operator confirms loading / unloading
                                          (otherwise the robot waits load_time /
                                          unload_time seconds)
Srv  /delivery/cancel_all std_srvs/Trigger  drop the queue and the current order
Pub  /delivery/status   std_msgs/String   JSON snapshot of the manager
Pub  /dock/command      std_msgs/String   "undock" when orders wait and the robot
                                          is on the dock
Sub  /dock/status       std_msgs/String   JSON from docking_controller
Act  navigate_to_pose   nav2_msgs/action/NavigateToPose
"""
import json
from collections import deque

from cstam_core.waypoints import WaypointBook, default_waypoints_file, normalize_name

IDLE = 'IDLE'
TO_KITCHEN = 'TO_KITCHEN'
LOADING = 'LOADING'
TO_TABLE = 'TO_TABLE'
UNLOADING = 'UNLOADING'

NAVIGATING_STATES = (TO_KITCHEN, TO_TABLE)


class DeliveryManager:
    """ROS-independent delivery queue and state machine.

    The owner calls `tick(now, ...)` periodically and executes the returned
    commands; navigation outcomes are reported back with `on_nav_result`.
    Commands are tuples:
        ('navigate', seq, waypoint)   send a Nav2 goal (seq identifies it)
        ('cancel_nav',)               cancel the active Nav2 goal
        ('request_undock',)           ask the docking controller to undock
    """

    def __init__(self, waypoints, pickup='kitchen', load_time=10.0,
                 unload_time=10.0, max_retries=2, history_size=50):
        self.book = waypoints
        self.pickup = normalize_name(pickup)
        if self.book.get(self.pickup) is None:
            raise ValueError(f"pickup location '{pickup}' not in waypoints")
        self.load_time = float(load_time)
        self.unload_time = float(unload_time)
        self.max_retries = int(max_retries)

        self.queue = deque()
        self.history = deque(maxlen=history_size)
        self.current = None
        self.state = IDLE
        self.note = ''
        self.completed = 0
        self.failed = 0

        self._next_id = 1
        self._nav_seq = 0
        self._nav_pending = False   # a goal must be (re)sent on next tick
        self._nav_active = False    # a goal is in flight
        self._stage_started = 0.0
        self._confirmed = False
        self._cancel_requested = False

    # ------------------------------------------------------------------ input
    def add_order(self, table, item='', now=0.0):
        """Queue a delivery. Returns (True, order) or (False, error)."""
        key = normalize_name(table)
        wp = self.book.get(key)
        if wp is None or not key.startswith('table_'):
            valid = ', '.join(sorted(self.book.tables(), key=lambda n: int(n.split('_')[1])))
            return False, f"unknown table '{table}' (valid: {valid})"
        order = {
            'id': self._next_id,
            'table': wp.name,
            'item': item or 'order',
            'status': 'queued',
            'created': now,
            'retries': 0,
        }
        self._next_id += 1
        self.queue.append(order)
        return True, order

    def confirm(self):
        """Operator confirmation: tray loaded (LOADING) or taken (UNLOADING)."""
        if self.state in (LOADING, UNLOADING):
            self._confirmed = True
            return True
        return False

    def cancel_all(self, now=0.0):
        """Drop every queued order and abort the current one."""
        dropped = len(self.queue)
        for order in self.queue:
            order['status'] = 'cancelled'
            self.history.append(order)
        self.queue.clear()
        if self.current is not None:
            if self._nav_active:
                self._cancel_requested = True
            self._finish('cancelled', now)
            dropped += 1
        return dropped

    # ------------------------------------------------------------- execution
    def tick(self, now, dock_ready=True, hold=False):
        cmds = []
        if self._cancel_requested:
            self._cancel_requested = False
            self._nav_active = False
            cmds.append(('cancel_nav',))

        if self.state == IDLE:
            if not self.queue:
                self.note = ''
            elif hold:
                self.note = 'on hold (docking controller)'
            elif not dock_ready:
                self.note = 'waiting for undock'
                cmds.append(('request_undock',))
            else:
                self.current = self.queue.popleft()
                self.current['status'] = 'to_kitchen'
                self.current['started'] = now
                self._enter(TO_KITCHEN, now)

        elif self.state == LOADING:
            if self._confirmed or now - self._stage_started >= self.load_time:
                self.current['status'] = 'to_table'
                self._enter(TO_TABLE, now)

        elif self.state == UNLOADING:
            if self._confirmed or now - self._stage_started >= self.unload_time:
                self._finish('delivered', now)

        if self._nav_pending and self.state in NAVIGATING_STATES:
            self._nav_pending = False
            self._nav_active = True
            self._nav_seq += 1
            target = self.pickup if self.state == TO_KITCHEN else self.current['table']
            cmds.append(('navigate', self._nav_seq, self.book.get(target)))
        return cmds

    def on_nav_result(self, seq, success, now):
        if seq != self._nav_seq or not self._nav_active:
            return  # stale result (cancelled or superseded goal)
        self._nav_active = False
        if self.state not in NAVIGATING_STATES:
            return
        if success:
            if self.state == TO_KITCHEN:
                self.current['status'] = 'loading'
                self._enter(LOADING, now)
            else:
                self.current['status'] = 'unloading'
                self._enter(UNLOADING, now)
            return
        self.current['retries'] += 1
        if self.current['retries'] > self.max_retries:
            self._finish('failed', now)
        else:
            self.note = f"navigation failed, retry {self.current['retries']}/{self.max_retries}"
            self._nav_pending = True

    # --------------------------------------------------------------- helpers
    def _enter(self, state, now):
        self.state = state
        self._stage_started = now
        self._confirmed = False
        self.note = ''
        if state in NAVIGATING_STATES:
            self._nav_pending = True

    def _finish(self, status, now):
        order = self.current
        order['status'] = status
        order['finished'] = now
        if status == 'delivered':
            self.completed += 1
        elif status == 'failed':
            self.failed += 1
        self.history.append(order)
        self.current = None
        self.state = IDLE
        self._nav_pending = False
        self._confirmed = False

    @property
    def busy(self):
        return self.state != IDLE

    def status(self):
        return {
            'state': self.state,
            'note': self.note,
            'current': dict(self.current) if self.current else None,
            'queue': [dict(o) for o in self.queue],
            'queue_length': len(self.queue),
            'completed': self.completed,
            'failed': self.failed,
            'recent': [dict(o) for o in list(self.history)[-5:]],
        }


# ---------------------------------------------------------------------- ROS
def main(args=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import Empty, String
    from std_srvs.srv import Trigger

    class DeliveryTaskManagerNode(Node):
        def __init__(self):
            super().__init__('delivery_task_manager')
            self.declare_parameter('waypoints_file', default_waypoints_file())
            self.declare_parameter('pickup_location', 'kitchen')
            self.declare_parameter('load_time', 10.0)
            self.declare_parameter('unload_time', 10.0)
            self.declare_parameter('max_retries', 2)
            self.declare_parameter('dock_status_timeout', 5.0)

            path = self.get_parameter('waypoints_file').value
            self.book = WaypointBook.from_yaml(path)
            self.manager = DeliveryManager(
                self.book,
                pickup=self.get_parameter('pickup_location').value,
                load_time=self.get_parameter('load_time').value,
                unload_time=self.get_parameter('unload_time').value,
                max_retries=self.get_parameter('max_retries').value,
            )
            self.dock_status_timeout = self.get_parameter('dock_status_timeout').value
            self.get_logger().info(f'Loaded {len(self.book)} waypoints from {path}')

            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.goal_handle = None

            self.dock_state = None
            self.dock_hold = False
            self.dock_stamp = None
            self.last_undock_request = -1e9
            self.last_status = None
            self.prev_state = IDLE

            self.status_pub = self.create_publisher(String, '/delivery/status', 10)
            self.dock_cmd_pub = self.create_publisher(String, '/dock/command', 10)
            self.create_subscription(String, '/delivery/request', self.on_request, 10)
            self.create_subscription(Empty, '/delivery/confirm', self.on_confirm, 10)
            self.create_subscription(String, '/dock/status', self.on_dock_status, 10)
            self.create_service(Trigger, '/delivery/cancel_all', self.on_cancel_all)
            self.create_timer(0.5, self.on_timer)
            self.create_timer(1.0, lambda: self.publish_status(force=True))
            self.get_logger().info('Delivery task manager ready: publish a table on /delivery/request')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        # ---- callbacks
        def on_request(self, msg):
            text = msg.data.strip()
            table, item = text, ''
            if text.startswith('{'):
                try:
                    data = json.loads(text)
                    table = str(data.get('table', data.get('target', '')))
                    item = str(data.get('item', ''))
                except (ValueError, AttributeError) as exc:
                    self.get_logger().error(f'Bad delivery request {text!r}: {exc}')
                    return
            ok, result = self.manager.add_order(table, item, self.now())
            if ok:
                self.get_logger().info(
                    f"Order #{result['id']} queued: {result['item']} -> {result['table']} "
                    f"(queue length {len(self.manager.queue)})")
            else:
                self.get_logger().warn(f'Rejected delivery request: {result}')
            self.publish_status(force=True)

        def on_confirm(self, _msg):
            if self.manager.confirm():
                self.get_logger().info(f'Operator confirmed {self.manager.state.lower()}')

        def on_cancel_all(self, _request, response):
            dropped = self.manager.cancel_all(self.now())
            response.success = True
            response.message = f'cancelled {dropped} order(s)'
            self.get_logger().warn(response.message)
            self.execute(self.manager.tick(self.now(), *self.dock_gate()))
            return response

        def on_dock_status(self, msg):
            try:
                data = json.loads(msg.data)
            except ValueError:
                return
            self.dock_state = data.get('state')
            self.dock_hold = bool(data.get('hold', False))
            self.dock_stamp = self.now()

        # ---- main loop
        def dock_gate(self):
            """(dock_ready, hold). Without a docking controller, never block."""
            if self.dock_stamp is None or self.now() - self.dock_stamp > self.dock_status_timeout:
                return True, False
            return self.dock_state == 'UNDOCKED', self.dock_hold

        def on_timer(self):
            self.execute(self.manager.tick(self.now(), *self.dock_gate()))
            self.publish_status()

        def execute(self, cmds):
            for cmd in cmds:
                if cmd[0] == 'navigate':
                    self.send_goal(cmd[1], cmd[2])
                elif cmd[0] == 'cancel_nav':
                    if self.goal_handle is not None:
                        self.goal_handle.cancel_goal_async()
                        self.goal_handle = None
                elif cmd[0] == 'request_undock':
                    if self.now() - self.last_undock_request > 3.0:
                        self.last_undock_request = self.now()
                        self.dock_cmd_pub.publish(String(data='undock'))
                        self.get_logger().info('Orders waiting: requesting undock')

        def send_goal(self, seq, wp):
            if not self.nav_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("Nav2 action server 'navigate_to_pose' not available")
                self.manager.on_nav_result(seq, False, self.now())
                return
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = self.book.frame_id
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = wp.x
            goal.pose.pose.position.y = wp.y
            q = wp.quaternion()
            goal.pose.pose.orientation.z = q[2]
            goal.pose.pose.orientation.w = q[3]
            order = self.manager.current
            self.get_logger().info(
                f"Order #{order['id']}: navigating to {wp.name} ({wp.x:.2f}, {wp.y:.2f})")
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.on_goal_response(seq, f))

        def on_goal_response(self, seq, future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn('Navigation goal rejected')
                self.manager.on_nav_result(seq, False, self.now())
                return
            if seq != self.manager._nav_seq:  # superseded / cancelled meanwhile
                handle.cancel_goal_async()
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.on_nav_done(seq, f))

        def on_nav_done(self, seq, future):
            status = future.result().status
            success = status == GoalStatus.STATUS_SUCCEEDED
            if seq == self.manager._nav_seq:
                self.goal_handle = None
                if not success and status != GoalStatus.STATUS_CANCELED:
                    self.get_logger().warn(f'Navigation ended with status {status}')
            self.manager.on_nav_result(seq, success, self.now())
            self.publish_status()

        def log_transition(self, prev_state):
            m = self.manager
            if m.state == prev_state:
                return
            if m.state == LOADING:
                self.get_logger().info(
                    f"Order #{m.current['id']}: at the kitchen, loading '{m.current['item']}' "
                    f"(confirm on /delivery/confirm or wait {m.load_time:.0f}s)")
            elif m.state == TO_TABLE:
                self.get_logger().info(f"Order #{m.current['id']}: loaded")
            elif m.state == UNLOADING:
                self.get_logger().info(
                    f"Order #{m.current['id']}: arrived at {m.current['table']}, "
                    f"waiting for the customer (confirm or wait {m.unload_time:.0f}s)")
            elif m.state == IDLE and m.history:
                last = m.history[-1]
                self.get_logger().info(
                    f"Order #{last['id']} {last['status']} ({last['table']}); "
                    f"{len(m.queue)} order(s) left in queue")

        def publish_status(self, force=False):
            if self.manager.state != self.prev_state:
                self.log_transition(self.prev_state)
                self.prev_state = self.manager.state
            text = json.dumps(self.manager.status())
            if force or text != self.last_status:
                self.last_status = text
                self.status_pub.publish(String(data=text))

    rclpy.init(args=args)
    node = DeliveryTaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
