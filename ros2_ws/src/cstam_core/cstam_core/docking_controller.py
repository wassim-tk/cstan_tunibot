#!/usr/bin/env python3
"""Auto-docking controller.

Decides *when* the robot returns to its charging dock and executes the
return-to-dock / leave-dock sequences through Nav2's docking server
(opennav_docking: navigate to the staging pose, then a controlled final
approach onto the pad).

Dock triggers
  * idle      : no order in progress and an empty queue for `idle_timeout` s
  * manual    : "dock" on /dock/command (robot stays docked until "resume")
  * low battery: /battery_state below `low_battery_threshold` %; the robot
                 stays docked until `resume_battery_threshold` % is reached

The controller never interrupts a delivery: with a manual / low-battery
request it sets `hold`, which stops the task manager from starting new
orders, and docks as soon as the current order is finished.

ROS interface
-------------
Sub /dock/command     std_msgs/String   "dock" | "undock" | "resume"
Sub /delivery/status  std_msgs/String   JSON from delivery_task_manager
Sub /battery_state    sensor_msgs/BatteryState  (optional)
Pub /dock/status      std_msgs/String   JSON {state, reason, hold, battery}
Act dock_robot / undock_robot (nav2_msgs)  or navigate_to_pose as fallback
"""
import json

UNDOCKED = 'UNDOCKED'
DOCKING = 'DOCKING'
DOCKED = 'DOCKED'
UNDOCKING = 'UNDOCKING'


class DockingLogic:
    """ROS-independent auto-docking state machine.

    `tick()` returns a list of actions: 'dock', 'undock' or 'cancel_dock'.
    The owner reports outcomes with `on_dock_result` / `on_undock_result`.
    """

    def __init__(self, idle_timeout=30.0, low_battery_threshold=20.0,
                 resume_battery_threshold=80.0, start_docked=True):
        self.idle_timeout = float(idle_timeout)
        self.low_battery_threshold = float(low_battery_threshold)
        self.resume_battery_threshold = float(resume_battery_threshold)

        self.state = DOCKED if start_docked else UNDOCKED
        self.reason = 'startup' if start_docked else ''
        self.battery = None
        self.manual_hold = False
        self.battery_hold = False
        self.undock_requested = False
        self.idle_since = None
        self.dock_failures = 0

    @property
    def hold(self):
        return self.manual_hold or self.battery_hold

    # ---------------------------------------------------------------- inputs
    def update_battery(self, percentage):
        self.battery = float(percentage)
        if self.battery < self.low_battery_threshold:
            self.battery_hold = True
        elif self.battery >= self.resume_battery_threshold:
            self.battery_hold = False

    def command(self, cmd):
        """Returns True if the command was understood."""
        cmd = cmd.strip().lower()
        if cmd == 'dock':
            self.manual_hold = True
        elif cmd == 'resume':
            self.manual_hold = False
        elif cmd == 'undock':
            self.undock_requested = True
        else:
            return False
        return True

    # ------------------------------------------------------------ main logic
    def tick(self, now, task_idle=True, queue_empty=True):
        actions = []
        if self.state == UNDOCKED:
            self.undock_requested = False
            reason = None
            if self.hold and task_idle:
                reason = 'low_battery' if self.battery_hold else 'manual'
            elif task_idle and queue_empty:
                if self.idle_since is None:
                    self.idle_since = now
                elif now - self.idle_since >= self.idle_timeout:
                    reason = 'idle'
            else:
                self.idle_since = None
            if reason:
                self.state = DOCKING
                self.reason = reason
                self.idle_since = None
                actions.append('dock')

        elif self.state == DOCKING:
            # A new order cancels an idle return; hold requests do not.
            if self.undock_requested and not self.hold:
                self.undock_requested = False
                actions.append('cancel_dock')

        elif self.state == DOCKED:
            if self.undock_requested and not self.hold:
                self.undock_requested = False
                self.state = UNDOCKING
                actions.append('undock')
            else:
                self.undock_requested = False
        return actions

    def on_dock_result(self, success, now=0.0, cancelled=False):
        if self.state != DOCKING:
            return
        if success:
            self.state = DOCKED
            self.dock_failures = 0
        elif cancelled:
            self.state = UNDOCKED
            self.reason = ''
            self.idle_since = None
        else:
            self.state = UNDOCKED
            self.dock_failures += 1
            self.reason = 'dock_failed'
            self.idle_since = now  # retry after another idle period

    def on_undock_result(self, success, now=0.0):
        if self.state != UNDOCKING:
            return
        self.state = UNDOCKED if success else DOCKED
        self.reason = '' if success else 'undock_failed'
        self.idle_since = now

    def status(self):
        return {
            'state': self.state,
            'reason': self.reason,
            'hold': self.hold,
            'manual_hold': self.manual_hold,
            'battery_hold': self.battery_hold,
            'battery': self.battery,
            'dock_failures': self.dock_failures,
        }


# ---------------------------------------------------------------------- ROS
def main(args=None):
    import math

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String

    from cstam_core.waypoints import WaypointBook, default_waypoints_file

    class DockingControllerNode(Node):
        def __init__(self):
            super().__init__('docking_controller')
            self.declare_parameter('idle_timeout', 30.0)
            self.declare_parameter('low_battery_threshold', 20.0)
            self.declare_parameter('resume_battery_threshold', 80.0)
            self.declare_parameter('start_docked', True)
            self.declare_parameter('use_docking_server', True)
            self.declare_parameter('dock_id', 'home_dock')
            self.declare_parameter('dock_type', 'simple_charging_dock')
            self.declare_parameter('staging_offset', 0.9)
            self.declare_parameter('waypoints_file', default_waypoints_file())
            self.declare_parameter('task_status_timeout', 5.0)

            p = self.get_parameter
            self.logic = DockingLogic(p('idle_timeout').value,
                                      p('low_battery_threshold').value,
                                      p('resume_battery_threshold').value,
                                      p('start_docked').value)
            self.use_docking_server = p('use_docking_server').value
            self.dock_id = p('dock_id').value
            self.dock_type = p('dock_type').value
            self.staging_offset = p('staging_offset').value
            self.task_status_timeout = p('task_status_timeout').value
            self.book = WaypointBook.from_yaml(p('waypoints_file').value)
            self.dock_wp = self.book.get('dock')
            if self.dock_wp is None:
                raise RuntimeError("waypoints file has no 'dock' location")

            self.dock_client = ActionClient(self, DockRobot, 'dock_robot')
            self.undock_client = ActionClient(self, UndockRobot, 'undock_robot')
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.dock_goal = None

            self.task_idle = True
            self.queue_empty = True
            self.task_stamp = None

            self.status_pub = self.create_publisher(String, '/dock/status', 10)
            self.create_subscription(String, '/dock/command', self.on_command, 10)
            self.create_subscription(String, '/delivery/status', self.on_task_status, 10)
            self.create_subscription(BatteryState, '/battery_state', self.on_battery, 10)
            self.create_timer(0.5, self.on_timer)
            self.get_logger().info(
                f'Docking controller ready (state {self.logic.state}, '
                f'idle timeout {self.logic.idle_timeout:.0f}s, dock at '
                f'({self.dock_wp.x:.2f}, {self.dock_wp.y:.2f}))')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        # ---- inputs
        def on_command(self, msg):
            if self.logic.command(msg.data):
                self.get_logger().info(f"Dock command '{msg.data.strip()}'")
            else:
                self.get_logger().warn(f"Unknown dock command '{msg.data}' (dock|undock|resume)")

        def on_task_status(self, msg):
            try:
                data = json.loads(msg.data)
            except ValueError:
                return
            self.task_idle = data.get('state') == 'IDLE'
            self.queue_empty = data.get('queue_length', 0) == 0
            self.task_stamp = self.now()

        def on_battery(self, msg):
            pct = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage
            if math.isnan(pct):
                return
            was_hold = self.logic.battery_hold
            self.logic.update_battery(pct)
            if self.logic.battery_hold and not was_hold:
                self.get_logger().warn(f'Low battery ({pct:.0f}%): returning to dock after current order')
            elif was_hold and not self.logic.battery_hold:
                self.get_logger().info(f'Battery recharged ({pct:.0f}%): deliveries resume')

        # ---- main loop
        def on_timer(self):
            if self.task_stamp is None or self.now() - self.task_stamp > self.task_status_timeout:
                task_idle, queue_empty = True, True  # no task manager running
            else:
                task_idle, queue_empty = self.task_idle, self.queue_empty
            for action in self.logic.tick(self.now(), task_idle, queue_empty):
                if action == 'dock':
                    self.start_docking()
                elif action == 'undock':
                    self.start_undocking()
                elif action == 'cancel_dock':
                    self.get_logger().info('New order received: cancelling return to dock')
                    if self.dock_goal is not None:
                        self.dock_goal.cancel_goal_async()
            self.status_pub.publish(String(data=json.dumps(self.logic.status())))

        # ---- docking
        def start_docking(self):
            self.get_logger().info(f'Returning to dock (reason: {self.logic.reason})')
            if self.use_docking_server and self.dock_client.wait_for_server(timeout_sec=2.0):
                goal = DockRobot.Goal()
                goal.use_dock_id = True
                goal.dock_id = self.dock_id
                goal.navigate_to_staging_pose = True
                goal.max_staging_time = 300.0
                self.send(self.dock_client, goal, self.on_dock_done)
            else:
                self.get_logger().warn('Docking server unavailable: navigating straight to the dock pose')
                self.send(self.nav_client, self.nav_goal(self.dock_wp.x, self.dock_wp.y), self.on_dock_done)

        def on_dock_done(self, success, status):
            self.dock_goal = None
            if status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Return to dock cancelled')
            elif success:
                self.get_logger().info('Docked: robot is on the charging station')
            else:
                self.get_logger().error(f'Docking failed (status {status}); will retry')
            self.logic.on_dock_result(success, self.now(),
                                      cancelled=status == GoalStatus.STATUS_CANCELED)

        # ---- undocking
        def start_undocking(self):
            self.get_logger().info('Undocking')
            if self.use_docking_server and self.undock_client.wait_for_server(timeout_sec=2.0):
                goal = UndockRobot.Goal()
                goal.dock_type = self.dock_type
                goal.max_undocking_time = 30.0
                self.send(self.undock_client, goal, self.on_undock_done)
            else:
                yaw = self.dock_wp.yaw
                x = self.dock_wp.x - self.staging_offset * math.cos(yaw)
                y = self.dock_wp.y - self.staging_offset * math.sin(yaw)
                self.send(self.nav_client, self.nav_goal(x, y, yaw), self.on_undock_done)

        def on_undock_done(self, success, status):
            self.dock_goal = None
            if success:
                self.get_logger().info('Undocked')
            else:
                self.get_logger().error(f'Undocking failed (status {status})')
            self.logic.on_undock_result(success, self.now())

        # ---- helpers
        def nav_goal(self, x, y, yaw=None):
            yaw = self.dock_wp.yaw if yaw is None else yaw
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = self.book.frame_id
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
            return goal

        def send(self, client, goal, done_cb):
            def on_response(future):
                handle = future.result()
                if not handle.accepted:
                    done_cb(False, GoalStatus.STATUS_ABORTED)
                    return
                self.dock_goal = handle
                handle.get_result_async().add_done_callback(on_result)

            def on_result(future):
                res = future.result()
                ok = res.status == GoalStatus.STATUS_SUCCEEDED
                if ok and hasattr(res.result, 'success'):
                    ok = bool(res.result.success)
                done_cb(ok, res.status)

            client.send_goal_async(goal).add_done_callback(on_response)

    rclpy.init(args=args)
    node = DockingControllerNode()
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
