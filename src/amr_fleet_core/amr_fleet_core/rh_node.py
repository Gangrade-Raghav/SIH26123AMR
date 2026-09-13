"""ROS 2 Node executing Rolling-Horizon Task Planning per AMR."""

import math
from typing import Any, Dict, List, Tuple

from amr_fleet_core.rh_planner import PlanningResponseData, RHConfig, RollingHorizonPlanner
from amr_fleet_msgs.msg import (
    PlanningRequest,
    PlanningResponse,
    RobotBundle,
    RollingHorizonPlan,
    TaskEvent as TaskEventMsg,
    TaskList,
)
from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def float_to_builtin_time(secs: float) -> BuiltinTime:
    """Convert float seconds to builtin_interfaces/Time msg."""
    b_time = BuiltinTime()
    b_time.sec = int(secs)
    b_time.nanosec = int((secs - int(secs)) * 1e9)
    return b_time


class RollingHorizonPlannerNode(Node):
    """
    Decentralized Rolling-Horizon Task Planning Node.

    Translates CBBA assigned bundles into sequenced sub-goals, single-agent A*
    rolling-horizon paths, execution windows, and trajectory commands.
    """

    def __init__(self) -> None:
        """Initialize RollingHorizonPlannerNode, pubs/subs, and timers."""
        super().__init__('rh_node')

        # Parameters
        self.declare_parameter('robot_id', 'amr_0')
        self.declare_parameter('horizon_steps', 10)
        self.declare_parameter('execution_window', 4)
        self.declare_parameter('replan_rate', 2.0)
        self.declare_parameter('sequencing_heuristic', 'PRIORITY_FIRST')
        self.declare_parameter('grid_resolution', 0.5)
        self.declare_parameter('goal_tolerance_m', 0.5)
        self.declare_parameter('enable_motion_execution', True)
        self.declare_parameter('spawn_x', 2.0)
        self.declare_parameter('spawn_y', 2.0)

        self.robot_id = self.get_parameter('robot_id').value
        h_steps = self.get_parameter('horizon_steps').value
        w_window = self.get_parameter('execution_window').value
        replan_rate = self.get_parameter('replan_rate').value
        heuristic = self.get_parameter('sequencing_heuristic').value
        res = self.get_parameter('grid_resolution').value
        tol = self.get_parameter('goal_tolerance_m').value
        self.enable_motion = self.get_parameter('enable_motion_execution').value

        try:
            r_idx = int(self.robot_id.split('_')[-1])
            default_y = 2.0 + r_idx * 3.0
        except Exception:
            default_y = 2.0

        spawn_x_val = float(self.get_parameter('spawn_x').value)
        spawn_y_val = float(self.get_parameter('spawn_y').value)
        if spawn_y_val == 2.0 and default_y != 2.0:
            spawn_y_val = default_y

        self.spawn_x = spawn_x_val
        self.spawn_y = spawn_y_val

        config = RHConfig(
            horizon_steps=h_steps,
            execution_window=w_window,
            replan_rate=replan_rate,
            sequencing_heuristic=heuristic,
            grid_resolution=res,
            goal_tolerance_m=tol,
        )

        self.planner = RollingHorizonPlanner(robot_id=self.robot_id, config=config)
        self.planner.update_position((self.spawn_x, self.spawn_y))
        self.cached_tasks: Dict[str, Dict[str, Any]] = {}
        self.cached_bundle: List[str] = []
        self.current_yaw: float = 0.0

        # Publishers
        self.pub_rolling_plan = self.create_publisher(
            RollingHorizonPlan, f'/{self.robot_id}/rolling_plan', 10,
        )
        self.pub_plan_request = self.create_publisher(
            PlanningRequest, f'/{self.robot_id}/planning_request', 10,
        )
        self.pub_plan_response = self.create_publisher(
            PlanningResponse, f'/{self.robot_id}/planning_response', 10,
        )
        self.pub_cmd_vel = self.create_publisher(
            Twist, f'/{self.robot_id}/cmd_vel', 10,
        )
        self.pub_task_status = self.create_publisher(
            TaskEventMsg, '/tasks/update_status', 20,
        )

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry,
            f'/{self.robot_id}/odom',
            self._handle_odom,
            10,
        )
        self.sub_bundle = self.create_subscription(
            RobotBundle,
            f'/{self.robot_id}/bundle',
            self._handle_bundle,
            10,
        )
        self.sub_tasks = self.create_subscription(
            TaskList,
            '/tasks/all',
            self._handle_tasks_all,
            10,
        )

        self.obstacle_ahead: bool = False
        self.sub_scan = self.create_subscription(
            LaserScan,
            f'/{self.robot_id}/scan',
            self._handle_scan,
            qos_profile_sensor_data,
        )

        # Timer
        period = 1.0 / max(0.1, replan_rate)
        self.timer = self.create_timer(period, self._planning_cycle)

        self.get_logger().info(
            f'Rolling-Horizon Planner initialized for {self.robot_id} '
            f'(h={h_steps}, w={w_window}, heuristic={heuristic})',
        )

    def _handle_odom(self, msg: Odometry) -> None:
        """Update robot localized position and yaw orientation."""
        pos = (
            round(self.spawn_x + msg.pose.pose.position.x, 3),
            round(self.spawn_y + msg.pose.pose.position.y, 3),
        )
        self.planner.update_position(pos)

        # Quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _handle_bundle(self, msg: RobotBundle) -> None:
        """Update assigned bundle from CBBA."""
        incoming_bundle = list(msg.task_ids)
        if incoming_bundle != self.cached_bundle:
            self.cached_bundle = incoming_bundle
            if self.cached_tasks:
                self.planner.update_assigned_bundle(self.cached_bundle, self.cached_tasks)

    def _handle_tasks_all(self, msg: TaskList) -> None:
        """Cache all task metadata from Task Manager."""
        tasks_map = {}
        for td in msg.tasks:
            tasks_map[td.task_id] = {
                'task_id': td.task_id,
                'pickup': (td.pickup_pose.x, td.pickup_pose.y),
                'dropoff': (td.dropoff_pose.x, td.dropoff_pose.y),
                'priority': td.priority,
                'status': td.status,
                'deadline': (
                    td.deadline.sec + td.deadline.nanosec * 1e-9
                    if td.deadline.sec > 0 else None
                ),
            }
        self.cached_tasks = tasks_map
        if self.cached_bundle:
            self.planner.update_assigned_bundle(self.cached_bundle, self.cached_tasks)

    def _handle_scan(self, msg: LaserScan) -> None:
        """Process LiDAR to detect obstacles in forward arc."""
        num_rays = len(msg.ranges)
        if num_rays == 0:
            return

        arc_rays = max(1, int(num_rays * (25.0 / 360.0)))
        forward_ranges = []
        for idx in range(-arc_rays, arc_rays + 1):
            r = msg.ranges[idx]
            if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r):
                forward_ranges.append(r)

        self.obstacle_ahead = bool(forward_ranges and min(forward_ranges) < 0.65)

    def _publish_task_transition(
        self,
        task_id: str,
        new_state: str,
        details: str = '',
    ) -> None:
        """Emit task lifecycle state transition event to M3 TaskManager."""
        msg = TaskEventMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.event_type = f'TRANSITION_TO_{new_state}'
        msg.new_state = new_state
        msg.robot_id = self.robot_id
        msg.timestamp = float_to_builtin_time(self.get_clock().now().nanoseconds * 1e-9)
        msg.details = details
        self.pub_task_status.publish(msg)

    def _publish_plan_request(
        self,
        task_id: str,
        sub_goal_type: str,
        goal: Tuple[float, float],
    ) -> None:
        """Publish planning request abstraction message."""
        msg = PlanningRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.start_pose.x = float(self.planner.current_position[0])
        msg.start_pose.y = float(self.planner.current_position[1])
        msg.goal_pose.x = float(goal[0])
        msg.goal_pose.y = float(goal[1])
        msg.horizon_steps = self.planner.config.horizon_steps
        msg.execution_window = self.planner.config.execution_window
        msg.task_id = task_id
        msg.sub_goal_type = sub_goal_type
        self.pub_plan_request.publish(msg)

    def _publish_plan_response(self, resp: PlanningResponseData) -> None:
        """Publish planning response abstraction message."""
        msg = PlanningResponse()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.task_id = resp.task_id
        msg.sub_goal_type = resp.sub_goal_type
        msg.total_cost = float(resp.total_cost)
        msg.planning_latency_ms = float(resp.planning_latency_ms)
        msg.success = bool(resp.success)
        msg.status_message = resp.status_message
        msg.horizon_steps = resp.horizon_steps
        msg.execution_window = resp.execution_window
        msg.replan_count = resp.replan_count

        for pt in resp.full_path:
            p = Point()
            p.x, p.y = float(pt[0]), float(pt[1])
            msg.full_path.append(p)

        for pt in resp.horizon_path:
            p = Point()
            p.x, p.y = float(pt[0]), float(pt[1])
            msg.horizon_path.append(p)

        for pt in resp.execution_path:
            p = Point()
            p.x, p.y = float(pt[0]), float(pt[1])
            msg.execution_path.append(p)

        self.pub_plan_response.publish(msg)

    def _publish_rolling_plan(self) -> None:
        """Publish aggregated rolling-horizon plan state."""
        msg = RollingHorizonPlan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.assigned_bundle = list(self.planner.assigned_bundle)

        curr_tid = (
            self.planner.ordered_tasks[self.planner.active_task_idx]
            if self.planner.active_task_idx < len(self.planner.ordered_tasks)
            else ''
        )
        msg.current_task_id = curr_tid
        msg.current_phase = self.planner.active_phase

        if self.planner.current_goal:
            msg.current_goal.x = float(self.planner.current_goal[0])
            msg.current_goal.y = float(self.planner.current_goal[1])

        for pt in self.planner.horizon_path:
            p = Point()
            p.x, p.y = float(pt[0]), float(pt[1])
            msg.horizon_path.append(p)

        for pt in self.planner.execution_path:
            p = Point()
            p.x, p.y = float(pt[0]), float(pt[1])
            msg.execution_path.append(p)

        msg.horizon_steps = self.planner.config.horizon_steps
        msg.execution_window = self.planner.config.execution_window
        msg.replan_count = self.planner.replan_count
        msg.planning_latency_ms = float(self.planner.last_latency_ms)
        msg.is_valid = bool(self.planner.last_plan_success)

        self.pub_rolling_plan.publish(msg)

    def _planning_cycle(self) -> None:
        """Periodic rolling-horizon planning, replanning check, and execution control."""
        if not self.planner.assigned_bundle and self.cached_bundle and self.cached_tasks:
            self.planner.update_assigned_bundle(self.cached_bundle, self.cached_tasks)

        # Check sub-goal arrival
        if self.planner.check_subgoal_arrival():
            curr_tid = (
                self.planner.ordered_tasks[self.planner.active_task_idx]
                if self.planner.active_task_idx < len(self.planner.ordered_tasks)
                else ''
            )
            event = self.planner.advance_subgoal()

            if event == 'PICKUP_REACHED':
                self.get_logger().info(
                    f"[{self.robot_id}] Reached pickup for task '{curr_tid}'. Moving to dropoff.",
                )
                self._publish_task_transition(
                    curr_tid, 'IN_PROGRESS', 'Arrived at pickup location',
                )
            elif event == 'TASK_COMPLETED':
                self.get_logger().info(
                    f"[{self.robot_id}] Completed task '{curr_tid}'. Advancing to next task.",
                )
                self._publish_task_transition(
                    curr_tid, 'COMPLETED', 'Delivered at dropoff location',
                )
            elif event == 'ALL_COMPLETED':
                self.get_logger().info(
                    f'[{self.robot_id}] All assigned bundle tasks completed!',
                )

        # Check replanning triggers
        if self.planner.check_replan_triggers() and self.planner.current_goal:
            curr_tid = (
                self.planner.ordered_tasks[self.planner.active_task_idx]
                if self.planner.active_task_idx < len(self.planner.ordered_tasks)
                else ''
            )
            self._publish_plan_request(
                curr_tid, self.planner.active_phase, self.planner.current_goal,
            )
            resp = self.planner.replan()
            self._publish_plan_response(resp)

        # Publish current plan status
        self._publish_rolling_plan()

        # Execute motion towards execution window waypoint
        if (self.enable_motion and self.planner.active_phase != 'IDLE'
                and self.planner.execution_path):
            self._execute_motion_step()

    def _execute_motion_step(self) -> None:
        """Drive robot towards immediate waypoint in execution window."""
        waypoint = self.planner.advance_execution_step()
        if not waypoint:
            return

        rx, ry = self.planner.current_position
        tx, ty = waypoint

        if self.obstacle_ahead:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd_vel.publish(cmd)
            return

        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)

        if dist < 0.15:
            # Reached intermediate waypoint
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = target_yaw - self.current_yaw

        # Normalize yaw error to [-pi, pi]
        while yaw_err > math.pi:
            yaw_err -= 2.0 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2.0 * math.pi

        cmd = Twist()
        if abs(yaw_err) > 0.4:
            # Rotate in place towards waypoint
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8 if yaw_err > 0 else -0.8
        else:
            # Drive forward with proportional turning
            cmd.linear.x = min(0.35, 0.6 * dist)
            cmd.angular.z = 1.2 * yaw_err

        self.pub_cmd_vel.publish(cmd)


def main(args=None):
    """Run the Rolling-Horizon Planner Node."""
    rclpy.init(args=args)
    node = RollingHorizonPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
