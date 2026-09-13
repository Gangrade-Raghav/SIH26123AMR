"""Decentralized ROS 2 Node executing CBBA for a single AMR."""

from typing import Any, Dict, Set

from amr_fleet_core.cbba_agent import CBBAAgent, CBBAConfig
from amr_fleet_msgs.msg import (
    CBBABid,
    RobotBundle,
    TaskEvent as TaskEventMsg,
    TaskList,
)
from builtin_interfaces.msg import Time as BuiltinTime
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def float_to_builtin_time(sec_float: float) -> BuiltinTime:
    """Convert float timestamp to builtin_interfaces/Time msg."""
    msg = BuiltinTime()
    if sec_float <= 0.0:
        msg.sec = 0
        msg.nanosec = 0
    else:
        msg.sec = int(sec_float)
        msg.nanosec = int((sec_float - int(sec_float)) * 1e9)
    return msg


class CBBANode(Node):
    """Decentralized ROS 2 node executing CBBA bundle building and consensus."""

    def __init__(self) -> None:
        """Initialize CBBANode parameters, pubs/subs, and timers."""
        super().__init__('cbba_node')

        # Parameters
        self.declare_parameter('robot_id', '')
        self.declare_parameter('max_bundle_size', 4)
        self.declare_parameter('weight_priority', 100.0)
        self.declare_parameter('weight_distance', 10.0)
        self.declare_parameter('weight_late', 5.0)
        self.declare_parameter('discount_factor', 0.95)
        self.declare_parameter('consensus_rate', 5.0)
        self.declare_parameter('stable_rounds_for_convergence', 5)

        robot_id_param = self.get_parameter('robot_id').get_parameter_value().string_value
        if not robot_id_param:
            # Fall back to namespace stripped of leading slashes
            ns = self.get_namespace().strip('/')
            robot_id_param = ns if ns else 'amr_0'
        self.robot_id = robot_id_param

        max_bundle = self.get_parameter('max_bundle_size').get_parameter_value().integer_value
        w_prio = self.get_parameter('weight_priority').get_parameter_value().double_value
        w_dist = self.get_parameter('weight_distance').get_parameter_value().double_value
        w_late = self.get_parameter('weight_late').get_parameter_value().double_value
        discount = self.get_parameter('discount_factor').get_parameter_value().double_value
        rate = self.get_parameter('consensus_rate').get_parameter_value().double_value
        self.stable_thresh = self.get_parameter(
            'stable_rounds_for_convergence'
        ).get_parameter_value().integer_value

        config = CBBAConfig(
            max_bundle_size=int(max_bundle),
            weight_priority=float(w_prio),
            weight_distance=float(w_dist),
            weight_late=float(w_late),
            discount_factor=float(discount),
        )
        self.agent = CBBAAgent(robot_id=self.robot_id, config=config)

        # Internal state
        self.task_pool: Dict[str, Any] = {}
        self.iteration: int = 0
        self.consecutive_stable_rounds: int = 0
        self.is_converged: bool = False
        self.assigned_tasks_committed: Set[str] = set()

        # Publishers
        self.pub_bids = self.create_publisher(CBBABid, '/fleet/cbba_bids', 50)
        self.pub_bundle = self.create_publisher(
            RobotBundle, f'/{self.robot_id}/bundle', 10
        )
        self.pub_task_status = self.create_publisher(
            TaskEventMsg, '/tasks/update_status', 20
        )

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry,
            f'/{self.robot_id}/odom',
            self._handle_odom,
            10,
        )
        self.sub_tasks = self.create_subscription(
            TaskList,
            '/tasks/available',
            self._handle_available_tasks,
            10,
        )
        self.sub_bids = self.create_subscription(
            CBBABid,
            '/fleet/cbba_bids',
            self._handle_peer_bid,
            50,
        )

        # Periodic timer
        timer_period = 1.0 / max(0.1, rate)
        self.timer = self.create_timer(timer_period, self._consensus_cycle)

        self.get_logger().info(
            f'CBBA Node initialized for {self.robot_id} (bundle cap: {max_bundle})'
        )

    def _handle_odom(self, msg: Odometry) -> None:
        """Update agent position from odometry."""
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.agent.update_position(pos)

    def _handle_available_tasks(self, msg: TaskList) -> None:
        """Cache incoming pool of available tasks."""
        current_map: Dict[str, Any] = {}
        for td in msg.tasks:
            t_id = td.task_id
            current_map[t_id] = {
                'task_id': t_id,
                'pickup': (td.pickup_pose.x, td.pickup_pose.y),
                'dropoff': (td.dropoff_pose.x, td.dropoff_pose.y),
                'priority': td.priority,
                'deadline': (
                    td.deadline.sec + td.deadline.nanosec * 1e-9
                    if td.deadline.sec > 0 else None
                ),
            }
        self.task_pool = current_map

    def _handle_peer_bid(self, msg: CBBABid) -> None:
        """Process peer bid vector and resolve bidding conflicts."""
        if msg.robot_id == self.robot_id:
            return

        peer_bids: Dict[str, float] = {}
        peer_robots: Dict[str, str] = {}
        peer_times: Dict[str, float] = {}

        for idx, t_id in enumerate(msg.task_ids):
            if idx < len(msg.winning_bids):
                peer_bids[t_id] = float(msg.winning_bids[idx])
            if idx < len(msg.winning_robots):
                peer_robots[t_id] = str(msg.winning_robots[idx])
            if idx < len(msg.timestamps):
                peer_times[t_id] = float(msg.timestamps[idx])

        changed = self.agent.resolve_conflicts(
            peer_id=msg.robot_id,
            peer_iteration=msg.iteration,
            peer_winning_bids=peer_bids,
            peer_winning_robots=peer_robots,
            peer_timestamps=peer_times,
            task_map=self.task_pool,
        )

        if changed:
            self.consecutive_stable_rounds = 0
            self.is_converged = False

    def _consensus_cycle(self) -> None:
        """Periodic CBBA bundle update and broadcast loop."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.iteration += 1

        # Phase 1: Build bundle if task pool is available
        added = 0
        if self.task_pool:
            added = self.agent.build_bundle(self.task_pool, current_time=now_sec)

        if added > 0:
            self.consecutive_stable_rounds = 0
            self.is_converged = False
        else:
            self.consecutive_stable_rounds += 1

        if (
            self.consecutive_stable_rounds >= self.stable_thresh
            and len(self.agent.state.bundle) > 0
        ):
            self.is_converged = True

        # Phase 2: Broadcast current bid beliefs
        bid_msg = CBBABid()
        bid_msg.header.stamp = self.get_clock().now().to_msg()
        bid_msg.robot_id = self.robot_id
        bid_msg.iteration = self.iteration

        all_task_ids = sorted(self.agent.state.winning_bids.keys())
        bid_msg.task_ids = all_task_ids
        bid_msg.winning_bids = [
            float(self.agent.state.winning_bids[t]) for t in all_task_ids
        ]
        bid_msg.winning_robots = [
            str(self.agent.state.winning_robots.get(t, '')) for t in all_task_ids
        ]
        bid_msg.timestamps = [
            float(self.agent.state.timestamps.get(t, 0.0)) for t in all_task_ids
        ]
        self.pub_bids.publish(bid_msg)

        # Phase 3: Publish bundle status for observability
        bundle_msg = RobotBundle()
        bundle_msg.header.stamp = self.get_clock().now().to_msg()
        bundle_msg.robot_id = self.robot_id
        bundle_msg.task_ids = list(self.agent.state.bundle)
        bundle_msg.bid_values = [
            float(self.agent.state.winning_bids.get(t, 0.0))
            for t in self.agent.state.bundle
        ]
        bundle_msg.is_converged = self.is_converged
        self.pub_bundle.publish(bundle_msg)

        # Phase 4: Commit task transitions to TaskManager if converged
        if self.is_converged:
            for t_id in self.agent.state.bundle:
                if t_id not in self.assigned_tasks_committed:
                    event_msg = TaskEventMsg()
                    event_msg.header.stamp = self.get_clock().now().to_msg()
                    event_msg.task_id = t_id
                    event_msg.event_type = 'ASSIGNED'
                    event_msg.new_state = 'ASSIGNED'
                    event_msg.robot_id = self.robot_id
                    event_msg.timestamp = float_to_builtin_time(now_sec)
                    event_msg.details = 'Allocated via decentralized CBBA consensus'
                    self.pub_task_status.publish(event_msg)
                    self.assigned_tasks_committed.add(t_id)
                    self.get_logger().info(
                        f'Committed task {t_id} assignment to {self.robot_id}'
                    )


def main(args=None) -> None:
    """Run CBBA node."""
    rclpy.init(args=args)
    node = CBBANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
