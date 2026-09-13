"""ROS 2 Task Manager Node for workload publishing and lifecycle management."""

import os
from typing import Dict, Optional

from amr_fleet_core.task_model import (
    InvalidTaskTransitionError,
    Task,
    TaskEvent,
    TaskLifecycleState,
)
from amr_fleet_core.workload import WorkloadManager
from amr_fleet_msgs.msg import TaskDefinition, TaskEvent as TaskEventMsg, TaskList
from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node


def float_to_builtin_time(sec_float: Optional[float]) -> BuiltinTime:
    """Convert float seconds to builtin_interfaces/Time msg."""
    msg = BuiltinTime()
    if sec_float is None or sec_float <= 0.0:
        msg.sec = 0
        msg.nanosec = 0
    else:
        msg.sec = int(sec_float)
        msg.nanosec = int((sec_float - int(sec_float)) * 1e9)
    return msg


class TaskManagerNode(Node):
    """ROS 2 Node orchestrating task pools, publishing state, and logging events."""

    def __init__(self) -> None:
        super().__init__('amr_task_manager')

        # Declare parameters
        self.declare_parameter('workload_file', '')
        self.declare_parameter('publish_rate', 1.0)

        workload_file = self.get_parameter('workload_file').get_parameter_value().string_value
        pub_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Locate default workload if not specified
        if not workload_file:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            ws_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
            candidate = os.path.join(
                ws_root, 'config', 'workloads', 'workload_small_deterministic.yaml'
            )
            if os.path.isfile(candidate):
                workload_file = candidate

        self.tasks: Dict[str, Task] = {}

        if workload_file and os.path.isfile(workload_file):
            loaded = WorkloadManager.load_from_yaml(workload_file)
            self.tasks = {t.task_id: t for t in loaded}
            self.get_logger().info(
                f'Loaded {len(self.tasks)} tasks from workload: {workload_file}'
            )
        else:
            self.get_logger().warn(
                f"No workload file found at '{workload_file}'. Starting with empty pool."
            )

        # Publishers
        self.pub_all_tasks = self.create_publisher(TaskList, '/tasks/all', 10)
        self.pub_available_tasks = self.create_publisher(TaskList, '/tasks/available', 10)
        self.pub_events = self.create_publisher(TaskEventMsg, '/tasks/events', 20)

        # Subscriber for state transitions
        self.sub_status_update = self.create_subscription(
            TaskEventMsg,
            '/tasks/update_status',
            self._handle_status_update,
            10,
        )

        # Periodic publication timer
        timer_period = 1.0 / max(0.1, pub_rate)
        self.timer = self.create_timer(timer_period, self._publish_task_lists)

        self.get_logger().info('Task Manager Node initialized.')

    def task_to_msg(self, task: Task) -> TaskDefinition:
        """Convert a domain Task object to a ROS 2 TaskDefinition msg."""
        msg = TaskDefinition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task.task_id

        msg.pickup_pose = Point()
        msg.pickup_pose.x = float(task.pickup[0])
        msg.pickup_pose.y = float(task.pickup[1])
        msg.pickup_pose.z = 0.0

        msg.dropoff_pose = Point()
        msg.dropoff_pose.x = float(task.dropoff[0])
        msg.dropoff_pose.y = float(task.dropoff[1])
        msg.dropoff_pose.z = 0.0

        msg.priority = int(task.priority)
        msg.created_at = float_to_builtin_time(task.created_at)
        msg.deadline = float_to_builtin_time(task.deadline)
        msg.status = task.state.value
        msg.assigned_robot_id = task.assigned_robot_id or ''
        return msg

    def event_to_msg(self, event: TaskEvent, task_id: str) -> TaskEventMsg:
        """Convert a domain TaskEvent to a ROS 2 TaskEvent msg."""
        msg = TaskEventMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.event_type = event.event_type
        msg.previous_state = event.from_state.value if event.from_state else ''
        msg.new_state = event.to_state.value
        msg.robot_id = event.robot_id or ''
        msg.timestamp = float_to_builtin_time(event.timestamp)
        msg.details = event.details
        return msg

    def _publish_task_lists(self) -> None:
        now_msg = self.get_clock().now().to_msg()

        # All tasks
        all_msg = TaskList()
        all_msg.header.stamp = now_msg
        all_msg.tasks = [self.task_to_msg(t) for t in self.tasks.values()]
        self.pub_all_tasks.publish(all_msg)

        # Available (Pending) tasks
        avail_msg = TaskList()
        avail_msg.header.stamp = now_msg
        avail_msg.tasks = [
            self.task_to_msg(t) for t in self.tasks.values()
            if t.state == TaskLifecycleState.PENDING
        ]
        self.pub_available_tasks.publish(avail_msg)

    def _handle_status_update(self, msg: TaskEventMsg) -> None:
        """Handle incoming status update transition request."""
        task_id = msg.task_id
        if task_id not in self.tasks:
            self.get_logger().warn(f"Received status update for unknown task '{task_id}'")
            return

        task = self.tasks[task_id]
        try:
            target_state = TaskLifecycleState.from_str(msg.new_state)
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            task.transition_to(
                target_state,
                timestamp=now_sec,
                robot_id=msg.robot_id or None,
                details=msg.details or 'Updated via ROS 2 interface',
            )

            # Publish the confirmed event
            latest_event = task.events[-1]
            event_msg = self.event_to_msg(latest_event, task_id)
            self.pub_events.publish(event_msg)
            self.get_logger().info(f"Task '{task_id}' transitioned to {target_state.value}")

        except InvalidTaskTransitionError as e:
            self.get_logger().error(f"Rejected transition for task '{task_id}': {e}")
        except ValueError as e:
            self.get_logger().error(f"Malformed state name for task '{task_id}': {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
