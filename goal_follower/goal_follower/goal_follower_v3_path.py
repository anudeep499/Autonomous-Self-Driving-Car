#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class GoalToNav2(Node):
    """
    Subscribes: /goal_pose (PoseStamped)
    Sends: NavigateToPose action request to Nav2
    """
    def __init__(self):
        super().__init__("goal_to_nav2")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")

        goal_topic = self.get_parameter("goal_topic").value
        action_name = self.get_parameter("nav2_action_name").value

        self.goal_sub = self.create_subscription(
            PoseStamped, goal_topic, self.goal_cb, 10
        )

        self.nav_client = ActionClient(self, NavigateToPose, action_name)

        self.get_logger().info(
            f"GoalToNav2 started.\n"
            f"  goal_topic: {goal_topic}\n"
            f"  nav2 action: {action_name}"
        )

    def goal_cb(self, goal_msg: PoseStamped):
        # wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 NavigateToPose action server not available yet.")
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_msg

        self.get_logger().info(
            f"Sending goal to Nav2: x={goal.pose.pose.position.x:.2f}, "
            f"y={goal.pose.pose.position.y:.2f}, frame={goal.pose.header.frame_id}"
        )

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the goal.")
            return

        self.get_logger().info("Nav2 accepted the goal.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        # status values are action statuses; 4 usually = SUCCEEDED in ROS2 action enums
        self.get_logger().info(f"Nav2 finished with status: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalToNav2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
