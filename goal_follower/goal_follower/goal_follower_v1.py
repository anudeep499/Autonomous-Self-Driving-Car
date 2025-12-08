#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from tf_transformations import euler_from_quaternion


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class SimpleGoalFollower(Node):
    def __init__(self):
        super().__init__('simple_goal_follower')

        # ----- Parameters -----
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Control parameters
        self.declare_parameter('k_theta', 3.0)        # steering gain
        self.declare_parameter('k_v', 0.2)            # linear speed gain
        self.declare_parameter('max_steer', 2.0)      # max |angular.z| (steering command)
        self.declare_parameter('max_speed', 0.3)      # max linear.x
        self.declare_parameter('min_speed', 0.05)     # minimum nonzero speed
        self.declare_parameter('goal_tolerance', 0.2)  # [m], distance to goal to stop
        self.declare_parameter('max_heading_error_deg', 155.0)  # [deg], beyond this we refuse to move

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.k_theta = self.get_parameter('k_theta').value
        self.k_v = self.get_parameter('k_v').value
        self.max_steer = self.get_parameter('max_steer').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        max_heading_error_deg = self.get_parameter('max_heading_error_deg').value
        self.max_heading_error = math.radians(max_heading_error_deg)
        
        # ----- State -----
        self.current_pose = None     # PoseWithCovarianceStamped
        self.goal_pose = None        # PoseStamped

        # ----- Interfaces -----
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self.pose_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            goal_topic,
            self.goal_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Control loop timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f"SimpleGoalFollower started.\n"
            f"  goal_topic: {goal_topic}\n"
            f"  pose_topic: {pose_topic}\n"
            f"  cmd_vel_topic: {cmd_vel_topic}"
        )

    # ----- Callbacks -----

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.get_logger().info(
            f"Received goal at x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f} in frame {msg.header.frame_id}"
        )

    # ----- Control Logic -----

    def control_loop(self):
        """
        Periodically compute cmd_vel based on current pose and goal.
        """
        twist = Twist()

        if self.current_pose is None or self.goal_pose is None:
            # No data -> publish zero command
            self.cmd_pub.publish(twist)
            return

        # Frames: we assume both are in the same frame (map)
        cp = self.current_pose.pose.pose
        gp = self.goal_pose.pose

        # Current position
        x = cp.position.x
        y = cp.position.y

        # Current yaw from quaternion
        q = cp.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)

        # Goal position
        gx = gp.position.x
        gy = gp.position.y

        # Vector to goal
        dx = gx - x
        dy = gy - y
        distance = math.hypot(dx, dy)

        # Angle of line from robot to goal
        target_angle = math.atan2(dy, dx)

        # Heading error
        heading_error = normalize_angle(target_angle - yaw)

        # If the goal is mostly behind the robot, don't attempt to go towards it.
        if abs(heading_error) > self.max_heading_error:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

            # Don't clear the goal: user can rotate the robot manually or move it,
            # and once the heading error is smaller again, it will start moving.
            # You can uncomment this if you want a log:
            self.get_logger().info(
                f"Heading error {heading_error:.2f} rad exceeds limit "
                f"{self.max_heading_error:.2f} rad. Not moving toward goal."
            )
            return

        # If we're close enough, stop and clear goal
        if distance < self.goal_tolerance:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            # Optionally: keep goal, but we can log once
            self.get_logger().info(
                f"Goal reached (dist={distance:.3f} < {self.goal_tolerance}). "
                "Waiting for new goal."
            )
            self.goal_pose = None
            return

        # ----- Steering (angular.z used as steering command) -----
        steer_cmd = self.k_theta * heading_error
        steer_cmd = max(-self.max_steer, min(self.max_steer, steer_cmd))

        # ----- Speed (linear.x as 'voltage' / speed intent) -----
        # Simple proportional law that slows down near the goal
        v_cmd = self.k_v * distance
        v_cmd = min(self.max_speed, v_cmd)

        # Enforce a minimum non-zero speed when far enough from goal
        if distance > 2.0 * self.goal_tolerance and v_cmd < self.min_speed:
            v_cmd = self.min_speed

        twist.linear.x = v_cmd
        twist.angular.z = steer_cmd

        self.cmd_pub.publish(twist)

        # Some debug output (throttled)
        # self.get_logger().info_throttle(
        #     1000.0,  # ms
        #     f"dist={distance:.2f}, heading_err={heading_error:.2f} rad, "
        #     f"v={v_cmd:.2f}, steer={steer_cmd:.2f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGoalFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
