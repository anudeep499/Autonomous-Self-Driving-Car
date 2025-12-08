#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener, TransformException


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # ----- Parameters -----
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('cmd_vel_topic', '/rover_vel')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # Control parameters (same style / defaults as SimpleGoalFollower)
        self.declare_parameter('k_theta', 1.4)        # steering gain
        self.declare_parameter('k_v', 0.2)            # linear speed gain (final waypoint only)
        self.declare_parameter('max_steer', 1.6)      # max |angular.z| (steering command)
        self.declare_parameter('max_speed', 0.22)      # cruise speed along path
        self.declare_parameter('min_speed', 0.05)     # minimum nonzero speed (final waypoint)
        self.declare_parameter('goal_tolerance', 2.5)  # [m], distance to waypoint to consider it reached
        self.declare_parameter('max_heading_error_deg', 155.0)  # [deg], beyond this we refuse to move

        # New: spacing between used waypoints (downsample path)
        self.declare_parameter('waypoint_spacing', 0.1)  # [m]

        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.k_theta = self.get_parameter('k_theta').value
        self.k_v = self.get_parameter('k_v').value
        self.max_steer = self.get_parameter('max_steer').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        max_heading_error_deg = self.get_parameter('max_heading_error_deg').value
        self.max_heading_error = math.radians(max_heading_error_deg)

        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value

        # ----- State -----
        self.current_path: list[PoseStamped] = []
        self.current_index: int = 0

        # ----- TF Buffer + Listener -----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----- Interfaces -----
        self.path_sub = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Control loop timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f"PathFollower started.\n"
            f"  path_topic: {path_topic}\n"
            f"  global_frame: {self.global_frame}\n"
            f"  base_frame: {self.base_frame}\n"
            f"  cmd_vel_topic: {cmd_vel_topic}\n"
            f"  waypoint_spacing: {self.waypoint_spacing:.2f} m"
        )

    # ----- Callbacks -----

    def path_callback(self, msg: Path):
        """
        Receive a new path and downsample it based on waypoint_spacing.
        Always keep the first and last poses.

        IMPORTANT: If we're already following a path and the new path ends
        at (almost) the same goal, ignore the new plan so we don't reset.
        """
        n_raw = len(msg.poses)
        if n_raw == 0:
            self.get_logger().warn("Received empty path.")
            return

        # If we *already* have a path and haven't finished it yet, check if
        # the new path's final goal is essentially the same as the current one.
        if self.current_path and self.current_index < len(self.current_path):
            old_goal = self.current_path[-1].pose.position
            new_goal = msg.poses[-1].pose.position

            dx = new_goal.x - old_goal.x
            dy = new_goal.y - old_goal.y
            goal_dist = math.hypot(dx, dy)

            # If the goal hasn't moved much, treat this as a replanning update
            # and keep following the existing path.
            if goal_dist < self.goal_tolerance:
                # Optional: make this debug if it’s too spammy
                self.get_logger().debug(
                    f"Ignoring replanned path (goal moved only {goal_dist:.3f} m)."
                )
                return

        # Otherwise, accept this as a new path and reset our state
        self.current_path = []
        self.current_index = 0

        spacing = max(self.waypoint_spacing, 0.01)  # avoid degenerate zero spacing

        last_x = None
        last_y = None

        for i, ps in enumerate(msg.poses):
            x = ps.pose.position.x
            y = ps.pose.position.y

            if last_x is None:
                # Always keep the first pose
                self.current_path.append(ps)
                last_x, last_y = x, y
                continue

            dist = math.hypot(x - last_x, y - last_y)

            # Keep pose if far enough from last kept, OR if it's the final pose
            if dist >= spacing or i == n_raw - 1:
                self.current_path.append(ps)
                last_x, last_y = x, y

        n_ds = len(self.current_path)
        self.get_logger().info(
            f"Received new path with {n_raw} poses, "
            f"downsampled to {n_ds} waypoints (spacing ~ {spacing:.2f} m)."
        )

    # ----- Control Logic -----

    def control_loop(self):
        """
        Periodically compute cmd_vel based on current pose (via TF) and current waypoint in path.
        """
        twist = Twist()
        


        # No active path or finished path -> stop
        if not self.current_path or self.current_index >= len(self.current_path):
            self.cmd_pub.publish(twist)
            return

        # ---- Get current pose from TF: global_frame -> base_frame ----
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()  # latest available
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {self.global_frame} -> {self.base_frame}: {ex}"
            )
            self.cmd_pub.publish(twist)
            return

        # Current position from transform
        t = transform.transform.translation
        x = t.x
        y = t.y

        # Current yaw from transform rotation
        q = transform.transform.rotation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)

        # ---- Current waypoint (assume path is in global_frame, e.g. map) ----
        wp_pose = self.current_path[self.current_index].pose
        gx = wp_pose.position.x
        gy = wp_pose.position.y

        # Vector to waypoint
        dx = gx - x
        dy = gy - y
        distance = math.hypot(dx, dy)

        # Angle of line from robot to waypoint
        target_angle = math.atan2(dy, dx)

        # Heading error
        heading_error = normalize_angle(target_angle - yaw)

        # Check if this is the final waypoint
        is_last_waypoint = (self.current_index == len(self.current_path) - 1)

        # If we're close enough to this waypoint, move to the next
        if distance < self.goal_tolerance:
            
            print(f"distance: {distance} m")
            self.current_index += 1

            if self.current_index >= len(self.current_path):
                print("Path complete")
                self.get_logger().info("Path complete, stopping.")
                self.cmd_pub.publish(twist)
                return
            else:
                self.get_logger().info(
                    f"Reached waypoint {self.current_index}/{len(self.current_path)}. "
                    f"Continuing to next."
                )
                ## self.cmd_pub.publish(twist)
                print(f"goal reached, speed: {twist.linear.x}")
                return

        # If the waypoint is mostly behind the robot, don't attempt to go towards it.
        if abs(heading_error) > self.max_heading_error:
            print("heading error is too large, setting twist to 0")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

            self.get_logger().info(
                f"Heading error {heading_error:.2f} rad exceeds limit "
                f"{self.max_heading_error:.2f} rad. Not moving toward waypoint."
            )
            return

        # ----- Steering (angular.z used as steering command) -----
        steer_cmd = self.k_theta * heading_error
        steer_cmd = max(-self.max_steer, min(self.max_steer, steer_cmd))

        # ----- Speed (linear.x as 'voltage' / speed intent) -----
        if is_last_waypoint:
            print("Final waypoint speed control")
            # Slowdown behavior only near the final waypoint
            v_cmd = self.k_v * distance
            v_cmd = min(self.max_speed, v_cmd)

            # Enforce a minimum non-zero speed when still not very close
            if distance > 2.0 * self.goal_tolerance and v_cmd < self.min_speed:
                v_cmd = self.min_speed
        else:
            # Cruise at constant speed for intermediate waypoints
            v_cmd = self.max_speed

        twist.linear.x = v_cmd
        twist.angular.z = steer_cmd
        
        ### print(f"Test 4 for twist: {twist}")

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
