from nav_msgs.msg import Path

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Frames / topics
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Control gains and tolerances (reuse what you already have)
        self.declare_parameter('k_theta', 3.0)
        self.declare_parameter('k_v', 0.2)
        self.declare_parameter('max_steer', 2.0)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('max_heading_error_deg', 155.0)

        self.k_theta = self.get_parameter('k_theta').value
        self.k_v = self.get_parameter('k_v').value
        self.max_steer = self.get_parameter('max_steer').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_heading_error = math.radians(
            self.get_parameter('max_heading_error_deg').value
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Path state
        self.current_path: list[PoseStamped] = []
        self.current_index: int = 0

        self.path_sub = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f"PathFollower started. path_topic={path_topic}, "
            f"cmd_vel_topic={cmd_vel_topic}, global_frame={self.global_frame}, "
            f"base_frame={self.base_frame}"
        )

    def path_callback(self, msg: Path):
        # Replace current path with new one
        self.current_path = list(msg.poses)
        self.current_index = 0
        n = len(self.current_path)
        self.get_logger().info(f"Received new path with {n} poses.")

    def control_loop(self):
        twist = Twist()

        if not self.current_path or self.current_index >= len(self.current_path):
            # No active path -> stop
            self.cmd_pub.publish(twist)
            return

        # Get robot pose from TF (map -> base_link)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {self.global_frame} -> {self.base_frame}: {ex}"
            )
            self.cmd_pub.publish(twist)
            return

        # Robot pose
        t = transform.transform.translation
        x = t.x
        y = t.y

        q = transform.transform.rotation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)

        # Current waypoint on the path
        wp = self.current_path[self.current_index].pose
        gx = wp.position.x
        gy = wp.position.y

        dx = gx - x
        dy = gy - y
        distance = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        heading_error = normalize_angle(target_angle - yaw)

        # Skip waypoint if close enough, move to the next one
        if distance < self.goal_tolerance:
            self.current_index += 1
            if self.current_index >= len(self.current_path):
                self.get_logger().info("Path complete, stopping.")
            else:
                self.get_logger().info(
                    f"Reached waypoint {self.current_index}, "
                    f"{len(self.current_path) - self.current_index} left."
                )
            self.cmd_pub.publish(twist)
            return

        # If goal/waypoint is mostly behind, don't move
        if abs(heading_error) > self.max_heading_error:
            self.cmd_pub.publish(twist)
            self.get_logger().info(
                f"Heading error {heading_error:.2f} rad exceeds limit "
                f"{self.max_heading_error:.2f} rad. Not moving."
            )
            return

        # Steering
        steer_cmd = self.k_theta * heading_error
        steer_cmd = max(-self.max_steer, min(self.max_steer, steer_cmd))

        # Speed
        v_cmd = self.k_v * distance
        v_cmd = min(self.max_speed, v_cmd)
        if distance > 2.0 * self.goal_tolerance and v_cmd < self.min_speed:
            v_cmd = self.min_speed

        twist.linear.x = v_cmd
        twist.angular.z = steer_cmd
        self.cmd_pub.publish(twist)
