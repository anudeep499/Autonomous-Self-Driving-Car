#!/usr/bin/env python3

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class ControllerState:
    v_ref_prev: float = 0.0
    yaw_int: float = 0.0
    vel_int: float = 0.0
    delta_prev: float = 0.0


class RoverControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_controller")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("wheelbase", 0.32)
        self.declare_parameter("wheel_radius", 0.05)

        self.declare_parameter("v_max", 2.0)
        self.declare_parameter("v_min", 0.15)
        self.declare_parameter("a_lat_max", 1.5)
        self.declare_parameter("a_long_max", 0.8)
        self.declare_parameter("a_long_min", -1.2)
        self.declare_parameter("curvature_eps", 1e-3)
        self.declare_parameter("lambda_goal", 1.6)
        self.declare_parameter("d_stop", 0.25)

        self.declare_parameter("kp_yaw", 0.8)
        self.declare_parameter("ki_yaw", 0.05)
        self.declare_parameter("yaw_int_limit", 0.6)
        self.declare_parameter("steering_max_deg", 28.0)
        self.declare_parameter("steering_rate_max_deg", 120.0)
        self.declare_parameter("v_steer_eps", 0.15)

        self.declare_parameter("kp_v", 0.9)
        self.declare_parameter("ki_v", 0.15)
        self.declare_parameter("vel_int_limit", 0.8)
        self.declare_parameter("k_ff_v", 0.35)

        self.declare_parameter("throttle_min", -1.0)
        self.declare_parameter("throttle_max", 1.0)

        self.declare_parameter("sigma_enc", 0.03)
        self.declare_parameter("sigma_slam", 0.08)

        self.declare_parameter("lookahead_dist", 0.8)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("goal_tolerance", 0.25)

        self.declare_parameter("encoder_topic", "/encoder_velocity")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("goal_topic", "/goal_pose")

        self.declare_parameter("throttle_cmd_topic", "/throttle_cmd")
        self.declare_parameter("steering_cmd_topic", "/steering_cmd")
        self.declare_parameter("vref_topic", "/controller/v_ref")
        self.declare_parameter("vmeas_topic", "/controller/v_meas")
        self.declare_parameter("curvature_topic", "/controller/curvature")

        # Cached parameters
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)

        self.v_max = float(self.get_parameter("v_max").value)
        self.v_min = float(self.get_parameter("v_min").value)
        self.a_lat_max = float(self.get_parameter("a_lat_max").value)
        self.a_long_max = float(self.get_parameter("a_long_max").value)
        self.a_long_min = float(self.get_parameter("a_long_min").value)
        self.curvature_eps = float(self.get_parameter("curvature_eps").value)
        self.lambda_goal = float(self.get_parameter("lambda_goal").value)
        self.d_stop = float(self.get_parameter("d_stop").value)

        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.ki_yaw = float(self.get_parameter("ki_yaw").value)
        self.yaw_int_limit = float(self.get_parameter("yaw_int_limit").value)
        self.steering_max = math.radians(float(self.get_parameter("steering_max_deg").value))
        self.steering_rate_max = math.radians(float(self.get_parameter("steering_rate_max_deg").value))
        self.v_steer_eps = float(self.get_parameter("v_steer_eps").value)

        self.kp_v = float(self.get_parameter("kp_v").value)
        self.ki_v = float(self.get_parameter("ki_v").value)
        self.vel_int_limit = float(self.get_parameter("vel_int_limit").value)
        self.k_ff_v = float(self.get_parameter("k_ff_v").value)

        self.throttle_min = float(self.get_parameter("throttle_min").value)
        self.throttle_max = float(self.get_parameter("throttle_max").value)

        self.sigma_enc = float(self.get_parameter("sigma_enc").value)
        self.sigma_slam = float(self.get_parameter("sigma_slam").value)

        self.lookahead_dist = float(self.get_parameter("lookahead_dist").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        encoder_topic = str(self.get_parameter("encoder_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        goal_topic = str(self.get_parameter("goal_topic").value)

        throttle_cmd_topic = str(self.get_parameter("throttle_cmd_topic").value)
        steering_cmd_topic = str(self.get_parameter("steering_cmd_topic").value)
        vref_topic = str(self.get_parameter("vref_topic").value)
        vmeas_topic = str(self.get_parameter("vmeas_topic").value)
        curvature_topic = str(self.get_parameter("curvature_topic").value)

        # -----------------------------
        # Internal state
        # -----------------------------
        self.ctrl = ControllerState()

        self.v_enc = 0.0
        self.v_slam = 0.0
        self.omega_imu = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.current_path = None
        self.goal_x = None
        self.goal_y = None

        self.last_time = self.get_clock().now()

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(Float32, encoder_topic, self.encoder_cb, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(Imu, imu_topic, self.imu_cb, 10)
        self.create_subscription(Path, path_topic, self.path_cb, 10)
        self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)

        # -----------------------------
        # Publishers
        # -----------------------------
        self.throttle_pub = self.create_publisher(Float32, throttle_cmd_topic, 10)
        self.steering_pub = self.create_publisher(Float32, steering_cmd_topic, 10)

        self.vref_pub = self.create_publisher(Float32, vref_topic, 10)
        self.vmeas_pub = self.create_publisher(Float32, vmeas_topic, 10)
        self.curvature_pub = self.create_publisher(Float32, curvature_topic, 10)

        # -----------------------------
        # Timer
        # -----------------------------
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info("rover_controller started")

    # -------------------------------------------------
    # Subscriber callbacks
    # -------------------------------------------------
    def encoder_cb(self, msg: Float32) -> None:
        self.v_enc = float(msg.data)

    def odom_cb(self, msg: Odometry) -> None:
        self.v_slam = float(msg.twist.twist.linear.x)
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def imu_cb(self, msg: Imu) -> None:
        self.omega_imu = float(msg.angular_velocity.z)

    def path_cb(self, msg: Path) -> None:
        self.current_path = msg

    def goal_cb(self, msg: PoseStamped) -> None:
        self.goal_x = float(msg.pose.position.x)
        self.goal_y = float(msg.pose.position.y)

    # -------------------------------------------------
    # Core helper methods
    # -------------------------------------------------
    def fuse_velocity(self, v_enc: float, v_slam: float) -> float:
        var_enc = max(self.sigma_enc ** 2, 1e-8)
        var_slam = max(self.sigma_slam ** 2, 1e-8)

        num = (v_enc / var_enc) + (v_slam / var_slam)
        den = (1.0 / var_enc) + (1.0 / var_slam)
        return num / den

    def compute_v_curve(self, curvature: float) -> float:
        return math.sqrt(self.a_lat_max / (abs(curvature) + self.curvature_eps))

    def compute_v_goal(self, distance_to_goal: float) -> float:
        if distance_to_goal <= self.d_stop:
            return 0.0
        return self.v_min + (self.v_max - self.v_min) * (
            1.0 - math.exp(-self.lambda_goal * distance_to_goal)
        )

    def compute_v_desired(self, curvature: float, distance_to_goal: float) -> float:
        v_curve = self.compute_v_curve(curvature)
        v_goal = self.compute_v_goal(distance_to_goal)
        return min(self.v_max, v_curve, v_goal)

    def apply_accel_limit(self, v_desired: float, dt: float) -> float:
        dv_desired = v_desired - self.ctrl.v_ref_prev
        dv_max = self.a_long_max * dt
        dv_min = self.a_long_min * dt
        dv_limited = clamp(dv_desired, dv_min, dv_max)

        v_ref = self.ctrl.v_ref_prev + dv_limited
        self.ctrl.v_ref_prev = v_ref
        return v_ref

    def compute_omega_ff(self, curvature: float, v_ref: float) -> float:
        return v_ref * curvature

    def compute_omega_ref(self, omega_ff: float, omega_meas: float, dt: float) -> float:
        e_omega = omega_ff - omega_meas

        self.ctrl.yaw_int += e_omega * dt
        self.ctrl.yaw_int = clamp(
            self.ctrl.yaw_int, -self.yaw_int_limit, self.yaw_int_limit
        )

        return omega_ff + self.kp_yaw * e_omega + self.ki_yaw * self.ctrl.yaw_int

    def omega_to_steering(self, omega_ref: float, v_ref: float, dt: float) -> float:
        v_safe = max(abs(v_ref), self.v_steer_eps)
        delta_cmd = math.atan((self.wheelbase * omega_ref) / v_safe)

        delta_cmd = clamp(delta_cmd, -self.steering_max, self.steering_max)

        max_step = self.steering_rate_max * dt
        delta_cmd = clamp(
            delta_cmd,
            self.ctrl.delta_prev - max_step,
            self.ctrl.delta_prev + max_step,
        )

        self.ctrl.delta_prev = delta_cmd
        return delta_cmd

    def compute_throttle(self, v_ref: float, v_meas: float, dt: float) -> float:
        e_v = v_ref - v_meas

        self.ctrl.vel_int += e_v * dt
        self.ctrl.vel_int = clamp(
            self.ctrl.vel_int, -self.vel_int_limit, self.vel_int_limit
        )

        throttle_ff = self.k_ff_v * v_ref
        throttle_fb = self.kp_v * e_v + self.ki_v * self.ctrl.vel_int
        throttle = throttle_ff + throttle_fb

        return clamp(throttle, self.throttle_min, self.throttle_max)

    # -------------------------------------------------
    # Path geometry helpers
    # -------------------------------------------------
    def compute_goal_distance(self) -> float:
        if self.goal_x is not None and self.goal_y is not None:
            return math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)

        if self.current_path is not None and len(self.current_path.poses) > 0:
            gx = self.current_path.poses[-1].pose.position.x
            gy = self.current_path.poses[-1].pose.position.y
            return math.hypot(gx - self.current_x, gy - self.current_y)

        return float("inf")

    def find_lookahead_index(self, poses) -> int:
        if not poses:
            return 0

        best_i = 0
        best_d = float("inf")
        for i, ps in enumerate(poses):
            px = ps.pose.position.x
            py = ps.pose.position.y
            d = math.hypot(px - self.current_x, py - self.current_y)
            if d < best_d:
                best_d = d
                best_i = i

        accum = 0.0
        i = best_i
        while i < len(poses) - 1 and accum < self.lookahead_dist:
            x1 = poses[i].pose.position.x
            y1 = poses[i].pose.position.y
            x2 = poses[i + 1].pose.position.x
            y2 = poses[i + 1].pose.position.y
            accum += math.hypot(x2 - x1, y2 - y1)
            i += 1
        return i

    def compute_curvature_from_path(self) -> float:
        if self.current_path is None or len(self.current_path.poses) < 2:
            return 0.0

        poses = self.current_path.poses
        idx = self.find_lookahead_index(poses)
        lookahead_pose = poses[idx].pose.position

        dx_w = lookahead_pose.x - self.current_x
        dy_w = lookahead_pose.y - self.current_y

        # Transform lookahead point into robot frame
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)

        x_r = cos_y * dx_w + sin_y * dy_w
        y_r = -sin_y * dx_w + cos_y * dy_w

        ld2 = x_r * x_r + y_r * y_r
        if ld2 < 1e-6:
            return 0.0

        # Pure pursuit curvature
        curvature = 2.0 * y_r / ld2
        return curvature

    # -------------------------------------------------
    # Main control loop
    # -------------------------------------------------
    def control_loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        if self.current_path is None or len(self.current_path.poses) < 2:
            return

        # 1) measured speed
        v_meas = self.fuse_velocity(self.v_enc, self.v_slam)

        # 2) path curvature and goal distance
        curvature = self.compute_curvature_from_path()
        distance_to_goal = self.compute_goal_distance()

        # 3) desired and smoothed reference speed
        v_desired = self.compute_v_desired(curvature, distance_to_goal)
        v_ref = self.apply_accel_limit(v_desired, dt)

        # 4) yaw-rate feedforward + feedback
        omega_ff = self.compute_omega_ff(curvature, v_ref)
        omega_ref = self.compute_omega_ref(omega_ff, self.omega_imu, dt)

        # 5) steering command
        steering = self.omega_to_steering(omega_ref, v_ref, dt)

        # 6) throttle command
        throttle = self.compute_throttle(v_ref, v_meas, dt)

        # Optional stop logic near goal
        if distance_to_goal <= self.goal_tolerance:
            throttle = 0.0
            v_ref = 0.0

        # Publish commands
        throttle_msg = Float32()
        throttle_msg.data = float(throttle)
        self.throttle_pub.publish(throttle_msg)

        steering_msg = Float32()
        steering_msg.data = float(steering)
        self.steering_pub.publish(steering_msg)

        # Publish debug
        msg_vref = Float32()
        msg_vref.data = float(v_ref)
        self.vref_pub.publish(msg_vref)

        msg_vmeas = Float32()
        msg_vmeas.data = float(v_meas)
        self.vmeas_pub.publish(msg_vmeas)

        msg_curv = Float32()
        msg_curv.data = float(curvature)
        self.curvature_pub.publish(msg_curv)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoverControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
