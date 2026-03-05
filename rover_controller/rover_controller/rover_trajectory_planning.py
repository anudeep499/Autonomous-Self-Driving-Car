#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, Vector3
from nav_msgs.msg import Odometry

from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion

# If you made rover_msgs:
from rover_msgs.msg import RoverActuatorCmd


def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def normalize_angle(a):
    while a > math.pi:  a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


class RoverController(Node):
    """
    The 'brain' node:
      Nav2 cmd_vel + rf2o odom + IMU yaw-rate + goal distance
      -> computes throttle_cmd + steering_cmd
      -> publishes /rover_actuator_cmd
    """
    def __init__(self):
        super().__init__("rover_controller")

        # -------- Params / topics --------
        self.declare_parameter("cmd_vel_in", "/cmd_vel")          # Nav2 output
        self.declare_parameter("odom_in", "/odom_rf2o")           # rf2o output
        self.declare_parameter("gyro_in", "/imu/gyro")            # rover_node output
        self.declare_parameter("goal_in", "/goal_pose")           # your goal topic
        self.declare_parameter("actuator_out", "/rover_actuator_cmd")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # -------- Speed policy knobs --------
        self.declare_parameter("v_hw", 1.5)          # absolute max allowed (m/s)
        self.declare_parameter("v_min", 0.10)        # crawl speed near goal (m/s)
        self.declare_parameter("v_max", 1.20)        # preferred max (m/s)
        self.declare_parameter("lambda_goal", 0.8)   # exponential ramp rate
        self.declare_parameter("d_stop", 0.20)       # stop radius (m)

        self.declare_parameter("a_lat_max", 1.5)     # lateral accel limit (m/s^2)
        self.declare_parameter("kappa_eps", 0.05)    # avoids blow-up at tiny curvature

        # -------- Accel limiter knob --------
        self.declare_parameter("a_max", 1.0)         # |dv/dt| <= a_max (m/s^2)

        # -------- Velocity measurement fusion knob --------
        self.declare_parameter("alpha_enc", 0.7)     # if encoder exists
        # NOTE: if you don't actually have encoders in code, set v_enc=None and just use v_slam

        # -------- PI + feedforward knobs --------
        self.declare_parameter("Kp_v", 0.8)
        self.declare_parameter("Ki_v", 0.3)
        self.declare_parameter("k_ff", 0.35)         # throttle per (m/s) (rough)

        self.declare_parameter("throttle_min", -1.0)
        self.declare_parameter("throttle_max",  1.0)

        # -------- Yaw feedback knobs --------
        self.declare_parameter("Kp_yaw", 1.0)
        self.declare_parameter("Ki_yaw", 0.15)
        self.declare_parameter("wheelbase_L", 0.32)   # meters (measure your RC)
        self.declare_parameter("steer_max", 1.0)      # normalized steering limit

        # -------- State --------
        self.last_cmd_vel = None   # Twist
        self.last_odom = None      # Odometry
        self.last_gyro = None      # Vector3
        self.goal_pose = None      # PoseStamped

        self.v_ref = 0.0
        self.I_v = 0.0
        self.I_yaw = 0.0

        # -------- TF --------
        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------- IO --------
        self.cmd_sub = self.create_subscription(
            Twist, self.get_parameter("cmd_vel_in").value, self.cb_cmd_vel, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter("odom_in").value, self.cb_odom, 10
        )
        self.gyro_sub = self.create_subscription(
            Vector3, self.get_parameter("gyro_in").value, self.cb_gyro, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, self.get_parameter("goal_in").value, self.cb_goal, 10
        )

        self.act_pub = self.create_publisher(
            RoverActuatorCmd, self.get_parameter("actuator_out").value, 10
        )

        # control loop
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info("RoverController started.")

    # ---------- callbacks ----------
    def cb_cmd_vel(self, msg: Twist):
        self.last_cmd_vel = msg

    def cb_odom(self, msg: Odometry):
        self.last_odom = msg

    def cb_gyro(self, msg: Vector3):
        self.last_gyro = msg

    def cb_goal(self, msg: PoseStamped):
        self.goal_pose = msg

    # ---------- helpers ----------
    def get_distance_to_goal(self):
        if self.goal_pose is None:
            return None

        try:
            now = rclpy.time.Time()
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, now)
        except TransformException:
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        gx = self.goal_pose.pose.position.x
        gy = self.goal_pose.pose.position.y
        return math.hypot(gx - x, gy - y)

    def speed_from_odom(self, odom: Odometry):
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        return math.hypot(vx, vy)

    # ---------- main loop ----------
    def loop(self):
        # Need Nav2 cmd_vel + odom + gyro to do full loop
        if self.last_cmd_vel is None or self.last_odom is None or self.last_gyro is None:
            return

        dt = self.dt

        # 1) Read Nav2 "intent"
        v_nav2 = self.last_cmd_vel.linear.x
        w_nav2 = self.last_cmd_vel.angular.z

        # 2) Estimate curvature from Nav2 intent (simple and works well enough)
        eps = 1e-3
        kappa = w_nav2 / max(abs(v_nav2), eps)  # κ ≈ ω / v

        # 3) Distance to goal (optional but recommended)
        d = self.get_distance_to_goal()
        if d is None:
            d = 1e9  # treat as far away if no goal/TF

        # 4) Build speed limits:
        v_min = self.get_parameter("v_min").value
        v_max = self.get_parameter("v_max").value
        v_hw  = self.get_parameter("v_hw").value
        lam   = self.get_parameter("lambda_goal").value
        d_stop = self.get_parameter("d_stop").value

        # 4a) goal-distance speed (exponential)
        if d < d_stop:
            v_goal = 0.0
        else:
            v_goal = v_min + (v_max - v_min) * (1.0 - math.exp(-lam * d))

        # 4b) curve/traction speed limit: a_lat = v^2 |κ| <= a_lat_max
        a_lat_max = self.get_parameter("a_lat_max").value
        kappa_eps = self.get_parameter("kappa_eps").value
        v_curve = math.sqrt(a_lat_max / (abs(kappa) + kappa_eps))

        # 4c) Nav2 speed magnitude limit
        v_nav2_mag = abs(v_nav2)

        # 4d) raw speed request = safest cap
        v_raw_mag = min(v_nav2_mag, v_goal, v_curve, v_hw)
        v_raw = math.copysign(v_raw_mag, v_nav2)  # keep nav2 direction

        # 5) acceleration limiting (smooth transitions)
        a_max = self.get_parameter("a_max").value
        dv = clamp(v_raw - self.v_ref, -a_max * dt, +a_max * dt)
        self.v_ref += dv

        # 6) measured forward speed (from rf2o odom for now)
        v_slam = self.speed_from_odom(self.last_odom)

        # If you later add encoders:
        v_enc = None
        if v_enc is None:
            v_meas = v_slam
        else:
            alpha = self.get_parameter("alpha_enc").value
            v_meas = alpha * v_enc + (1 - alpha) * v_slam

        # 7) Velocity PI + feedforward -> throttle_cmd (normalized)
        Kp_v = self.get_parameter("Kp_v").value
        Ki_v = self.get_parameter("Ki_v").value
        k_ff = self.get_parameter("k_ff").value

        e_v = self.v_ref - v_meas
        self.I_v += e_v * dt

        throttle_ff = k_ff * self.v_ref
        throttle_fb = Kp_v * e_v + Ki_v * self.I_v
        throttle_cmd = throttle_ff + throttle_fb

        tmin = self.get_parameter("throttle_min").value
        tmax = self.get_parameter("throttle_max").value
        throttle_sat = clamp(throttle_cmd, tmin, tmax)

        # simple anti-windup
        if throttle_sat != throttle_cmd and abs(Ki_v) > 1e-6:
            self.I_v += (throttle_sat - throttle_cmd) / Ki_v

        throttle_cmd = throttle_sat

        # 8) Yaw feedback steering
        w_meas = self.last_gyro.z  # IMU yaw-rate (rad/s)
        e_w = w_nav2 - w_meas

        Kp_y = self.get_parameter("Kp_yaw").value
        Ki_y = self.get_parameter("Ki_yaw").value
        self.I_yaw += e_w * dt

        w_ref = w_nav2 + (Kp_y * e_w + Ki_y * self.I_yaw)

        # Convert desired yaw-rate to steering angle via bicycle model:
        # ω = (v/L) tan(δ)  =>  δ = atan( (L ω) / v )
        L = self.get_parameter("wheelbase_L").value
        v_for_steer = max(abs(self.v_ref), 0.10)  # avoid crazy steering at ~0 speed
        delta = math.atan((L * w_ref) / v_for_steer)

        # Convert delta to normalized steering in [-1, 1]
        # (You calibrate: delta_max is your physical max steering angle)
        delta_max = 0.45  # rad ~ 25 deg (example)
        steer_norm = clamp(delta / delta_max, -1.0, 1.0)
        steer_norm = clamp(steer_norm, -self.get_parameter("steer_max").value,
                           +self.get_parameter("steer_max").value)

        # 9) Publish actuator command
        out = RoverActuatorCmd()
        out.throttle = float(throttle_cmd)
        out.steering = float(steer_norm)
        self.act_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RoverController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
