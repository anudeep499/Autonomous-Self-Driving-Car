import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Vector3, TransformStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class ImuOdom(Node):
    def __init__(self):
        super().__init__('imu_odom')
        self.declare_parameter('publish_rate', 20.0)    # publish rate of IMU data in Hz
        self.declare_parameter('en_calculate_velocity', True)  # if set to True, node will attempt to calculate velocities
        self.declare_parameter('en_x_accel', True)  # if set to True, calculates x velocity and position based on accel data
        self.declare_parameter('en_y_accel', False)  # if set to True, calculates y velocity and position based on accel data, should not be used for ackerman steering if IMU is oriented with y perpendicular to wheels
        self.declare_parameter('en_thresholding', True)  # will deadband accel data, ignoring values close to 0
        self.declare_parameter('en_zupt', True)  # ZUPT (Zero-Velocity Update) - If set to True, velocity will be set to 0 when cmd_vel is 0
        self.declare_parameter('ax_threshold', 0.2)  # used for ZUPT and accel thresholding
        self.declare_parameter('ay_threshold', 0.2)
        
        self.declare_parameter('base_frame', 'base_link')   # default child frame name for TransformStamped
        self.declare_parameter('odom_frame', 'odom')    # default base frame name for TransformStamped
        
        self.declare_parameter('max_dt', 0.5)                      # s; drop huge dt spikes
         
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # Configuration settings
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.en_calculate_velocity = bool(self.get_parameter('en_calculate_velocity').value)
        self.en_x_accel = bool(self.get_parameter('en_x_accel').value)
        self.en_y_accel = bool(self.get_parameter('en_y_accel').value)
        self.en_thresholding = bool(self.get_parameter('en_thresholding').value)
        self.en_zupt = bool(self.get_parameter('en_zupt').value)
        self.ax_threshold = float(self.get_parameter('ax_threshold').value)
        self.ay_threshold = float(self.get_parameter('ay_threshold').value)

        
        # Subscribers
        self.gyro_sub = self.create_subscription(Vector3, '/imu/gyro', self.gyro_callback, imu_qos)
        self.accel_sub = self.create_subscription(Vector3, '/imu/accel', self.accel_callback, imu_qos)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, cmd_qos)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)    # broadcasts to /tf (dynamic transform)
        
        # Safety params
        self.max_dt = float(self.get_parameter('max_dt').value)
        
        # Current pose state (position, orientation)
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        
        # Current twist state (linear velocities, angular velocity)
        self.curr_vx = 0.0
        self.curr_vy = 0.0
        self.curr_wz = 0.0 # angular velocity about z
        
        # Latest IMU samples from gyro_sub and accel_sub
        self.latest_ax = 0.0
        self.latest_ay = 0.0
        self.latest_wz = 0.0
        
        # Latest cmd_vel values 
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0   # should always be zero for ackerman
        self.cmd_wz = 0.0
        
        # Time of current state
        self.state_time = None
        
        self.get_logger().info("imu_odom started, calculating")
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_state)
        
    # ---- helpers ----
    @staticmethod
    def _wrap_pi(a: float) -> float:
        while a >  math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a
    
    def _propagate_to(self, now):
        """Advance full state from state_time -> now using last known IMU samples (ZOH)."""
        if self.state_time is None:
            self.state_time = now
            return

        dt = (now - self.state_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > self.max_dt:
            # bad/huge step — reset timebase but don't integrate
            self.state_time = now
            return

        # 1) yaw from gyro (planar)
        self.curr_wz = self.latest_wz
        self.curr_yaw = self._wrap_pi(self.curr_yaw + self.curr_wz * dt)

        # 2) rotate accel body->odom (assume flat: roll≈pitch≈0)
        c, s = math.cos(self.curr_yaw), math.sin(self.curr_yaw)
        ax_o = self.latest_ax * c - self.latest_ay * s
        ay_o = self.latest_ax * s + self.latest_ay * c

        # 3) integrate vel & pos
        if self.en_calculate_velocity:
            self.curr_vx += ax_o * dt
            self.curr_vy += ay_o * dt
        
        # 4) ZUPT logic (if enabled)
        if self.en_zupt:
            if abs(self.cmd_vx) < 0.01 and abs(self.latest_ax) < (self.ax_threshold * 0.5) and abs(self.cmd_vy) < 0.01 and abs(self.latest_ay) < (self.ay_threshold * 0.5):
                self.curr_vx = 0.0
                self.curr_vy = 0.0

        self.curr_x += self.curr_vx * dt
        self.curr_y += self.curr_vy * dt

        self.state_time = now
        
    # ---- callbacks ----
    def accel_callback(self, msg: Vector3):
        now = self.get_clock().now()
        self._propagate_to(now)
        
        # X accel
        if self.en_x_accel:
            self.latest_ax = msg.x if (not self.en_thresholding or abs(msg.x) > self.ax_threshold) else 0.0
        else:
            self.latest_ax = 0.0

        # Y accel (often disabled for Ackermann)
        if self.en_y_accel:
            self.latest_ay = msg.y if (not self.en_thresholding or abs(msg.y) > self.ay_threshold) else 0.0
        else:
            self.latest_ay = 0.0
        
        # self.latest_ax = msg.x if (self.en_x_accel and (self.en_thresholding and (abs(msg.x) > self.ax_threshold))) else 0  # ignore y accel data if en_y_accel is false (ie for ackerman-steering)
        # self.latest_ay = msg.y if (self.en_y_accel and (self.en_thresholding and (abs(msg.y) > self.ay_threshold))) else 0  # ignore y accel data if en_y_accel is false (ie for ackerman-steering)

    def gyro_callback(self, msg: Vector3):
        now = self.get_clock().now()
        self._propagate_to(now)
        self.latest_wz = msg.z
        
    def cmd_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z

    # ---- publisher tick ----
    def publish_state(self):
        now = self.get_clock().now()
        self._propagate_to(now)
        now_msg = now.to_msg()

        # Odometry
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.curr_x
        odom.pose.pose.position.y = self.curr_y
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.curr_yaw)
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        odom.twist.twist.linear.x = self.curr_vx
        odom.twist.twist.linear.y = self.curr_vy
        odom.twist.twist.angular.z = self.curr_wz

        # Minimal covariances (need to tune later)
        odom.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2,
        ]

        odom.twist.covariance = [
            0.3, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.3, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.3,
        ]

        self.odom_pub.publish(odom)

        # TF: odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = now_msg
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.curr_x
        tf.transform.translation.y = self.curr_y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)
        
def main(args=None):
    rclpy.init(args=args)
    node = ImuOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
