#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import time
import os

from geometry_msgs.msg import Twist
import pyrealsense2 as rs


class LineFollowerFitOnly(Node):
    def __init__(self):
        super().__init__('line_follower_fit_only')

        # ------------- Config -------------
        self.debug = True

        # Constant forward speed
        self.forward_speed = 0.22
        self.max_turn_rate = 1.5      # [rad/s] max search turn rate (tune)  # NEW
        self.lost_turn_duration = 3.0 # [s] how long to keep turning after losing line  # NEW
        self.last_turn_sign = 0.0     # +1 for left, -1 for right, 0 = unknown  # NEW
        self.lost_start_time = None   # when we first lost the line  # NEW
        
        # TUNING
        self.weighting = True
        
        # Centroid PID gains
        self.k_p = 0.0055
        self.k_d = 0.0008
        
        # Theta PID gains (from line angle error)
        self.k_theta_p = 1.1  # tune this
        self.k_theta_d = 0.2
        
        # Centroid error confidence
        self.centroid_weight = 0.65

        # HSV mask for blue (your original values)
        self.lower_blue = np.array([95, 95, 100]) # 70, 80, 130
        self.upper_blue = np.array([110, 235, 220]) # 140, 255, 255

        # ROI height from bottom (in both raw & BEV)
        self.roi_height = 365   # pixels from bottom up # 360
        self.roi_bottom = 1 # 20

        # Minimum number of pixels required to trust the fit
        self.min_pixels_for_fit = 120

        # Debug image folder
        self.debug_folder = os.path.expanduser("~/line_follower_fit_debug")
        os.makedirs(self.debug_folder, exist_ok=True)
        self.latest_raw = None
        self.latest_bev = None
        self.latest_mask = None
        self.latest_debug = None

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/rover_vel', 10)

        # ------------- RealSense setup -------------
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

        self.get_logger().info("Starting RealSense pipeline...")
        self.pipeline.start(self.config)

        # Precompute BEV transform matrix once we know image size (we assume 640x480)
        self.bev_M = None
        
        self.last_theta_error = 0.0
        self.last_cx_error = 0.0

        # Frame timer
        self.frame_timer = self.create_timer(1.0 / 15.0, self.process_frame)

        # Stats / centroid book-keeping
        self.last_centroid = None
        self.last_num_pixels = 0
        self.last_num_rows_with_blue = 0
        self.last_num_cols_with_blue = 0

        # Debug save timer
        if self.debug:
            self.debug_timer = self.create_timer(5.0, self.save_debug_images)

        self.get_logger().info("LineFollowerFitOnly node started.")

    # --------------------------------------------
    # Initialize BEV matrix
    # --------------------------------------------
    def init_bev_matrix(self, w: int, h: int):
        """
        Initialize a simple perspective transform to a bird's-eye view.
        You will likely want to tune the src points for your camera angle.
        """
        # Source quad in the original image (approximate trapezoid around the tape)
        src = np.float32([
            [0,       h - self.roi_bottom],                # bottom-left
            [w - 1,   h - self.roi_bottom],                # bottom-right
            [int(0.3* w), h - self.roi_height],  # top-left (tune)
            [int(0.7 * w), h - self.roi_height],   # top-right (tune)
        ])

        # Destination quad: full rectangle (top-down)
        dst = np.float32([
            [0,       h - 1],
            [w - 1,   h - 1],
            [0,       0],
            [w - 1,   0],
        ])

        self.bev_M = cv2.getPerspectiveTransform(src, dst)
        self.get_logger().info("Initialized BEV perspective transform.")

    # --------------------------------------------
    # Main frame processing
    # --------------------------------------------
    def process_frame(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as e:
            self.get_logger().warn(f"Failed to get frame from RealSense: {e}")
            return

        color_frame = frames.get_color_frame()
        if not color_frame:
            self.get_logger().warn("No color frame received from RealSense.")
            return

        frame = np.asanyarray(color_frame.get_data())
        self.latest_raw = frame.copy()

        centroid_weight = 1

        h, w, _ = frame.shape
        
        cx_target = w // 2

        if self.bev_M is None:
            self.init_bev_matrix(w, h)
            
        # =========================================================
        # 1) CENTROID FROM RAW IMAGE (using bottom ROI on raw frame)
        # =========================================================
        # 1) Threshold RAW ONCE
        hsv_raw = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_color_raw = cv2.inRange(hsv_raw, self.lower_blue, self.upper_blue)

        # 2) RAW centroid ROI
        if self.roi_height < h:
            roi_top_raw = h - self.roi_height
        else:
            roi_top_raw = 0
            
        # remove pixels outside of ROI
        mask_raw = np.zeros_like(mask_color_raw)
        mask_raw[roi_top_raw:h, :] = mask_color_raw[roi_top_raw:h, :]

        # binary values for unwarped mask pixels:
        ys_raw, xs_raw = np.where(mask_raw > 0)
        num_pixels_raw_xs = xs_raw.size
        num_pixels_raw_ys = ys_raw.size

        # 3) Warp the binary mask to BEV instead of full color
            
        bev_mask_color = cv2.warpPerspective(mask_raw, self.bev_M, (w, h))

        self.latest_mask = bev_mask_color

        # Basic stats on BEV (and raw) mask: row/column
        
        ys, xs = np.where(bev_mask_color > 0)
        
        num_pixels_xs = xs.size
        num_pixels_xy = ys.size
        
        cx_centroid = None
        cy_centroid = None

        if num_pixels_xs >= self.min_pixels_for_fit:
            if self.weighting:
                h, w = bev_mask_color.shape[:2]
                y_norm = ys.astype(float) / (self.roi_height)

                max_w = 1
                min_w = 0.2
                weights = max_w - (max_w - min_w) * y_norm

                cx_centroid = int(np.average(xs, weights=weights))
                print(f"\n weighted centroid: {cx_centroid} \n")
                cy_centroid = int(np.average(ys, weights=weights))
                cx_centroid_uw = float(xs.mean())
                print(f"\n unweighted centroid: {cx_centroid_uw} \n")
                cy_centroid_uw = float(ys.mean())
            else:
                if self.debug:
                    self.get_logger().info("No BEV pixels > 0; skipping centroid.")
                cx_centroid = float(xs.mean())
                cy_centroid = float(ys.mean())
            self.last_centroid = (cx_centroid, cy_centroid)
        else:
            self.last_centroid = None

        # Debug vis in BEV space
        debug_vis = cv2.cvtColor(bev_mask_color, cv2.COLOR_GRAY2BGR)

        self.latest_debug = debug_vis.copy()

        twist = Twist()

        # ----------------------------------------
        # 3) Fit line only if enough BEV pixels
        # ----------------------------------------

        if num_pixels_xs >= 2*self.min_pixels_for_fit:
            centroid_weight = self.centroid_weight
            pts = np.column_stack((xs, ys)).astype(np.float32)

            # Fit line: returns a normalized direction vector (vx, vy)
            # and a point on the line (x0, y0)
            vx, vy, x0, y0 = cv2.fitLine(
                pts,
                cv2.DIST_L2,
                0,
                0.01,
                0.01
            )

            vx = float(vx)
            vy = float(vy)
            x0 = float(x0)
            y0 = float(y0)

            # Normalize line direction to be consistent
            if vy > 0:
                vx = -vx
                vy = -vy

            # Draw the fitted line on the debug image
            y1 = 0
            y2 = h - 1

            if abs(vy) < 1e-6:
                # Degenerate case: nearly horizontal line -> no good for steering
                self.get_logger().warn("Fitted line is almost horizontal. Stopping.")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return

            t1 = (y1 - y0) / vy
            t2 = (y2 - y0) / vy
            x1 = int(x0 + t1 * vx)
            x2 = int(x0 + t2 * vx)

            cv2.line(debug_vis, (x1, int(y1)), (x2, int(y2)), (0, 0, 255), 3)
            
            if cx_centroid is not None:
                cv2.circle(debug_vis, (cx_centroid, cy_centroid), 6, (0, 0, 255), -1) 
                cv2.drawMarker(debug_vis, (cx_centroid, cy_centroid), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)
            self.latest_debug = debug_vis.copy()

            # ----------------------------------------
            # 4) Steering from line angle (fit) + centroid (raw)
            # ----------------------------------------
            # Fit angle
            theta = -np.arctan2(vy, vx)
            print(theta)

            theta_desired = np.pi / 2.0
            theta_error = - (theta - theta_desired)  ## CHANGED FOR INVERSION


            # Centroid error from RAW image
            if cx_centroid is not None:
                cx_error = - (cx_centroid - cx_target)  ## CHANGED FOR INVERSION
                
            else:
                cx_error = 0.0  # no centroid; treat as no lateral error

            theta_derivative = theta_error - self.last_theta_error
            cx_derivative = cx_error - self.last_cx_error

            
            
            # calculate angular twist based on centroid and theta
            centroid_component = centroid_weight * (self.k_p * cx_error + self.k_d * cx_derivative)
            theta_component = -(1 - centroid_weight) * (self.k_theta_p * theta_error + self.k_theta_d * theta_derivative)
            
            twist.linear.x = self.forward_speed
            twist.angular.z = centroid_component + theta_component
            
            # --- NEW: remember last turn direction (if actually turning)
            if abs(twist.angular.z) > 0.235:
                self.last_turn_sign = np.sign(twist.angular.z)  # +1 left, -1 right
            else:
                self.last_turn_sign = 0

            self.lost_start_time = None

            self.cmd_pub.publish(twist)
            self.last_cx_error = cx_error

            if self.debug:
                self.get_logger().info(
                    f"[FIT] pixels={num_pixels_xs}, theta={theta:.3f}, "
                    f"theta_err={theta_error:.3f}, cx_err={cx_error:.1f}, "
                    f"centroid_component={centroid_component:.3f}, theta_component={theta_component:.1f}, "
                )

        else:
            # Not enough pixels -> stop
            if self.debug:
                self.get_logger().info(
                    f"[NO LINE] Only {num_pixels_xs} pixels, below threshold {self.min_pixels_for_fit}."
                )
                
            now = time.time()
            
            if self.lost_start_time is None:          # NEW
                self.lost_start_time = now            # NEW
            
            elapsed = now - self.lost_start_time      # NEW
            
            if (
                self.last_turn_sign != 0.0            # we know last direction      NEW
                and elapsed < self.lost_turn_duration # still within search window NEW
            ):
                # Keep turning in the last direction at max rate
                twist.linear.x = self.forward_speed
                twist.angular.z = self.last_turn_sign * self.max_turn_rate
                if self.debug:
                    self.get_logger().info(
                        f"[SEARCHING] elapsed={elapsed:.2f}s, "
                        f"turn_sign={self.last_turn_sign:+.0f}, "
                        f"angular_z={twist.angular.z:.2f}"
                    )
            elif (
                self.last_turn_sign == 0.0            # we know last direction      NEW
                and elapsed < self.lost_turn_duration # still within search window NEW
            ):
                # Keep turning in the last direction at max rate
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0
                if self.debug:
                    self.get_logger().info(
                        f"[SEARCHING] elapsed={elapsed:.2f}s, "
                        f"turn_sign={self.last_turn_sign:+.0f}, "
                        f"angular_z={twist.angular.z:.2f}"
                    )
            else:
                # Either we never had a direction or we timed out -> stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                if self.debug:
                    self.get_logger().info(
                        f"[SEARCH STOP] elapsed={elapsed:.2f}s, stopping robot."
                    )
                        
            self.cmd_pub.publish(twist)

            self.latest_debug = debug_vis.copy()

    # --------------------------------------------
    # Debug image saving
    # --------------------------------------------
    def save_debug_images(self):
        if self.latest_raw is None:
            return

        timestamp = int(time.time())

        cv2.imwrite(os.path.join(self.debug_folder, "raw.jpg"), self.latest_raw)
        if self.latest_bev is not None:
            cv2.imwrite(os.path.join(self.debug_folder, "bev.jpg"), self.latest_bev)
        if self.latest_mask is not None:
            cv2.imwrite(os.path.join(self.debug_folder, "mask.jpg"), self.latest_mask)
        if self.latest_debug is not None:
            cv2.imwrite(os.path.join(self.debug_folder, "debug.jpg"), self.latest_debug)

        self.get_logger().info(f"Saved debug images @ {timestamp}")

    # --------------------------------------------
    # Cleanup
    # --------------------------------------------
    def destroy_node(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)
        self.get_logger().info("Published final stop Twist before shutdown.")
        
        self.get_logger().info("Stopping RealSense pipeline...")
        try:
            self.pipeline.stop()
        except Exception as e:
            self.get_logger().warn(f"Error stopping pipeline: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerFitOnly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
