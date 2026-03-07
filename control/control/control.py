#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import csv
import os
from transforms3d.euler import quat2euler
from time import time

# ROS Messages
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

# Tuning parameters for Pure Pursuit and PID speed control

# aggressive snap back or drifting wide                     > increase LFC, L_min
# overshoot tight corners / cuts corner sharply             > k_curve
# too slow / fast in corners                                > k_speed, k_curve, MIN_SPEED, MAX_SPEED
# oscillates on straights                                   > LFC, KP
# slow to reach target speed                                > KP, KI
# brakes too late before corners                            > k_speed, previous lookahead steps


class ReactivePurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # --- TUNING PARAMETERS ---
        self.k = 0.4                    # Lookahead distance gain
        self.LFC = 2.0                  # Base Lookahead distance - INCREASED for dense waypoints
        self.MAX_SPEED = 2.5            # Maximum speed (m/s) - reduced for safety  
        self.MIN_SPEED = 0.5            # Minimum speed in corners
        self.KP = 1.0                   # Speed P-Gain
        self.KI = 0.1                   # Speed I-Gain
        self.KD = 0.05                  # Speed D-Gain
        self.WB = 0.33                  # Wheelbase
        self.k_speed = 3.0              # curvature speed reduction gain
        self.k_curve = 1.5              # curvature lookahead reduction gain
                                        # higher means more aggressive slowing in corners

        # --- STATE VARIABLES ---
        self.speed = 0.0                # current speed   
        self.target_speed = 2.0         # target speed
        self.x = 0.0                    # current x position
        self.y = 0.0                    # current y position
        self.yaw = 0.0                  # current heading
        
        # PID Error Storage
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time()                # Last control loop time

        # --- WAYPOINT LOADING ---
        self.waypoints = self.load_waypoints()
        self.old_nearest_idx = None
        self.average_spacing = self.compute_average_spacing()
        self.L_min = 1.5
        self.L_max = 4.0
        
        self._log_counter = 0  # Counter for logging frequency

        if len(self.waypoints) == 0:
            self.get_logger().error("No waypoints loaded! Check CSV file.")
        else:
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")

        # --- SUBSCRIBERS & PUBLISHERS ---
        
        # 1. Updates current speed and pose
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)

        # 2. Publishes drive commands
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

        self.get_logger().info("Waypoint-based Pure Pursuit Node Initialized")
    
    def load_waypoints(self):
        """Load waypoints from CSV file (no header format)"""
        # Try to find the CSV file in the workspace
        csv_path = os.path.join(os.path.dirname(__file__), '..', '8_track_shifted.csv')
        
        if not os.path.exists(csv_path):
            # Try alternative path
            csv_path = os.path.expanduser('~/f1tenth_ws/src/control/8_track_shifted.csv')
        
        waypoints = []
        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) == 4:
                        # Format: index, x, y, heading
                        idx = int(row[0])
                        x = float(row[1])
                        y = float(row[2])
                        heading = float(row[3])
                    elif len(row) == 3:
                        # Format: x, y, heading
                        x = float(row[0])
                        y = float(row[1])
                        heading = float(row[2])
                    else:
                        self.get_logger().warn(f"Skipping invalid row: {row}")
                        continue
                    waypoints.append([x, y, heading])
            
            self.get_logger().info(f"Loaded {len(waypoints)} waypoints from: {csv_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
        
        waypoints = np.array(waypoints)
    
        # Check if path is already closed
        gap = math.hypot(
            waypoints[-1, 0] - waypoints[0, 0],
            waypoints[-1, 1] - waypoints[0, 1]
        )
        avg_spacing = float(np.mean(np.hypot(
            np.diff(waypoints[:, 0]),
            np.diff(waypoints[:, 1])
        )))

        if gap > avg_spacing * 5:
        # Interpolate waypoints to bridge the gap smoothly
            n_bridge = max(2, int(gap / avg_spacing))
            for i in range(1, n_bridge):
                t = i / n_bridge
                bx = waypoints[-1, 0] + t * (waypoints[0, 0] - waypoints[-1, 0])
                by = waypoints[-1, 1] + t * (waypoints[0, 1] - waypoints[-1, 1])
                bh = waypoints[-1, 2] + t * (waypoints[0, 2] - waypoints[-1, 2])
                waypoints = np.vstack([waypoints, [bx, by, bh]])
        
        return np.array(waypoints)
    
    def compute_average_spacing(self):
        if len(self.waypoints) < 2: return 0.5
        diffs = np.diff(self.waypoints[:, :2], axis=0)
        distances = np.hypot(diffs[:, 0], diffs[:, 1])
        return float(np.mean(distances))

    def odom_callback(self, msg):
        """ Update current speed and pose from the car's odometry """
        self.speed = msg.twist.twist.linear.x
        
        # Update position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Update yaw (heading)
        q = msg.pose.pose.orientation
        (roll, pitch, yaw) = quat2euler([q.w, q.x, q.y, q.z])
        self.yaw = yaw
        
        # Now run the control loop
        self.control_loop()

    def get_distance(self, idx):
            wp = self.waypoints[idx % len(self.waypoints)]
            return math.hypot(wp[0] - self.x, wp[1] - self.y)

    def compute_path_curvature(self, index, lookahead_steps=10):
        """ Estimate curvature of path ahead using lookahead waypoints """
        if len(self.waypoints) < 3: return 0.0

        end_index = (index + lookahead_steps) % len(self.waypoints)
        start_index = index

        p1 = self.waypoints[start_index, :2]
        p2 = self.waypoints[end_index, :2]

        arc_len = lookahead_steps * self.average_spacing
        if arc_len < 1e-6: return 0.0
        
        p1_next = self.waypoints[(start_index + 1) % len(self.waypoints)]
        p2_next = self.waypoints[(end_index + 1) % len(self.waypoints)]

        h1 = math.atan2(p1_next[1] - p1[1], p1_next[0] - p1[0])
        h2 = math.atan2(p2_next[1] - p2[1], p2_next[0] - p2[0])
        dh = abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))

        curvature = dh / arc_len
        return curvature
    

    def find_closest_waypoint(self):
        if self.old_nearest_idx is None:
            dx = self.waypoints[:, 0] - self.x
            dy = self.waypoints[:, 1] - self.y
            d = np.hypot(dx, dy)
            index = np.argmin(d)
            self.old_nearest_idx = index
        
        index = self.old_nearest_idx
        step = 0

        while step < len(self.waypoints):
            next_idx = (index + 1) % len(self.waypoints)
            if self.get_distance(next_idx) > self.get_distance(index):
                break
            index = next_idx
            step += 1
        self.old_nearest_idx = index
        return index

        
    def find_lookahead_waypoint(self):
        # Find nearest waypoint BEHIND or at current position
        closest_waypoint = self.find_closest_waypoint()

        # Adjust lookahead distance based on curvature 
        # more curvature (tighter turn) -> shorter lookahead to slow down
        curvature = self.compute_path_curvature(closest_waypoint, lookahead_steps=15)
        LF = (self.LFC + (abs(self.speed) * self.k)) / (1.0 + self.k_curve * curvature)
        LF = np.clip(LF, self.L_min, self.L_max)  # Clamp lookahead distance based on path density

        # Search forward from nearest point for lookahead point
        search_idx = closest_waypoint
        min_skip = max(1, int(self.L_min / (self.average_spacing + 1e-6)))  # Skip some waypoints for dense tracks
        
        for i in range(min_skip, len(self.waypoints)):
            test_idx = (closest_waypoint + i) % len(self.waypoints)
            dist = self.get_distance(test_idx)
            if dist >= LF:
                search_idx = test_idx
                break
        
        return LF, search_idx, self.waypoints[search_idx, 0], self.waypoints[search_idx, 1]

    
    def control_loop(self):
        """ 
        The Main Control Loop 
        1. Find lookahead target waypoint
        2. Calculate Steering (Pure Pursuit)
        3. Calculate Throttle (PID Speed Control)
        4. Publish Drive Command
        """
        
        if len(self.waypoints) == 0: return
        
        # --- STEP 1: FIND TARGET WAYPOINT ---
        LF, target_idx, target_x, target_y = self.find_lookahead_waypoint()                          # Visualize current target waypoint
        
        # distance from car to target in global frame
        dx = target_x - self.x
        dy = target_y - self.y
        
        # Rotate to car's reference frame
        delta_x = dx * math.cos(-self.yaw) - dy * math.sin(-self.yaw)
        delta_y = dx * math.sin(-self.yaw) + dy * math.cos(-self.yaw)
        
        # PURE PURSUIT STEERING formula: delta = atan(2*L*sin(alpha)/Ld)
        # Simplified: delta = atan(2*L*dy/Ld^2) where dy is lateral error in car frame
        Ld = math.sqrt(delta_x**2 + delta_y**2)
        Ld = max(Ld, 0.1)  # Prevent division by zero
        
        steering_angle = math.atan2(2.0 * self.WB * delta_y, Ld**2)
        steering_angle = np.clip(steering_angle, -0.41, 0.41) # clamp steering

        # --- STEP 3: ADAPTIVE SPEED CONTROL ---

        closest_index = self.find_closest_waypoint()
        preview_curvature = self.compute_path_curvature(closest_index, lookahead_steps=25)
        predictive_speed = self.MAX_SPEED / (1.0 + self.k_speed * abs(preview_curvature))
        self.target_speed = np.clip(predictive_speed, self.MIN_SPEED, self.MAX_SPEED)

        # Slow down in corners based on steering angle
        """
        steering_ratio = abs(steering_angle) / 0.41  # 0 to 1
        steering_speed = self.MAX_SPEED - (self.MAX_SPEED - self.MIN_SPEED) * steering_ratio
        self.target_speed = np.clip(steering_speed, self.MIN_SPEED, self.MAX_SPEED)  # Ensure minimum speed in corners    
        """


        error = self.target_speed - self.speed
        p = self.KP * error # proportional term

        current_time = time()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.integral += error * dt  # integral term
        self.integral = np.clip(self.integral, -2.0, 2.0)
        i = self.KI * self.integral

        dt = max(dt, 0.01)                              # prevent division by zero
        d = self.KD * (error - self.prev_error) / dt # derivative term
        
        self.prev_error = error
        speed = self.target_speed + p + i + d
        speed = np.clip(speed, 0.0, self.MAX_SPEED)


        # --- STEP 4: PUBLISH ---
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        
        self.drive_pub.publish(drive_msg)
        
        # Log progress occasionally
        self._log_counter += 1 
        if self._log_counter % 20 == 0:
            self.get_logger().info(
                f"Pos: ({self.x:.2f}, {self.y:.2f}), Target WP[{target_idx}]: ({target_x:.2f}, {target_y:.2f}), "
                f"CarFrame: dx={delta_x:.2f}, dy={delta_y:.2f}, Ld={Ld:.2f}, "
                f"Steering: {steering_angle:.3f} rad ({math.degrees(steering_angle):.1f}°)"
            )

def main(args=None):
    rclpy.init(args=args)
    node = ReactivePurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()