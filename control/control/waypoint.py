#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import csv
import os
from nav_msgs.msg import Odometry

# Helper function to get Yaw from Quaternion (so we don't need heavy libraries)
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return yaw_z # in radians

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        
        # 1. FILE SETUP
        # This saves 'raceline.csv' to your Home folder
        self.filename = os.path.expanduser('~/raceline.csv')
        self.file = open(self.filename, 'w')
        self.writer = csv.writer(self.file)
        
        # 2. SETTINGS
        self.min_distance = 0.1  # Only save if we moved 0.1 meters
        self.last_x = 0.0
        self.last_y = 0.0
        
        # 3. SUBSCRIBER
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.save_waypoint,
            10)
            
        self.get_logger().info(f"Logging waypoints to {self.filename}...")
        self.get_logger().info("Drive the car to record path!")

    def save_waypoint(self, msg):
        # Extract Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract Speed
        speed = msg.twist.twist.linear.x
        
        # Extract Yaw
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Check Distance (Avoid saving 1000 points in the same spot)
        distance = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)

        if distance > self.min_distance:
            # Write row: [x, y, speed, yaw]
            self.writer.writerow([x, y, speed, yaw])
            
            self.last_x = x
            self.last_y = y
            # Print to terminal occasionally so you know it's working
            # (We check integer distance to avoid spamming)
            if int(distance) % 1 == 0:
                self.get_logger().info(f"Saved Point: {x:.2f}, {y:.2f}")

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()