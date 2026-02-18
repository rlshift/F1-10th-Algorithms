import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')
        
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub = self.create_subscription(
            LaserScan, 
            lidarscan_topic, 
            self.lidar_callback, 
            10
        )
        
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, 
            drive_topic, 
            10
        )

        # Parameters
        self.bubble_radius = 1.2 # Radius in meters to zero out around closest point
        self.preprocess_conv_size = 3 # Kernel size for preprocessing smoothing
        self.best_point_conv_size = 80 # Kernel size for smoothing end result
        self.max_lidar_range = 3 # Clip data above this distance (m)
        self.car_width = 0.5 # Safety width

        self.previous_steering_angle = 0.0

    def preprocess_lidar(self, ranges):
        ranges = np.array(ranges)
        # Clean NaNs and Infs
        ranges = np.nan_to_num(ranges, posinf=self.max_lidar_range, neginf=0.0)
        
        # Clip range (cap at max_lidar_range) so deep gaps don't skew the average
        ranges = np.clip(ranges, 0, self.max_lidar_range)
    
        return ranges

    def find_max_gap(self, free_space_ranges):
        # Return the start index & end index of the max gap in free_space_ranges
                
        # Mask the bubble (zeros) as False, gaps (non-zeros) as True
        mask = free_space_ranges > 0
        
        # Find runs of non-zero (gap)
        d = np.diff(np.concatenate(([0], mask.view(np.int8), [0])))
        start_indices = np.flatnonzero(d == 1)
        end_indices = np.flatnonzero(d == -1)
        
        # Calculate lengths of gaps
        gap_lengths = end_indices - start_indices
        
        if len(gap_lengths) == 0:
            return 0, len(free_space_ranges) - 1
            
        # Find index of largest gap
        max_idx = np.argmax(gap_lengths)
        
        return start_indices[max_idx], end_indices[max_idx] - 1

    def find_best_point(self, start_i, end_i, ranges):
        # Start_i & end_i are start and end indices of max-gap range, respectively
        # Return index of best point in ranges
        
        center_index = len(ranges) // 2

        safety_buffer = 10

        if(start_i + safety_buffer < center_index < end_i - safety_buffer):
            return center_index
        
        gap_slice = ranges[start_i:end_i]
        max_val_idx = np.argmax(gap_slice)


        return start_i + max_val_idx
        
    def lidar_callback(self, data):
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # Limit Field of View
        # Only care about gaps that are roughly in front of us.
        #  +/- 70 degrees (+/- 1.2 radians)
        min_angle = -1.2
        max_angle = 1.2
        
        # Calculate indices for the sliced view
        # angle = angle_min + i * increment  ->  i = (angle - angle_min) / increment
        start_idx = int((min_angle - data.angle_min) / data.angle_increment)
        end_idx = int((max_angle - data.angle_min) / data.angle_increment)
        
        # Check to keep indices inside array bounds
        start_idx = max(0, start_idx)
        end_idx = min(len(ranges)-1, end_idx)

        # Slice the array
        fov_ranges = proc_ranges[start_idx:end_idx]

        # Find closest point in our NEW sliced view
        closest_idx_in_fov = np.argmin(fov_ranges)
        min_dist = fov_ranges[closest_idx_in_fov]

        # Zero out bubble
        if min_dist < 0.001: min_dist = 0.001
        bubble_radius_indices = int(self.bubble_radius / (min_dist * data.angle_increment))
        
        start_zero = max(0, closest_idx_in_fov - bubble_radius_indices)
        end_zero = min(len(fov_ranges) - 1, closest_idx_in_fov + bubble_radius_indices)
        fov_ranges[start_zero : end_zero+1] = 0.0

        # Find Max Gap in the sliced array
        gap_start, gap_end = self.find_max_gap(fov_ranges)

        # Find Best Point in the sliced array
        best_point_idx_in_fov = self.find_best_point(gap_start, gap_end, fov_ranges)

        # Convert it back to the original array index to get the correct angle
        real_best_index = start_idx + best_point_idx_in_fov

        # Calculate Steering Angle
        steering_angle = data.angle_min + (real_best_index * data.angle_increment)

        # Clamp the Steering Angle
        # Even if the math says 0.8 rad, the car can only do ~0.4. 
        max_steering_angle = 0.4  # radians (approx 23 degrees)
        steering_angle = np.clip(steering_angle, -max_steering_angle, max_steering_angle)

        # Smooth the steering
        alpha = 0.1 # Lower = smoother but slower reaction
        steering_angle = (alpha * steering_angle) + ((1.0 - alpha) * self.previous_steering_angle)

        # If steering angle is very small, just go straight
        if (abs(steering_angle) % 1.0) < 0.005:
            steering_angle = 0 

        self.previous_steering_angle = steering_angle

        # Publish
        current_steering_abs = abs(steering_angle)
        max_speed = 4.0
        min_speed = 1.5
        max_turn = 0.4 # radians

        speed_ratio = np.clip(current_steering_abs / max_turn, 0.0, 1.0)
        speed = max_speed - (speed_ratio * (max_speed - min_speed))

        drive_msg = AckermannDriveStamped()
        drive_msg.header = data.header
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
