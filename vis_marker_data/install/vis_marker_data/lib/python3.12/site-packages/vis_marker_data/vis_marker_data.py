import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import math
import sys
import csv

class WaypointMarkerPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_marker_publisher')
        
        # Create a publisher for the MarkerArray
        self.publisher_ = self.create_publisher(MarkerArray, 'waypoints_markers', 10)
        
        # Publish at 1 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Replace this with your actual x, y, heading (in radians) data
        self.load_waypoints("/home/matthew/PurePursuitWaypoints/8_track.csv")

    def load_waypoints(self, filename):
        # Load x, y, yaw from csv
        with open(filename) as file:
            reader = csv.reader(file)
            self.waypoints = [tuple(map(float, row)) for row in reader]
        
        temp = [(x, y, yaw) for (_, x, y, yaw) in self.waypoints]
        self.waypoints = temp


    def euler_to_quaternion(self, yaw_):
        yaw_ = yaw_ - 90
        yaw = yaw_ * 3.14159265 / 180
        # Converts a 2D yaw angle to a 3D quaternion
        qw = math.cos(yaw * 0.5)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw * 0.5)
        return qx, qy, qz, qw

    def timer_callback(self):
        marker_array = MarkerArray()

        for i, (x, y, heading) in enumerate(self.waypoints):
            marker = Marker()
            
            # The frame ID must match your fixed frame in RViz!
            marker.header.frame_id = "map" 
            marker.header.stamp = self.get_clock().now().to_msg()
            
            marker.ns = "waypoints"
            marker.id = i
            
            # Using an ARROW marker to show both position and heading
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Set the position
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.0
            
            # Set the orientation using our quaternion conversion
            qx, qy, qz, qw = self.euler_to_quaternion(heading)
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            
            # Scale the arrow (x is length, y is width, z is height)
            marker.scale.x = 0.05  
            marker.scale.y = 0.01  
            marker.scale.z = 0.01  
            
            # Set the color (RGBA, values range from 0.0 to 1.0)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0 # 1.0 means fully opaque
            
            marker_array.markers.append(marker)
            
        self.publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()