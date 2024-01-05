#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        timer_period = 1.0  # seconds (adjust as needed)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.marker_array = MarkerArray()
        self.initialize_marker_array()

    def initialize_marker_array(self):

        path = '/home/klein/ws/src/hack/config/' + 'challenge1_waypoints_01.csv'


        # Read waypoints from CSV file and store in marker array
        with open(path, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            next(csvreader)  # Skip the header row
            id = 0
            for row in csvreader:
                # Create a new marker for each waypoint
                marker = Marker()
                marker.header.frame_id = "map"
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale.x = 1.5
                marker.scale.y = 1.5
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                # Extract the x, y, z values from the last three columns
                x, y, z = map(float, row[-3:])
                marker.pose.position = Point(x=x, y=y, z=z)
                
                marker.id = id
                id += 1
                
                self.marker_array.markers.append(marker)

    def timer_callback(self):
        # Update the timestamp for each marker
        for marker in self.marker_array.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
