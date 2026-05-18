#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
import numpy as np
import math

class PathTrackerNode(Node):
    def __init__(self):
        super().__init__('path_tracker_node')
        
        # Declare parameters
        self.declare_parameter('waypoint_distance', 0.5)  # meters between waypoints
        
        # Get parameter values
        self.waypoint_distance = self.get_parameter('waypoint_distance').value
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/object_path', 10)
        self.waypoints_pub = self.create_publisher(PoseArray, '/object_waypoints', 10)
        
        # Subscribe to object position
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/object_position',
            self.position_callback,
            10
        )
        
        # Initialize path and waypoints
        self.path = Path()
        self.path.header.frame_id = "camera_link"
        
        self.waypoints = PoseArray()
        self.waypoints.header.frame_id = "camera_link"
        
        # Store last waypoint position
        self.last_waypoint = None
        
        self.get_logger().info(
            f'Path Tracker node started. '
            f'Adding waypoints every {self.waypoint_distance}m'
        )
    
    def parameter_callback(self, params):
        """Handle parameter updates"""
        for param in params:
            if param.name == 'waypoint_distance':
                self.waypoint_distance = param.value
                self.get_logger().info(f'Updated waypoint_distance to {self.waypoint_distance}m')
        return True
    
    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def position_callback(self, msg):
        """Process new object position"""
        # Update path
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(msg)
        
        # Publish updated path
        self.path_pub.publish(self.path)
        
        # Check if we need to add a new waypoint
        if (self.last_waypoint is None or 
            self.calculate_distance(msg.pose, self.last_waypoint) >= self.waypoint_distance):
            
            # Add new waypoint
            self.waypoints.header.stamp = msg.header.stamp
            self.waypoints.poses.append(msg.pose)
            self.last_waypoint = msg.pose
            
            # Publish updated waypoints
            self.waypoints_pub.publish(self.waypoints)
            
            self.get_logger().debug(
                f'Added new waypoint at '
                f'x={msg.pose.position.x:.2f}m, '
                f'y={msg.pose.position.y:.2f}m'
            )
        
        # Keep path length reasonable (last 100 poses)
        if len(self.path.poses) > 100:
            self.path.poses = self.path.poses[-100:]

def main(args=None):
    rclpy.init(args=args)
    
    node = PathTrackerNode()
    
    # Override parameter if provided via command line
    if args is not None:
        from rclpy.parameter import Parameter
        import sys
        import argparse
        
        parser = argparse.ArgumentParser()
        parser.add_argument('--waypoint-distance', type=float, help='Distance between waypoints in meters')
        parser.add_argument('ros_args', nargs=argparse.REMAINDER)
        
        parsed_args = parser.parse_args(args if args is not None else sys.argv[1:])
        
        if parsed_args.waypoint_distance:
            node.set_parameters([Parameter('waypoint_distance', Parameter.Type.DOUBLE, parsed_args.waypoint_distance)])
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 