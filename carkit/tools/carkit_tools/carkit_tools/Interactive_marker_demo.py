#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
import math
import numpy as np

class Demo1Node(Node):
    def __init__(self):
        super().__init__('demo1_node')
        
        # Initialize interactive marker server
        self.server = InteractiveMarkerServer(self, 'demo1_marker')
        
        # Publisher for Ackermann commands
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            '/demo_cmd',
            10
        )
        
        # Subscribe to pcl_pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pcl_pose',
            self.pose_callback,
            10
        )
        
        self.current_pose = None
        self.marker_created = False
        
        # Timer to check for initial pose and create marker
        self.init_timer = self.create_timer(0.1, self.init_callback)
        
        # Timer to publish commands
        self.cmd_timer = self.create_timer(0.1, self.publish_command)  # 10Hz
        
        # Store marker pose
        self.marker_pose = None
        
        self.get_logger().info('Demo1 node started. Waiting for initial pcl_pose...')
    
    def pose_callback(self, msg):
        """Handle incoming pcl_pose messages"""
        self.current_pose = msg.pose.pose
        
        # If this is the first pose and marker isn't created yet, create it
        if not self.marker_created and self.current_pose is not None:
            self.create_marker()
            self.marker_created = True
    
    def init_callback(self):
        """Check if we've received initial pose"""
        if self.current_pose is not None and not self.marker_created:
            self.create_marker()
            self.marker_created = True
            self.init_timer.cancel()
            self.get_logger().info('Interactive marker created 2m ahead of current pose')
    
    def create_marker(self):
        """Create an interactive marker 2m ahead of current pose"""
        if self.current_pose is None:
            return
            
        # Create interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = "demo1_marker"
        int_marker.description = "Demo1 Target"
        
        # Position marker 2m ahead of current pose
        angle = math.atan2(2 * self.current_pose.orientation.w * self.current_pose.orientation.z,
                          1 - 2 * self.current_pose.orientation.z * self.current_pose.orientation.z)
        int_marker.pose.position.x = self.current_pose.position.x + 2.0 * math.cos(angle)
        int_marker.pose.position.y = self.current_pose.position.y + 2.0 * math.sin(angle)
        int_marker.pose.position.z = self.current_pose.position.z
        int_marker.pose.orientation = self.current_pose.orientation
        
        # Store initial marker pose
        self.marker_pose = int_marker.pose
        
        # Create the visual marker (a sphere)
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # Blue color
        marker.color.a = 1.0
        
        # Create control for the marker
        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(marker)
        int_marker.controls.append(visual_control)
        
        # Create control for moving the marker
        control = InteractiveMarkerControl()
        control.name = "move_xy"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.orientation.w = 1.0
        control.orientation.z = 1.0  # XY plane movement
        int_marker.controls.append(control)
        
        # Add the marker to the server
        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.marker_feedback)
        self.server.applyChanges()
    
    def marker_feedback(self, feedback):
        """Handle feedback from the interactive marker"""
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.marker_pose = feedback.pose
    
    def calculate_distance(self):
        """Calculate distance between current pose and marker"""
        if self.current_pose is None or self.marker_pose is None:
            return None
            
        dx = self.marker_pose.position.x - self.current_pose.position.x
        dy = self.marker_pose.position.y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def publish_command(self):
        """Publish Ackermann command based on distance"""
        distance = self.calculate_distance()
        if distance is None:
            return
            
        # Calculate speed based on distance
        # speed = (distance - 2) * 0.5
        # Clamp between 0.5 and 1.5
        speed = np.clip((distance - 2.0) * 0.5, 0.5, 1.5)
        
        # Create and publish command
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "map"
        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = 0.0  # No steering for now
        
        self.cmd_pub.publish(cmd)
        self.get_logger().debug(f'Published command: distance={distance:.2f}m, speed={speed:.2f}m/s')

def main(args=None):
    rclpy.init(args=args)
    
    node = Demo1Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 