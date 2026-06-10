#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA
import math
import numpy as np
from collections import deque

class InteractiveWaypointsNode(Node):
    def __init__(self):
        super().__init__('interactive_waypoints_node')
        
        # Initialize interactive marker server
        self.server = InteractiveMarkerServer(self, 'waypoint_marker')
        
        # Publisher for the path
        self.path_pub = self.create_publisher(Path, '/follow_path', 10)
        
        # Subscribe to pcl_pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pcl_pose',
            self.pose_callback,
            10
        )
        
        # Storage for waypoints (limit to 5)
        self.waypoints = deque(maxlen=5)
        self.last_waypoint_position = None
        self.waypoint_distance_threshold = 0.5  # 0.5m between waypoints
        self.current_pose = None
        
        # Wait for first pcl_pose before creating marker
        self.get_logger().info('Waiting for initial pcl_pose...')
        
        # Timer to check for initial pose and create marker
        self.init_timer = self.create_timer(0.1, self.init_callback)
        self.marker_created = False
        
        # Timer to publish path periodically
        self.timer = self.create_timer(0.1, self.publish_path)  # 10Hz
    
    def pose_callback(self, msg):
        """Handle incoming pcl_pose messages"""
        self.current_pose = msg.pose.pose  # Extract the pose from PoseWithCovarianceStamped
        
        # If this is the first pose and marker isn't created yet, create it
        if not self.marker_created and self.current_pose is not None:
            self.create_interactive_marker()
            self.marker_created = True
            
            # Add initial waypoint at current position
            self.add_waypoint(self.current_pose.position, self.current_pose.orientation)
    
    def init_callback(self):
        """Check if we've received initial pose"""
        if self.current_pose is not None and not self.marker_created:
            self.create_interactive_marker()
            self.marker_created = True
            self.init_timer.cancel()  # Stop checking
            self.get_logger().info('Interactive marker created at current pose')
    
    def create_interactive_marker(self):
        """Create an interactive marker that can be dragged to create waypoints"""
        if self.current_pose is None:
            self.get_logger().warn('No current pose available yet')
            return
            
        # Create interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = "waypoint_creator"
        int_marker.description = "Drag to create waypoints"
        
        # Set initial position to current pose
        int_marker.pose.position = self.current_pose.position
        int_marker.pose.orientation = self.current_pose.orientation
        
        # Create the visual marker (a sphere)
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Create control for the marker
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
        
        # Create control for moving the marker
        control = InteractiveMarkerControl()
        control.name = "move_xy"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        int_marker.controls.append(control)
        
        # Add the marker to the server
        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.marker_feedback)
        self.server.applyChanges()
    
    def marker_feedback(self, feedback):
        """Handle feedback from the interactive marker"""
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.get_logger().info('Marker grabbed')
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.get_logger().info('Marker released')
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Get current marker position
            current_pos = feedback.pose.position
            
            # Check if we should add a new waypoint
            if self.should_add_waypoint(current_pos):
                self.add_waypoint(current_pos, feedback.pose.orientation)
                self.get_logger().info(f'Added waypoint at ({current_pos.x:.2f}, {current_pos.y:.2f})')
                
                # Update markers for all waypoints (to maintain correct numbering)
                self.update_waypoint_markers()
        
        self.server.applyChanges()
    
    def should_add_waypoint(self, current_pos):
        """Determine if a new waypoint should be added based on distance"""
        if self.last_waypoint_position is None:
            return True
        
        # Calculate distance from last waypoint
        distance = math.sqrt(
            (current_pos.x - self.last_waypoint_position.x) ** 2 +
            (current_pos.y - self.last_waypoint_position.y) ** 2
        )
        
        return distance >= self.waypoint_distance_threshold
    
    def add_waypoint(self, position, orientation):
        """Add a new waypoint to the list"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position = position
        pose_stamped.pose.orientation = orientation
        
        # Add to deque (automatically maintains max length of 5)
        self.waypoints.append(pose_stamped)
        self.last_waypoint_position = position
    
    def update_waypoint_markers(self):
        """Update all waypoint markers to maintain correct numbering"""
        # First, clear all existing waypoint markers
        for i in range(len(self.waypoints)):
            marker_name = f"waypoint_{i}"
            self.server.erase(marker_name)
        
        # Create new markers for current waypoints
        for i, waypoint in enumerate(self.waypoints):
            self.create_waypoint_marker(i, waypoint.pose.position)
        
        self.server.applyChanges()
    
    def create_waypoint_marker(self, waypoint_id, position):
        """Create a visual marker for a waypoint"""
        marker_name = f"waypoint_{waypoint_id}"
        
        # Create interactive marker for the waypoint
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = marker_name
        int_marker.description = f"Waypoint {waypoint_id + 1}"
        int_marker.pose.position = position
        int_marker.pose.orientation.w = 1.0
        
        # Create the visual marker (a small cylinder)
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # Create control for the marker
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
        
        # Add text label
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.scale.z = 0.2
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = str(waypoint_id + 1)
        text_marker.pose.position.z = 0.3
        
        control_text = InteractiveMarkerControl()
        control_text.always_visible = True
        control_text.markers.append(text_marker)
        int_marker.controls.append(control_text)
        
        # Add the waypoint marker to the server
        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.waypoint_feedback)
    
    def waypoint_feedback(self, feedback):
        """Handle feedback from waypoint markers"""
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # Could implement waypoint deletion here
            pass
    
    def publish_path(self):
        """Publish the current path"""
        if len(self.waypoints) > 0:
            path = Path()
            path.header.frame_id = "map"
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = list(self.waypoints)  # Convert deque to list
            
            self.path_pub.publish(path)
    
    def clear_waypoints(self):
        """Clear all waypoints"""
        self.waypoints.clear()
        self.last_waypoint_position = None
        
        # Remove all waypoint markers
        for i in range(len(self.waypoints)):
            marker_name = f"waypoint_{i}"
            self.server.erase(marker_name)
        
        self.server.applyChanges()
        self.get_logger().info('All waypoints cleared')

def main(args=None):
    rclpy.init(args=args)
    
    node = InteractiveWaypointsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
