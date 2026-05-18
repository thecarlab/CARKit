#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import re
import math

class ObjectAngleNode(Node):
    def __init__(self):
        super().__init__('object_angle_node')
        
        # Declare parameters
        self.declare_parameter('target_object_type', 'bottle')
        self.declare_parameter('horizontal_fov', 69.4)  # degrees
        
        # Get parameter values
        self.target_object_type = self.get_parameter('target_object_type').value
        self.horizontal_fov = self.get_parameter('horizontal_fov').value
        
        # Convert FOV to radians for calculations
        self.horizontal_fov_rad = math.radians(self.horizontal_fov)
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Publisher for object angle (in degrees)
        self.angle_pub = self.create_publisher(Float32, '/object_angle', 10)
        
        # Subscribe to YOLO detections
        self.yolo_sub = self.create_subscription(
            String,
            '/yolo/detections',
            self.yolo_callback,
            10
        )
        
        # Subscribe to depth image (for resolution)
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        self.cv_bridge = CvBridge()
        self.image_width = None
        self.image_height = None
        self.resolution_printed = False
        
        self.get_logger().info(
            f'Object Angle node started. Tracking {self.target_object_type} '
            f'with {self.horizontal_fov}° horizontal FOV'
        )
    
    def parameter_callback(self, params):
        """Handle parameter updates"""
        for param in params:
            if param.name == 'target_object_type':
                self.target_object_type = param.value
                self.get_logger().info(f'Updated target_object_type to {self.target_object_type}')
            elif param.name == 'horizontal_fov':
                self.horizontal_fov = param.value
                self.horizontal_fov_rad = math.radians(self.horizontal_fov)
                self.get_logger().info(f'Updated horizontal_fov to {self.horizontal_fov}°')
        return True
    
    def calculate_angle(self, center_x):
        """Calculate angle to object based on its center x-coordinate"""
        if self.image_width is None:
            return None
            
        # Convert pixel position to normalized position (-0.5 to 0.5)
        normalized_pos = (center_x / self.image_width) - 0.5
        
        # Calculate angle in degrees (positive is right, negative is left)
        angle_rad = (normalized_pos / 0.5) * (self.horizontal_fov_rad / 2)
        angle_deg = math.degrees(angle_rad)
        
        return angle_deg
    
    def yolo_callback(self, msg):
        """Process YOLO detections to find specified object"""
        # Extract all detections using regex
        detections = re.findall(r'(\w+) \[([\d\.,\s]+)\] \(([\d\.]+)\)', msg.data)
        
        # Filter detections for target object type
        target_detections = [
            (bbox_str, float(conf)) 
            for obj_type, bbox_str, conf in detections 
            if obj_type == self.target_object_type
        ]
        
        if target_detections:
            # Get the detection with highest confidence
            best_detection = max(target_detections, key=lambda x: x[1])
            bbox_str, confidence = best_detection
            
            if confidence >= 0.2:  # Only process if confidence is at least 20%
                # Convert bbox string to list of floats
                bbox = [float(x) for x in bbox_str.split(',')]
                
                # Calculate center x-coordinate
                center_x = (bbox[0] + bbox[2]) / 2
                
                # Calculate angle
                angle = self.calculate_angle(center_x) * -1
                
                if angle is not None:
                    # Publish angle in degrees
                    msg = Float32()
                    msg.data = float(angle)
                    self.angle_pub.publish(msg)
                    
                    self.get_logger().debug(
                        f'Found {self.target_object_type} at angle: {angle:.1f}° '
                        f'(confidence: {confidence:.2%})'
                    )
    
    def depth_callback(self, msg):
        """Get image resolution from depth image"""
        if not self.resolution_printed:
            self.image_height = msg.height
            self.image_width = msg.width
            self.get_logger().info(
                f'Image resolution: {self.image_width}x{self.image_height}'
            )
            self.resolution_printed = True

def main(args=None):
    rclpy.init(args=args)
    
    node = ObjectAngleNode()
    
    # Override parameter if provided via command line
    if args is not None:
        from rclpy.parameter import Parameter
        import sys
        import argparse
        
        parser = argparse.ArgumentParser()
        parser.add_argument('--target-object-type', type=str, help='Type of object to track')
        parser.add_argument('--horizontal-fov', type=float, help='Horizontal field of view in degrees')
        parser.add_argument('ros_args', nargs=argparse.REMAINDER)
        
        parsed_args = parser.parse_args(args if args is not None else sys.argv[1:])
        
        if parsed_args.target_object_type:
            node.set_parameters([Parameter('target_object_type', Parameter.Type.STRING, parsed_args.target_object_type)])
        if parsed_args.horizontal_fov:
            node.set_parameters([Parameter('horizontal_fov', Parameter.Type.DOUBLE, parsed_args.horizontal_fov)])
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
