#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import numpy as np
import re

class Demo1Node(Node):
    def __init__(self):
        super().__init__('demo1_node')
        
        # Declare parameters with default values
        self.declare_parameter('target_distance', 0.5)  # meters
        self.declare_parameter('speed_ratio', 1.5)  # multiplier for speed calculation
        
        # Get initial parameter values
        self.target_distance = self.get_parameter('target_distance').value
        self.speed_ratio = self.get_parameter('speed_ratio').value
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Publisher for Ackermann commands
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            '/ackermann_cmd',
            10
        )
        
        # Subscribe to pcl_pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pcl_pose',
            self.pose_callback,
            10
        )
        
        # Subscribe to YOLO detections
        self.yolo_sub = self.create_subscription(
            String,
            '/yolo/detections',
            self.yolo_callback,
            10
        )
        
        # Subscribe to depth image
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        self.current_pose = None
        self.cv_bridge = CvBridge()
        
        # Store bottle detection info
        self.bottle_bbox = None  # [x1, y1, x2, y2]
        self.bottle_distance = None  # in meters
        self.depth_image = None
        
        # Timer to publish commands
        self.cmd_timer = self.create_timer(0.1, self.publish_command)  # 10Hz
        
        self.get_logger().info('Demo1 node started. Tracking bottle...')
    
    def pose_callback(self, msg):
        """Handle incoming pcl_pose messages"""
        self.current_pose = msg.pose.pose
    
    def yolo_callback(self, msg):
        """Process YOLO detections to find apple"""
        # Extract all detections using regex
        detections = re.findall(r'(\w+) \[([\d\.,\s]+)\] \(([\d\.]+)\)', msg.data)
        
        # Look for bottle detection
        for obj_type, bbox_str, conf in detections:
            if obj_type == 'apple':
                # Convert confidence to float and check threshold
                confidence = float(conf)
                if confidence < 0.45:  # Skip if confidence is less than 60%
                    self.get_logger().debug(f'Skipping low confidence apple detection: {confidence:.2%}')
                    continue
                    
                # Convert bbox string to list of floats
                bbox = [float(x) for x in bbox_str.split(',')]
                self.bottle_bbox = bbox
                self.get_logger().debug(f'Found bottle at bbox: {bbox} with confidence: {confidence:.2%}')
                break
        else:
            self.bottle_bbox = None
    
    def depth_callback(self, msg):
        """Process depth image"""
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # If we have a bottle detection, get its distance
            if self.bottle_bbox is not None:
                # Get integer coordinates of the bbox
                x1 = max(0, int(self.bottle_bbox[0]))
                y1 = max(0, int(self.bottle_bbox[1]))
                x2 = min(self.depth_image.shape[1], int(self.bottle_bbox[2]))
                y2 = min(self.depth_image.shape[0], int(self.bottle_bbox[3]))
                
                # Extract the depth values in the bbox region
                depth_region = self.depth_image[y1:y2, x1:x2]
                
                # Find minimum non-zero depth value (convert from mm to m)
                valid_depths = depth_region[depth_region > 0] / 1000.0  # mm to m
                if valid_depths.size > 0:
                    self.bottle_distance = np.min(valid_depths)
                    self.get_logger().info(f'Bottle minimum distance: {self.bottle_distance:.2f}m')
        
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def parameter_callback(self, params):
        """Handle parameter updates"""
        for param in params:
            if param.name == 'target_distance':
                self.target_distance = param.value
                self.get_logger().info(f'Updated target_distance to {self.target_distance}m')
            elif param.name == 'speed_ratio':
                self.speed_ratio = param.value
                self.get_logger().info(f'Updated speed_ratio to {self.speed_ratio}')
        return True

    def publish_command(self):
        """Publish Ackermann command based on bottle distance"""
        if self.bottle_distance is not None:
            # Stop if we're within 5cm of target distance
            if abs(self.bottle_distance - self.target_distance) < 0.05:
                cmd = AckermannDriveStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = "map"
                cmd.drive.speed = 0.0
                cmd.drive.steering_angle = 0.0
                
                self.cmd_pub.publish(cmd)
                self.get_logger().info(f'At target distance ({self.bottle_distance:.2f}m), stopping')
                self.bottle_distance = None
                return

            # Calculate speed based on bottle distance using parameters
            speed = (self.bottle_distance - self.target_distance) * self.speed_ratio
            
            self.get_logger().info(f'Speed before clamping: distance={self.bottle_distance:.2f}m, speed={speed:.2f}m/s')
            if abs(speed) < 0.25:
                speed = 0.0
            else:
                speed = np.clip(abs(speed), 0.25, 1.0) * np.sign(speed)
            
            # Create and publish command
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "map"
            cmd.drive.speed = float(speed)
            cmd.drive.steering_angle = 0.0  # No steering for now
            
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'Published command: distance={self.bottle_distance:.2f}m, speed={speed:.2f}m/s')
            self.bottle_distance = None

        else:
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "map"
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = 0.0  # No steering for now
            
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'No apple detected')

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
