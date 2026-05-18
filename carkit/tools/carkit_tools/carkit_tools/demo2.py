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

class Demo2Node(Node):
    def __init__(self):
        super().__init__('demo2_node')
        
        # Declare parameters with default values
        self.declare_parameter('target_distance', 0.5)  # meters
        self.declare_parameter('speed_ratio', 1.5)  # multiplier for speed calculation
        self.declare_parameter('steering_ratio', 0.5)  # multiplier for steering calculation
        self.declare_parameter('max_steering_angle', 0.4)  # maximum steering angle in radians (~23 degrees)
        self.declare_parameter('camera_horizontal_fov', 1.089)  # RealSense horizontal FOV in radians (~62.5 degrees)
        
        # Get initial parameter values
        self.target_distance = self.get_parameter('target_distance').value
        self.speed_ratio = self.get_parameter('speed_ratio').value
        self.steering_ratio = self.get_parameter('steering_ratio').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.camera_horizontal_fov = self.get_parameter('camera_horizontal_fov').value
        
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
        self.bottle_center_x = None  # horizontal center of bottle in image
        self.image_width = None  # width of the image
        self.depth_image = None
        
        # Timer to publish commands
        self.cmd_timer = self.create_timer(0.033, self.publish_command)  # 30Hz  
        
        self.get_logger().info('Demo2 node started. Tracking bottle with steering control...')
    
    def pose_callback(self, msg):
        """Handle incoming pcl_pose messages"""
        self.current_pose = msg.pose.pose
    
    def parameter_callback(self, params):
        """Handle parameter updates"""
        for param in params:
            if param.name == 'target_distance':
                self.target_distance = param.value
                self.get_logger().info(f'Updated target_distance to {self.target_distance}m')
            elif param.name == 'speed_ratio':
                self.speed_ratio = param.value
                self.get_logger().info(f'Updated speed_ratio to {self.speed_ratio}')
            elif param.name == 'steering_ratio':
                self.steering_ratio = param.value
                self.get_logger().info(f'Updated steering_ratio to {self.steering_ratio}')
            elif param.name == 'max_steering_angle':
                self.max_steering_angle = param.value
                self.get_logger().info(f'Updated max_steering_angle to {self.max_steering_angle}')
            elif param.name == 'camera_horizontal_fov':
                self.camera_horizontal_fov = param.value
                self.get_logger().info(f'Updated camera_horizontal_fov to {self.camera_horizontal_fov} radians')
        return True
    
    def yolo_callback(self, msg):
        """Process YOLO detections to find bottle"""
        # Extract all detections using regex
        detections = re.findall(r'(\w+) \[([\d\.,\s]+)\] \(([\d\.]+)\)', msg.data)
        
        # Look for bottle detection
        for obj_type, bbox_str, conf in detections:
            if obj_type == 'apple':
                # Convert confidence to float and check threshold
                confidence = float(conf)
                if confidence < 0.4:  # Skip if confidence is less than 40%
                    self.get_logger().debug(f'Skipping low confidence bottle detection: {confidence:.2%}')
                    continue
                    
                # Convert bbox string to list of floats
                bbox = [float(x) for x in bbox_str.split(',')]
                self.bottle_bbox = bbox
                
                # Calculate horizontal center of bottle
                if self.image_width is not None:
                    self.bottle_center_x = (bbox[0] + bbox[2]) / 2
                    relative_pos = (self.bottle_center_x / self.image_width) - 0.5  # -0.5 to 0.5
                    self.get_logger().debug(f'Bottle center at {relative_pos:+.2%} from center')
                
                self.get_logger().debug(f'Found bottle at bbox: {bbox} with confidence: {confidence:.2%}')
                break
        else:
            self.bottle_bbox = None
            self.bottle_center_x = None
    
    def depth_callback(self, msg):
        """Process depth image"""
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.image_width = self.depth_image.shape[1]  # Update image width
            
            # If we have a bottle detection, get its distance
            if self.bottle_bbox is not None:
                # Get integer coordinates of the bbox
                x1 = max(0, int(self.bottle_bbox[0]))
                y1 = max(0, int(self.bottle_bbox[1]))
                x2 = min(self.depth_image.shape[1], int(self.bottle_bbox[2]))
                y2 = min(self.depth_image.shape[0], int(self.bottle_bbox[3]))
                
                # Calculate horizontal angle to bottle center
                bottle_center_x = (x1 + x2) / 2
                relative_pos = (bottle_center_x / self.image_width) - 0.5  # -0.5 to 0.5
                horizontal_angle = relative_pos * self.camera_horizontal_fov / 2  # angle in radians
                
                # Extract the depth values in the bbox region
                depth_region = self.depth_image[y1:y2, x1:x2]
                
                # Find minimum non-zero depth value (convert from mm to m)
                valid_depths = depth_region[depth_region > 0] / 1000.0  # mm to m
                if valid_depths.size > 0:
                    min_depth = np.min(valid_depths)
                    # Set bottle distance directly to min_depth
                    self.bottle_distance = min_depth
                    self.get_logger().info(
                        f'Bottle: depth={min_depth:.2f}m, '
                        f'angle={math.degrees(horizontal_angle):.1f}°'
                    )
        
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def publish_command(self):
        """Publish Ackermann command based on bottle position and distance"""
        if self.bottle_distance is not None and self.bottle_center_x is not None:
            # Calculate relative position first
            relative_pos = abs((self.bottle_center_x / self.image_width) - 0.5)
            
            # First check if we need to center the object
            if relative_pos > 0.1:
                # When off center, use fixed speed and steering to center
                speed = 0.5
                relative_steering_pos = (self.bottle_center_x / self.image_width) - 0.5
                steering = -relative_steering_pos * self.steering_ratio
                steering = np.clip(steering, -self.max_steering_angle, self.max_steering_angle)
                
                # Create and publish command
                cmd = AckermannDriveStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = "map"
                cmd.drive.speed = float(speed)
                cmd.drive.steering_angle = float(steering)
                
                self.cmd_pub.publish(cmd)
                self.get_logger().info(
                    f'Centering: rel_pos={relative_pos:.2f}, '
                    f'speed={speed:.2f}m/s, '
                    f'steering={math.degrees(steering):.1f}°'
                )
                return
                
            # If centered, check if we're at target distance
            if abs(self.bottle_distance - self.target_distance) < 0.05:
                cmd = AckermannDriveStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = "map"
                cmd.drive.speed = 0.0
                cmd.drive.steering_angle = 0.0
                
                self.cmd_pub.publish(cmd)
                self.get_logger().info(f'At target distance ({self.bottle_distance:.2f}m), stopping')
                return

            # If centered but not at target distance, adjust distance with no steering
            speed = (self.bottle_distance - self.target_distance) * self.speed_ratio
            speed = np.clip(abs(speed), 0.5, 0.75) * np.sign(speed)
            steering = 0.0
            
            # Create and publish command
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "map"
            cmd.drive.speed = float(speed)
            cmd.drive.steering_angle = float(steering)
            
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f'Adjusting distance: distance={self.bottle_distance:.2f}m, '
                f'speed={speed:.2f}m/s'
            )
        else:
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "map"
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = 0.0
            
            self.cmd_pub.publish(cmd)
            #self.get_logger().info('No apple detected')

def main(args=None):
    rclpy.init(args=args)
    
    node = Demo2Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
