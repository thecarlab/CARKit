#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import re
from rclpy.parameter import Parameter

class DistanceMetricsNode(Node):
    def __init__(self):
        super().__init__('distance_metrics_node')
        
        # Declare parameter for object type to track
        self.declare_parameter('target_object_type', 'bottle')
        self.target_object_type = self.get_parameter('target_object_type').value
        
        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Publishers for distance metrics
        self.avg_dist_pub = self.create_publisher(Float32, '/average_distance', 10)
        self.min_dist_pub = self.create_publisher(Float32, '/min_distance', 10)
        self.max_dist_pub = self.create_publisher(Float32, '/max_distance', 10)
        self.median_dist_pub = self.create_publisher(Float32, '/median_distance', 10)
        
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
        
        self.cv_bridge = CvBridge()
        
        # Store object detection info
        self.object_bbox = None  # [x1, y1, x2, y2]
        self.depth_image = None
        self.image_width = None
        
        self.get_logger().info(f'Distance Metrics node started. Tracking {self.target_object_type}...')
    
    def parameter_callback(self, params):
        """Handle parameter updates"""
        for param in params:
            if param.name == 'target_object_type':
                self.target_object_type = param.value
                self.get_logger().info(f'Updated target_object_type to {self.target_object_type}')
        return True
    
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
            
            if confidence >= 0.4:  # Only process if confidence is at least 40%
                # Convert bbox string to list of floats
                self.object_bbox = [float(x) for x in bbox_str.split(',')]
                self.get_logger().debug(f'Found {self.target_object_type} at bbox: {self.object_bbox} with confidence: {confidence:.2%}')
            else:
                self.object_bbox = None
        else:
            self.object_bbox = None
    
    def depth_callback(self, msg):
        """Process depth image and publish distance metrics"""
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.image_width = self.depth_image.shape[1]
            
            # If we have an object detection, calculate distance metrics
            if self.object_bbox is not None:
                # Get integer coordinates of the bbox
                x1 = max(0, int(self.object_bbox[0]))
                y1 = max(0, int(self.object_bbox[1]))
                x2 = min(self.depth_image.shape[1], int(self.object_bbox[2]))
                y2 = min(self.depth_image.shape[0], int(self.object_bbox[3]))
                
                # Extract the depth values in the bbox region
                depth_region = self.depth_image[y1:y2, x1:x2]
                
                # Get valid depth values (non-zero) and convert from mm to m
                valid_depths = depth_region[depth_region > 0] / 1000.0
                
                if valid_depths.size > 0:
                    # Calculate metrics
                    avg_depth = np.mean(valid_depths)
                    min_depth = np.min(valid_depths)
                    max_depth = np.max(valid_depths)
                    median_depth = np.median(valid_depths)
                    
                    # Publish metrics
                    self.avg_dist_pub.publish(Float32(data=float(avg_depth)))
                    self.min_dist_pub.publish(Float32(data=float(min_depth)))
                    self.max_dist_pub.publish(Float32(data=float(max_depth)))
                    self.median_dist_pub.publish(Float32(data=float(median_depth)))
                    
                    self.get_logger().debug(
                        f'{self.target_object_type} distances (m): '
                        f'avg={avg_depth:.2f}, min={min_depth:.2f}, '
                        f'max={max_depth:.2f}, median={median_depth:.2f}'
                    )
        
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = DistanceMetricsNode()
    
    # Override parameter if provided via command line
    if args is not None:
        from rclpy.parameter import Parameter
        import sys
        import argparse
        
        parser = argparse.ArgumentParser()
        parser.add_argument('--target-object-type', type=str, help='Type of object to track')
        parser.add_argument('ros_args', nargs=argparse.REMAINDER)
        
        parsed_args = parser.parse_args(args if args is not None else sys.argv[1:])
        
        if parsed_args.target_object_type:
            node.set_parameters([Parameter('target_object_type', Parameter.Type.STRING, parsed_args.target_object_type)])
            node.get_logger().info(f'Target object type set from command line: {parsed_args.target_object_type}')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 