#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class CmdVelToAckermannNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')
        
        # Create publisher for Ackermann commands
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped,
            '/ackermann_cmd',
            10
        )
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('cmd_vel to Ackermann converter node started')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to AckermannDriveStamped with direct mapping"""
        # Create Ackermann command message
        ackermann_cmd = AckermannDriveStamped()
        ackermann_cmd.header.stamp = self.get_clock().now().to_msg()
        ackermann_cmd.header.frame_id = "base_link"
        
        # Direct mapping
        ackermann_cmd.drive.speed = msg.linear.x
        ackermann_cmd.drive.steering_angle = msg.angular.z
        
        # Publish command
        self.ackermann_pub.publish(ackermann_cmd)
        self.get_logger().debug(
            f'Converting: linear.x={msg.linear.x:.2f} -> speed={ackermann_cmd.drive.speed:.2f}, '
            f'angular.z={msg.angular.z:.2f} -> steering={ackermann_cmd.drive.steering_angle:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    node = CmdVelToAckermannNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 