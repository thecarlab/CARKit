#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_message_stamp', True)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.use_message_stamp = bool(self.get_parameter('use_message_stamp').value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20,
        )

        self.get_logger().info(
            f'Broadcasting {self.odom_frame} -> {self.base_frame} TF from {self.odom_topic}'
        )

    def odom_callback(self, msg):
        stamped = self._make_transform(msg)
        self.tf_broadcaster.sendTransform(stamped)

    def _make_transform(self, msg):
        stamped = TransformStamped()
        if self.use_message_stamp:
            stamped.header.stamp = msg.header.stamp
        else:
            stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.odom_frame or msg.header.frame_id
        stamped.child_frame_id = self.base_frame or msg.child_frame_id
        stamped.transform.translation.x = msg.pose.pose.position.x
        stamped.transform.translation.y = msg.pose.pose.position.y
        stamped.transform.translation.z = msg.pose.pose.position.z
        stamped.transform.rotation = msg.pose.pose.orientation
        return stamped


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
