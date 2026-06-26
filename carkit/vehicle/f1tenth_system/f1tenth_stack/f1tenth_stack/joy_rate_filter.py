# MIT License
#
# Copyright (c) 2026 CARKit maintainers
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy


class JoyRateFilter(Node):
    def __init__(self) -> None:
        super().__init__('joy_rate_filter')

        self.declare_parameter('input_topic', 'joy_device')
        self.declare_parameter('output_topic', 'joy')
        self.declare_parameter('publish_rate', 20.0)

        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')
        self._min_period = Duration(seconds=1.0 / publish_rate)

        joy_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._last_publish_time = None
        self.create_subscription(
            Joy,
            str(self.get_parameter('input_topic').value),
            self._on_joy,
            joy_qos,
        )
        self._publisher = self.create_publisher(
            Joy,
            str(self.get_parameter('output_topic').value),
            joy_qos,
        )

    def _on_joy(self, msg: Joy) -> None:
        now = self.get_clock().now()
        if (
            self._last_publish_time is not None
            and now - self._last_publish_time < self._min_period
        ):
            return

        self._last_publish_time = now

        output = Joy()
        output.header = msg.header
        output.header.stamp = now.to_msg()
        output.axes = list(msg.axes)
        output.buttons = list(msg.buttons)
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyRateFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
