"""F1TENTH/VESC odometry: measured ERPM + RealSense gyro, never commands."""

import math
import time
from collections import deque
from statistics import fmean, pstdev

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener
from vesc_msgs.msg import VescStateStamped

from .measured_odometry import MeasuredOdometry, rotate_vector


class RealSenseOdometry(Node):
    def __init__(self):
        super().__init__('realsense_wheel_odometry')
        self.gain = float(self.declare_parameter('speed_to_erpm_gain', 4614.0).value)
        self.offset = float(self.declare_parameter('speed_to_erpm_offset', 0.0).value)
        self.timeout = float(self.declare_parameter('feedback_timeout_sec', 0.25).value)
        self.max_speed = float(self.declare_parameter('max_measured_speed', 6.0).value)
        self.bias = float(self.declare_parameter('gyro_bias_z', 0.0).value)
        self.calibrate_bias = bool(self.declare_parameter('calibrate_gyro_bias', True).value)
        self.bias_samples = deque()
        self.bias_ready = not self.calibrate_bias
        if not all(map(math.isfinite, (self.gain, self.offset, self.timeout, self.max_speed, self.bias))):
            raise ValueError('Odometry parameters must be finite')
        if self.gain == 0 or self.timeout <= 0 or self.max_speed <= 0:
            raise ValueError('Invalid RPM conversion or feedback timeout')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        gyro_topic = self.declare_parameter('gyro_topic', '/camera/camera/gyro/sample').value
        accel_topic = self.declare_parameter('accel_topic', '/camera/camera/accel/sample').value
        wheel_topic = self.declare_parameter('wheel_topic', '/sensors/core').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.motion = MeasuredOdometry(max_gap=self.timeout)
        self.wheel = self.gyro = self.accel = None
        self.last_reason = None
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(VescStateStamped, wheel_topic, self.on_wheel, qos_profile_sensor_data)
        self.create_subscription(Imu, gyro_topic, self.on_gyro, qos_profile_sensor_data)
        self.create_subscription(Imu, accel_topic, self.on_accel, qos_profile_sensor_data)
        self.create_timer(0.02, self.tick)
        self.get_logger().info('Odometry uses measured VESC RPM and RealSense gyro; no command fallback')

    def valid_stamp(self, header, previous):
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp <= 0 or not -0.05 <= age <= self.timeout:
            return None
        if previous is not None and stamp <= previous[0]:
            return None
        return stamp

    def on_wheel(self, message):
        stamp = self.valid_stamp(message.header, self.wheel)
        speed = (message.state.speed - self.offset) / self.gain
        if message.state.fault_code != 0 or not math.isfinite(speed) or abs(speed) > self.max_speed:
            self.wheel = None
            return
        if stamp is not None:
            self.wheel = (stamp, time.monotonic(), speed)

    def on_gyro(self, message):
        stamp = self.valid_stamp(message.header, self.gyro)
        if stamp is None:
            return
        try:
            if not message.header.frame_id:
                raise ValueError('IMU has no sensor frame')
            transform = self.buffer.lookup_transform(self.base_frame, message.header.frame_id, Time())
            q, v = transform.transform.rotation, message.angular_velocity
            rate = rotate_vector((q.x, q.y, q.z, q.w), (v.x, v.y, v.z))[2]
            if abs(rate) > 20.0 or message.angular_velocity_covariance[0] < 0:
                raise ValueError('Invalid gyro measurement')
        except (TransformException, ValueError):
            self.gyro = None
            return
        self.gyro = (stamp, time.monotonic(), rate)

    def on_accel(self, message):
        stamp = self.valid_stamp(message.header, self.accel)
        if stamp is None:
            return
        try:
            transform = self.buffer.lookup_transform(self.base_frame, message.header.frame_id, Time())
            q, v = transform.transform.rotation, message.linear_acceleration
            acceleration = rotate_vector((q.x, q.y, q.z, q.w), (v.x, v.y, v.z))
            if message.linear_acceleration_covariance[0] < 0:
                raise ValueError('Invalid acceleration measurement')
        except (TransformException, ValueError):
            self.accel = None
            return
        self.accel = (stamp, time.monotonic(), *acceleration)

    def calibrate(self):
        """Estimate a fixed session bias only after a two-second stationary window.

        The accelerometer is used to reject movement during initialization, never
        integrated into speed. No adaptation runs during subsequent driving.
        """
        if (not self.fresh(self.accel) or abs(self.wheel[2]) > .015 or
                abs(self.gyro[2]) > .05 or not 8.0 < math.sqrt(sum(v*v for v in self.accel[2:])) < 11.0):
            self.bias_samples.clear()
            return False
        stamp = self.gyro[0]
        if not self.bias_samples or stamp > self.bias_samples[-1][0]:
            self.bias_samples.append((stamp, self.gyro[2], *self.accel[2:]))
        while self.bias_samples and stamp - self.bias_samples[0][0] > 2.2:
            self.bias_samples.popleft()
        if len(self.bias_samples) < 50 or stamp - self.bias_samples[0][0] < 2.0:
            return False
        columns = list(zip(*self.bias_samples))
        if pstdev(columns[1]) > .01 or any(pstdev(column) > .15 for column in columns[2:]):
            self.bias_samples.clear()
            return False
        self.bias = fmean(columns[1])
        self.bias_ready = True
        self.bias_samples.clear()
        self.get_logger().info(f'Stationary RealSense body yaw bias: {self.bias:.6f} rad/s')
        return True

    def fresh(self, value):
        if value is None:
            return False
        age = self.get_clock().now().nanoseconds * 1e-9 - value[0]
        return -0.05 <= age <= self.timeout and time.monotonic() - value[1] <= self.timeout

    def tick(self):
        reason = None
        if not self.fresh(self.wheel):
            reason = 'Waiting for fresh measured VESC RPM; odometry paused'
        elif not self.fresh(self.gyro):
            reason = 'Waiting for fresh RealSense gyro and mounting TF; odometry paused'
        elif not self.bias_ready and not self.calibrate():
            reason = 'Hold vehicle still for RealSense gyro bias initialization (2 seconds)'
        if reason is not None:
            self.motion.pause()
            if not self.fresh(self.wheel) or not self.fresh(self.gyro):
                self.bias_samples.clear()
            if reason != self.last_reason:
                self.get_logger().warning(reason)
            self.last_reason = reason
            return
        if self.last_reason is not None:
            self.get_logger().info('Measured odometry ready: RealSense gyro + VESC RPM')
            self.last_reason = None
        stamp, _, rate = self.gyro
        rate -= self.bias
        speed = self.wheel[2]
        if not self.motion.update(stamp, speed, rate):
            return
        message = Odometry()
        message.header.stamp = Time(seconds=stamp).to_msg()
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.motion.x
        message.pose.pose.position.y = self.motion.y
        message.pose.pose.orientation.z = math.sin(self.motion.yaw / 2)
        message.pose.pose.orientation.w = math.cos(self.motion.yaw / 2)
        message.twist.twist.linear.x = speed
        message.twist.twist.angular.z = rate
        for i in (2, 3, 4):
            message.pose.covariance[i * 6 + i] = 1e6
            message.twist.covariance[i * 6 + i] = 1e6
        for i, row in enumerate((0, 1, 5)):
            for j, column in enumerate((0, 1, 5)):
                message.pose.covariance[row * 6 + column] = self.motion.covariance[i][j]
        message.twist.covariance[0] = self.motion.speed_variance
        message.twist.covariance[7] = 0.01  # Planar nonholonomic assumption; not a measured lateral velocity.
        message.twist.covariance[35] = self.motion.yaw_rate_variance
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can interrupt take_message during context shutdown. Keep real
        # runtime failures visible, but do not report normal SIGINT as a crash.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
