#!/usr/bin/env python3
"""Observe ADA2 hardware topics for 20 seconds; never publish commands.

Run inside the container after sourcing install/setup.bash and starting the
chassis/sensors in the WebUI. Exit nonzero if any expected stream is missing.
"""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu, LaserScan
from vesc_msgs.msg import VescStateStamped


def main():
    rclpy.init()
    node = rclpy.create_node('carkit_stationary_hardware_check')
    results = {}
    subscriptions = []

    def subscribe(topic, message_type, describe):
        results[topic] = {'messages': 0}

        def received(message):
            result = results[topic]
            result['messages'] += 1
            result.update(describe(message))

        subscriptions.append(node.create_subscription(
            message_type, topic, received, qos_profile_sensor_data))

    subscribe('/sensors/core', VescStateStamped, lambda m: {
        'voltage': m.state.voltage_input, 'electrical_rpm': m.state.speed,
        'fault_code': m.state.fault_code,
    })
    subscribe('/scan', LaserScan, lambda m: {
        'frame': m.header.frame_id, 'ranges': len(m.ranges),
        'valid_returns': sum(math.isfinite(r) and m.range_min <= r <= m.range_max
                             for r in m.ranges),
    })
    subscribe('/odom', Odometry, lambda m: {
        'frame': m.header.frame_id, 'child_frame': m.child_frame_id,
        'speed_m_s': m.twist.twist.linear.x,
        'yaw_rate_rad_s': m.twist.twist.angular.z,
    })
    for kind in ('gyro', 'accel'):
        subscribe(f'/camera/camera/{kind}/sample', Imu,
                  lambda m: {'frame': m.header.frame_id})
    invalid_images = 0

    def describe_image(message):
        nonlocal invalid_images
        pixels = None
        if message.data:
            try:
                pixels = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8),
                                     cv2.IMREAD_COLOR)
            except cv2.error:
                pass
        if pixels is None:
            invalid_images += 1
        return {'format': message.format, 'bytes': len(message.data),
                'decoded_shape': list(pixels.shape) if pixels is not None else None,
                'invalid_images': invalid_images}

    subscribe('/camera/camera/color/image_raw/compressed', CompressedImage,
              describe_image)
    deadline = time.monotonic() + 20.0
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(results, indent=2))
    required = tuple(results)
    if not results['/odom']['messages']:
        print('Measured odometry missing: hold still for IMU initialization and check RPM, gyro, accel and mounting TF.')
    return 0 if all(results[topic]['messages'] for topic in required) and not invalid_images else 1


if __name__ == '__main__':
    raise SystemExit(main())
