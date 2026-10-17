"""Live ROS regression: duplicate camera frames cannot refresh behavior safety."""
import os
import signal
import subprocess
import time

import pytest
import rclpy
from std_msgs.msg import String, Bool
from carkit_perception_msgs.msg import YoloDetection2DArray


@pytest.mark.skipif(not os.environ.get('CARKIT_BEHAVIOR_EXE'), reason='requires built behavior node')
def test_repeated_or_old_source_frames_do_not_keep_perception_fresh():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID')
    rclpy.init()
    node = rclpy.create_node('perception_freshness_probe')
    mode = node.create_publisher(String, '/control_center/main_state', 10)
    detections = node.create_publisher(YoloDetection2DArray, '/yolo/detections_2d', 10)
    states = []
    node.create_subscription(Bool, '/behavior/override_active', lambda m: states.append(m.data), 10)
    process = subprocess.Popen([os.environ['CARKIT_BEHAVIOR_EXE']], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    def spin(duration, frame=None):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            mode.publish(String(data='AUTO_DRIVE'))
            if frame is not None:
                detections.publish(frame)
            rclpy.spin_once(node, timeout_sec=.01)
    try:
        spin(.8)
        assert states and states[-1]  # No perception yet: hold zero override.
        frame = YoloDetection2DArray()
        frame.header.stamp = node.get_clock().now().to_msg()
        spin(.15, frame)
        assert states[-1] is False  # New empty scene permits normal navigation.
        spin(.7, frame)
        assert states[-1] is True  # Repeats must not renew freshness.
        frame.header.stamp = node.get_clock().now().to_msg()
        spin(.15, frame)
        assert states[-1] is False
        frame.header.stamp.sec -= 5
        spin(.7, frame)
        assert states[-1] is True
    finally:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
        node.destroy_node(); rclpy.shutdown()
