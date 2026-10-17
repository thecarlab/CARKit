"""Real ROS nodes with simulated sensor feedback; never run in the vehicle domain."""

import math
import os
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != '199' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='Requires isolated ROS domain 199 and ROS_LOCALHOST_ONLY=1',
)


def test_measured_motion_tf_loss_and_command_independence(runtime_nodes):
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Imu
    from tf2_ros import StaticTransformBroadcaster
    from vesc_msgs.msg import VescStateStamped
    from carkit_bringup.realsense_odometry import RealSenseOdometry

    rclpy.init()
    from carkit_bringup.imu_rate_limit import ImuRateLimit
    peer = rclpy.create_node('measured_odometry_sensor_test')
    executor = SingleThreadedExecutor()
    runtime = runtime_nodes
    runtime.create("imu_rate_limit", ImuRateLimit, executor)
    runtime.create("realsense_wheel_odometry", RealSenseOdometry, executor)
    executor.add_node(peer)
    wheel = peer.create_publisher(VescStateStamped, '/sensors/core', 10)
    gyro = peer.create_publisher(Imu, '/camera/camera/gyro/sample_raw', 10)
    accel = peer.create_publisher(Imu, '/camera/camera/accel/sample_raw', 10)
    command = peer.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
    received = []
    subscription = peer.create_subscription(Odometry, '/odom', received.append, 10)
    tf = StaticTransformBroadcaster(peer)
    speed, rate, gyro_enabled = 0.0, 0.0, True
    frozen_stamp = None

    def pump(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            stamp = peer.get_clock().now().to_msg()
            w = VescStateStamped()
            w.header.stamp = stamp
            w.state.speed = speed * 4614.0
            wheel.publish(w)
            if gyro_enabled:
                g = Imu()
                g.header.stamp = frozen_stamp or stamp
                g.header.frame_id = 'test_camera_gyro_optical_frame'
                g.angular_velocity.y = rate + .006  # Fixed sensor bias must not turn the car.
                gyro.publish(g)
            acceleration = Imu()
            acceleration.header.stamp = stamp
            acceleration.header.frame_id = 'test_camera_gyro_optical_frame'
            acceleration.linear_acceleration.y = 9.81
            accel.publish(acceleration)
            c = AckermannDriveStamped()
            c.drive.speed = 3.0
            c.drive.steering_angle = -.245
            command.publish(c)
            # Drain callbacks from the rate adapter and odometry as separate nodes.
            for _ in range(16):
                executor.spin_once(timeout_sec=.0001)
            time.sleep(.005)

    def heading(message):
        q = message.pose.pose.orientation
        return 2 * math.atan2(q.z, q.w)

    try:
        # Sensors without their mounting TF must not be treated as body-frame data.
        pump(.7)
        assert not received
        mounting = TransformStamped()
        mounting.header.frame_id = 'base_link'
        mounting.child_frame_id = 'test_camera_gyro_optical_frame'
        mounting.header.stamp = peer.get_clock().now().to_msg()
        mounting.transform.rotation.x = math.sin(math.pi / 4)
        mounting.transform.rotation.w = math.cos(math.pi / 4)
        tf.sendTransform(mounting)
        speed = .4
        pump(.4)
        assert not received  # Bias cannot be calibrated while wheels are moving.
        speed = 0.0
        pump(2.6)
        assert len(received) > 5
        assert abs(received[-1].pose.pose.position.x) < 1e-8
        assert abs(heading(received[-1])) < 1e-8

        speed = .4
        before = received[-1].pose.pose.position.x
        pump(.8)
        assert .25 < received[-1].pose.pose.position.x - before < .4
        assert received[-1].twist.twist.linear.x == pytest.approx(.4)
        assert received[-1].twist.twist.angular.z == pytest.approx(0.)
        assert abs(heading(received[-1])) < 1e-8  # Opposite steering command has no effect.

        speed, rate = 0.0, .2
        pump(.2)
        position = received[-1].pose.pose.position
        initial_yaw = heading(received[-1])
        pump(.7)
        assert .10 < heading(received[-1]) - initial_yaw < .18
        assert received[-1].twist.twist.angular.z == pytest.approx(.2)
        assert received[-1].pose.pose.position.x == pytest.approx(position.x, abs=1e-6)
        assert received[-1].pose.pose.position.y == pytest.approx(position.y, abs=1e-6)

        gyro_enabled = False
        pump(.5)
        count = len(received)
        yaw_at_loss = heading(received[-1])
        pump(.4)
        assert len(received) == count  # Fresh wheel feedback/commands cannot replace the gyro.
        gyro_enabled = True
        frozen_stamp = received[-1].header.stamp
        pump(.4)
        assert len(received) == count  # Replayed sensor packets are not fresh measurements.
        frozen_stamp, rate = None, 0.0
        pump(.4)
        assert len(received) > count
        assert heading(received[-1]) == pytest.approx(yaw_at_loss, abs=.01)

        speed = -.3
        before = received[-1].pose.pose.position.x
        pump(.7)
        assert received[-1].pose.pose.position.x < before - .15
        assert received[-1].twist.twist.linear.x == pytest.approx(-.3)

        speed = float('nan')
        pump(.3)
        count = len(received)
        pump(.3)
        assert len(received) == count
        topics = {name for name, _ in peer.get_subscriber_names_and_types_by_node(
            'realsense_wheel_odometry', '/')}
        assert not topics.intersection({'/ackermann_cmd', '/drive', '/teleop', '/commands/servo/position'})
    finally:
        peer.destroy_subscription(subscription)
        executor.shutdown()
        peer.destroy_node()
        runtime.close()
        rclpy.shutdown()
