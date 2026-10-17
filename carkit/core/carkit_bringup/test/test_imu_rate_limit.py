import os
import time
import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != '199' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='Requires isolated ROS domain 199 localhost',
)


def test_50hz_preserves_samples_and_stops_on_lost_input(runtime_nodes):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from carkit_bringup.imu_rate_limit import ImuRateLimit
    rclpy.init()
    peer = rclpy.create_node('imu_rate_test')
    executor = SingleThreadedExecutor()
    runtime = runtime_nodes
    runtime.create("imu_rate_limit", ImuRateLimit, executor)
    executor.add_node(peer)
    received = {key: [] for key in ('gyro', 'accel')}
    publishers, subscribers = {}, []
    for stream in received:
        publishers[stream] = peer.create_publisher(Imu, f'/camera/camera/{stream}/sample_raw', 10)
        subscribers.append(peer.create_subscription(Imu, f'/camera/camera/{stream}/sample',
            lambda m, key=stream: received[key].append((time.monotonic(), m)), qos_profile_sensor_data))
    def pump(duration, publish=True):
        until = time.monotonic() + duration
        next_send = 0
        while time.monotonic() < until:
            if publish and time.monotonic() >= next_send:
                next_send = time.monotonic() + .005
                for stream, publisher in publishers.items():
                    message = Imu()
                    message.header.stamp = peer.get_clock().now().to_msg()
                    message.header.frame_id = stream + '_optical_frame'
                    message.angular_velocity.z = .123
                    publisher.publish(message)
            executor.spin_once(timeout_sec=.001)
    try:
        pump(.5)
        for values in received.values():
            values.clear()
        pump(2)
        for stream, values in received.items():
            rate = (len(values)-1)/(values[-1][0]-values[0][0])
            assert 45 < rate < 55
            assert all(m.header.frame_id == stream+'_optical_frame' and m.angular_velocity.z == .123 for _,m in values)
            stamps = [m.header.stamp.sec*1000000000+m.header.stamp.nanosec for _,m in values]
            assert all(b>a for a,b in zip(stamps, stamps[1:]))
        pump(.1, False)
        counts = {key: len(values) for key,values in received.items()}
        pump(.3, False)
        assert counts == {key: len(values) for key,values in received.items()}
    finally:
        executor.shutdown()
        runtime.close()
        peer.destroy_node()
        rclpy.shutdown()
