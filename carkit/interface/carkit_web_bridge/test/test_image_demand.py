"""Native bridge demand and throttling, with synthetic ROS images only."""
import json
import os
import secrets
import socket
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(not os.environ.get('CARKIT_BRIDGE_EXE'), reason='Requires native bridge binary')


def test_camera_subscription_lifetime_throttle_and_retained_map():
    import cbor2
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from nav_msgs.msg import OccupancyGrid
    import websocket
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') == '199'
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    token = secrets.token_urlsafe(32)
    process = subprocess.Popen([os.environ['CARKIT_BRIDGE_EXE'], '--ros-args',
                                '-p', 'address:=127.0.0.1', '-p', f'port:={port}'],
                               env={**os.environ, 'CARKIT_CONTROL_TOKEN': token}, stdout=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('image_demand_test')
    topic = '/camera/camera/color/image_raw/compressed'
    publisher = node.create_publisher(CompressedImage, topic, 10)
    map_pub = node.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clients = []

    def pump(seconds, images=False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if images:
                m = CompressedImage()
                m.header.stamp = node.get_clock().now().to_msg()
                m.format = 'jpeg'
                m.data = bytes([1,2,3,4])
                publisher.publish(m)
            rclpy.spin_once(node, timeout_sec=.02)

    def connect():
        deadline = time.monotonic() + 6
        while True:
            try:
                client = websocket.create_connection(f'ws://127.0.0.1:{port}', timeout=.15, suppress_origin=True)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(.05)
        clients.append(client)
        client.send(json.dumps({'op':'authenticate', 'token':token}))
        return client

    def receive(client):
        messages=[]
        while True:
            try:
                message=client.recv()
                messages.append(cbor2.loads(message) if isinstance(message, bytes) else json.loads(message))
            except websocket.WebSocketTimeoutException:
                return messages

    def subscribe(client, key, throttle=0):
        client.send(json.dumps({'op':'subscribe','topic':key,'throttle_rate':throttle}))

    try:
        first=connect()
        pump(.5)
        assert publisher.get_subscription_count()==0  # A connected dashboard alone needs no JPEGs.
        grid=OccupancyGrid();grid.info.width=1;grid.info.height=1;grid.info.resolution=.1;grid.data=[42]
        map_pub.publish(grid);pump(.2)
        subscribe(first,'/map');pump(.2)
        assert any(m.get('topic')=='/map' and m['msg']['data']==[42] for m in receive(first))
        subscribe(first,topic,200)
        pump(.8)
        assert publisher.get_subscription_count()==1
        receive(first)
        pump(1.2,True)
        frames=[m for m in receive(first) if m.get('topic')==topic]
        assert 4<=len(frames)<=7
        assert all(m['msg']['data']==bytes([1,2,3,4]) for m in frames)
        second=connect();subscribe(second,topic);pump(.3)
        first.send(json.dumps({'op':'unsubscribe','topic':topic}));pump(.4)
        assert publisher.get_subscription_count()==1
        second.close();clients.remove(second);pump(.6)
        assert publisher.get_subscription_count()==0
        subscribe(first,topic);pump(.5)
        assert publisher.get_subscription_count()==1
        first.close();clients.remove(first);pump(.6)
        assert publisher.get_subscription_count()==0
    finally:
        for client in clients:client.close()
        node.destroy_node();rclpy.shutdown()
        process.terminate();process.wait(timeout=10)
