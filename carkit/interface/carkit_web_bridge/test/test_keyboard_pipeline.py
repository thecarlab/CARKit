"""Hardware-free integration: browser WebSocket -> bridge -> real ROS arbiter.

Run with CARKIT_BRIDGE_EXE and CARKIT_ARBITER_EXE pointing to built binaries,
ROS_LOCALHOST_ONLY=1, and an unused ROS_DOMAIN_ID. No vehicle driver is started.
"""
import json
import os
import signal
import secrets
import socket
import subprocess
import time

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import websocket


@pytest.mark.skipif(not os.environ.get('CARKIT_BRIDGE_EXE') or not os.environ.get('CARKIT_ARBITER_EXE'),
                    reason='requires built bridge and arbiter executables')
def test_keyboard_commands_modes_disconnect_and_browser_ownership():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        pytest.fail('Run this hardware-free test in an explicit localhost-only ROS domain')
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    rclpy.init()
    node = rclpy.create_node('keyboard_pipeline_test')
    commands, modes = [], []
    from geometry_msgs.msg import PoseWithCovarianceStamped
    initial_poses = []
    node.create_subscription(PoseWithCovarianceStamped, "/initialpose", initial_poses.append, 10)
    node.create_subscription(AckermannDriveStamped, '/ackermann_cmd', commands.append, 10)
    node.create_subscription(String, '/control_center/main_state', lambda m: modes.append(m.data), 10)
    nav = node.create_publisher(AckermannDriveStamped, '/drive', 10)
    processes, clients = [], []
    token = secrets.token_urlsafe(32)

    def spin(duration=.12, navigation=False):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if navigation:
                cmd = AckermannDriveStamped(); cmd.drive.speed = .2
                nav.publish(cmd)
            rclpy.spin_once(node, timeout_sec=.01)

    def send(client, topic, message):
        client.send(json.dumps({'op':'publish', 'topic':topic, 'msg':message}))

    def drive(client, speed=.4, steering=.25):
        send(client, '/teleop', {'drive':{'speed':speed, 'steering_angle':steering}})

    try:
        processes.append(subprocess.Popen([os.environ['CARKIT_BRIDGE_EXE'], '--ros-args',
            '-p', 'address:=127.0.0.1', '-p', f'port:={port}', '-p', 'max_speed:=0.5',
            '-p', 'max_reverse_speed:=0.3', '-p', 'max_steering_angle:=0.27'],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            env={**os.environ, "CARKIT_CONTROL_TOKEN": token}))
        processes.append(subprocess.Popen([os.environ['CARKIT_ARBITER_EXE']],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE))
        deadline = time.monotonic() + 5
        while True:
            try:
                clients.append(websocket.create_connection(f'ws://127.0.0.1:{port}', timeout=1, suppress_origin=True))
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(.05)
        with pytest.raises(websocket.WebSocketException):
            websocket.create_connection(f'ws://127.0.0.1:{port}', timeout=1,
                                        origin='https://untrusted.example')
        primary = clients[0]
        observer = websocket.create_connection(f'ws://127.0.0.1:{port}', timeout=1, suppress_origin=True)
        clients.append(observer)
        spin(.6)
        assert modes[-1] == 'HUMAN_CONTROL'
        drive(primary); spin()
        assert commands[-1].drive.speed == 0  # Unauthenticated commands rejected.
        primary.send(json.dumps({'op':'authenticate', 'token':token}))
        observer.send(json.dumps({'op':'authenticate', 'token':token}))
        primary.send(json.dumps({'op':'claim_control'}))
        spin()
        send(observer, '/initialpose', {'header':{'frame_id':'map'}, 'pose':{'pose':{'position':{'x':1.0,'y':2.0,'z':0.0}, 'orientation':{'x':0.0,'y':0.0,'z':0.0,'w':1.0}},'covariance':[0.0]*36}})
        spin()
        assert initial_poses and initial_poses[-1].pose.pose.position.x == 1.0
        send(observer, '/enable_autonomous_control', {'data':1})
        spin()
        assert modes[-1] == 'HUMAN_CONTROL'
        send(observer, '/control_center/emergency_stop', {'data':True})
        spin()
        assert modes[-1] == 'EMERGENCY_STOP'
        send(observer, '/control_center/emergency_stop', {'data':False})
        spin()
        assert modes[-1] == 'EMERGENCY_STOP'  # Observers may stop, never reset.
        send(primary, '/control_center/emergency_stop', {'data':False})
        spin()
        assert modes[-1] == 'HUMAN_CONTROL'
        primary.send(json.dumps({'op':'control_heartbeat'}))
        drive(primary); spin()
        assert commands[-1].drive.speed == pytest.approx(.4)
        assert commands[-1].drive.steering_angle == pytest.approx(.25)
        drive(observer, -.3); spin(.05)
        assert commands[-1].drive.speed == pytest.approx(.4)
        drive(primary, 0, 0); spin()
        assert commands[-1].drive.speed == 0
        drive(primary, 9, 9); spin()
        assert commands[-1].drive.speed == pytest.approx(.5)
        assert commands[-1].drive.steering_angle == pytest.approx(.27)
        spin(.4)
        assert commands[-1].drive.speed == 0  # No repeated server-side stale commands.
        drive(primary, -.8); spin()
        assert commands[-1].drive.speed == pytest.approx(-.3)
        primary.close(); clients.remove(primary); spin()
        assert commands[-1].drive.speed == 0
        # Another viewer can take over after the owner disconnects.
        observer.send(json.dumps({'op':'claim_control'}))
        drive(observer); spin()
        assert commands[-1].drive.speed == pytest.approx(.4)
        drive(observer, 0, 0)
        observer.send(json.dumps({'op':'control_heartbeat'}))
        send(observer, '/enable_autonomous_control', {'data':1})
        spin(.6, navigation=True)
        assert modes[-1] == 'AUTO_DRIVE'
        assert commands[-1].drive.speed == pytest.approx(.2)
        send(observer, '/control_center/emergency_stop', {'data':True})
        spin(.12, navigation=True)
        assert commands[-1].drive.speed == 0
        send(observer, '/enable_autonomous_control', {'data':1})
        spin(.1, navigation=True)
        assert commands[-1].drive.speed == 0  # Caps Lock cannot clear E-stop.
        send(observer, '/control_center/emergency_stop', {'data':False})
        spin(.6, navigation=True)
        assert modes[-1] == 'HUMAN_CONTROL'
        assert commands[-1].drive.speed == 0
        spin(3.2)
        drive(observer); spin()
        assert commands[-1].drive.speed == 0  # An expired operator must claim again.
        observer.send(json.dumps({'op':'claim_control'}))
        drive(observer); spin()
        assert commands[-1].drive.speed == pytest.approx(.4)
    finally:
        for client in clients:
            client.close()
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        node.destroy_node()
        rclpy.shutdown()
