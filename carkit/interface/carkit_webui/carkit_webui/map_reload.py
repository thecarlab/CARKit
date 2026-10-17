"""Reload an active Nav2 map without restarting the vehicle launch process."""
import sys
import time

import rclpy
from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from rclpy.node import Node
from std_msgs.msg import Int8
from ackermann_msgs.msg import AckermannDriveStamped


def reload_map(path):
    rclpy.init()
    node = Node('carkit_map_reload')
    clients = []

    def call(kind, name, request, timeout=8.0):
        client = node.create_client(kind, name)
        clients.append(client)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{name} is unavailable; the stack was not restarted')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.exception():
            raise RuntimeError(f'{name} did not complete; the stack was not restarted')
        return future.result()

    try:
        # Check the map service before affecting an existing navigation session.
        probe = node.create_client(LoadMap, '/map_server/load_map')
        if not probe.wait_for_service(timeout_sec=10):
            raise RuntimeError('Map server is unavailable; enable planning before changing maps live')
        mode = node.create_publisher(Int8, '/enable_autonomous_control', 10)
        stop = node.create_publisher(AckermannDriveStamped, '/teleop', 10)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            mode.publish(Int8(data=0))
            stop.publish(AckermannDriveStamped())
            rclpy.spin_once(node, timeout_sec=.05)
        for action in ('navigate_to_pose', 'navigate_through_poses'):
            call(CancelGoal, f'/{action}/_action/cancel_goal', CancelGoal.Request())
        request = LoadMap.Request()
        request.map_url = path
        response = call(LoadMap, '/map_server/load_map', request, timeout=15)
        if response.result != 0:
            raise RuntimeError(f'Map server rejected the map (result {response.result}); previous map retained')
        # A successful load publishes /map for AMCL and both static layers.
        for name in ('global_costmap', 'local_costmap'):
            try:
                call(ClearEntireCostmap, f'/{name}/clear_entirely_{name}', ClearEntireCostmap.Request(), timeout=3)
            except RuntimeError as error:
                print(f'Warning: {error}', flush=True)
        print('Map reloaded; set the initial pose before resuming autonomous navigation.', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    reload_map(sys.argv[1])
