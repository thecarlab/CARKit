#!/usr/bin/env python3

# Copyright 2026 University of Delaware
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# CARKit learning annotation: assembles ROS nodes, parameters, and remappings for startup.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Build and return the ROS 2 launch description for this package."""
    config = PathJoinSubstitution(
        [FindPackageShare("carkit_behavior"), "config", "behavior_center.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="carkit_behavior",
                executable="behavior_center_node",
                name="behavior_center_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
