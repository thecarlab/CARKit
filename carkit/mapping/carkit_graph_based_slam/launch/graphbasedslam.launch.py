import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    graphbasedslam_param_dir = launch.substitutions.LaunchConfiguration(
        'graphbasedslam_param_dir',
        default=os.path.join(
            get_package_share_directory('carkit_graph_based_slam'),
            'param',
            'graphbasedslam.yaml'))

    graphbasedslam = launch_ros.actions.Node(
        package='carkit_graph_based_slam',
        executable='carkit_graph_based_slam_node',
        parameters=[graphbasedslam_param_dir],
        output='screen'
        )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'graphbasedslam_param_dir',
            default_value=graphbasedslam_param_dir,
            description='Full path to graphbasedslam parameter file to load'),
        graphbasedslam,
            ])