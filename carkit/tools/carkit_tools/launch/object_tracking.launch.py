from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    target_object_type_arg = DeclareLaunchArgument(
        'target_object_type',
        default_value='book',
        description='Type of object to track'
    )
    
    horizontal_fov_arg = DeclareLaunchArgument(
        'horizontal_fov',
        default_value='69.4',
        description='Horizontal field of view in degrees'
    )
    
    waypoint_distance_arg = DeclareLaunchArgument(
        'waypoint_distance',
        default_value='0.5',
        description='Distance between waypoints in meters'
    )
    
    # Create node actions
    object_position_node = Node(
        package='carkit_tools',
        executable='object_position',
        name='object_position_node',
        parameters=[{
            'target_object_type': LaunchConfiguration('target_object_type'),
            'horizontal_fov': LaunchConfiguration('horizontal_fov')
        }],
        output='screen'
    )
    
    path_tracker_node = Node(
        package='carkit_tools',
        executable='path_tracker',
        name='path_tracker_node',
        parameters=[{
            'waypoint_distance': LaunchConfiguration('waypoint_distance')
        }],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(target_object_type_arg)
    ld.add_action(horizontal_fov_arg)
    ld.add_action(waypoint_distance_arg)
    
    # Add nodes
    ld.add_action(object_position_node)
    ld.add_action(path_tracker_node)
    
    return ld 
