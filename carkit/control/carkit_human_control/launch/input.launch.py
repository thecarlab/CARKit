"""Hardware-independent joystick input, with an optional manual-only relay."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('carkit_human_control'), 'config', 'joy_teleop.yaml'])
    defaults = {'device_id': '0', 'speed_axis': '1', 'steering_axis': '2',
                'mode_toggle_button': '10', 'speed_scale': '0.8', 'steering_scale': '0.34',
                'max_reverse_speed': '0.8', 'direct_relay': 'false',
                'vehicle_command_topic': '/ackermann_cmd'}
    args = [DeclareLaunchArgument('joy_config', default_value=config)]
    args += [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    def value(name, kind):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)
    return LaunchDescription(args + [
        Node(package='joy', executable='joy_node', name='joy', parameters=[
            LaunchConfiguration('joy_config'), {'device_id': value('device_id', int)}]),
        Node(package='carkit_human_control', executable='joystick_teleop', name='joy_teleop',
             parameters=[LaunchConfiguration('joy_config'), {
                 **{k: value(k, int) for k in ('speed_axis', 'steering_axis', 'mode_toggle_button')},
                 **{k: value(k, float) for k in ('speed_scale', 'steering_scale')},
                 'standalone_manual': value('direct_relay', bool),
             }]),
        Node(package='carkit_human_control', executable='command_relay',
             condition=IfCondition(LaunchConfiguration('direct_relay')), parameters=[{
                 'output_topic': LaunchConfiguration('vehicle_command_topic'),
                 'max_speed': value('speed_scale', float),
                 'max_reverse_speed': value('max_reverse_speed', float),
                 'max_steering_angle': value('steering_scale', float),
             }]),
    ])
