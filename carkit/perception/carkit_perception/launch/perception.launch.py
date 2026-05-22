import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('carkit_perception')

    model_path = LaunchConfiguration('model_path')
    image_topic = LaunchConfiguration('image_topic')
    inference_image_topic = LaunchConfiguration('inference_image_topic')
    detection_topic = LaunchConfiguration('detection_topic')
    rviz_config = LaunchConfiguration('rviz_config')
    start_rviz = LaunchConfiguration('start_rviz')

    perception_node = Node(
        package='carkit_perception',
        executable='perception_node',
        name='yolo_node',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'image_topic': image_topic,
            'inference_image_topic': inference_image_topic,
            'detection_topic': detection_topic,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='perception_rviz',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=os.path.join(package_share, 'models', 'yolo11n.pt'),
            description='YOLO model path. Use yolo11n.engine only on systems with TensorRT installed.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/camera/color/image_raw',
            description='Input camera image topic.',
        ),
        DeclareLaunchArgument(
            'inference_image_topic',
            default_value='/yolo/inference_image',
            description='Annotated inference image output topic.',
        ),
        DeclareLaunchArgument(
            'detection_topic',
            default_value='/yolo/detections',
            description='Text detection output topic.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_share, 'rviz', 'perception.rviz'),
            description='RViz config for perception debugging.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz image viewer.',
        ),
        perception_node,
        rviz_node,
    ])
