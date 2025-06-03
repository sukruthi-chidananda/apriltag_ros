from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='cam_top',
            description='Namespace of the camera'
        ),

        ComposableNodeContainer(
            name='apriltag_container',
            namespace='apriltag_' + camera_name.perform({}),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='apriltag_ros',
                    plugin='AprilTagNode',
                    name='apriltag',
                    remappings=[
                        ('image_rect', [camera_name, '/color/image_raw']),
                        ('camera_info', [camera_name, '/color/camera_info'])
                    ],
                    parameters=[{
                        'tag_family': '36h11',
                        'size': 0.16  # meters, update based on your tag
                    }]
                )
            ],
            output='screen'
        )
    ])
