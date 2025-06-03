from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
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
            namespace=LaunchConfiguration('camera_name'),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='apriltag_ros',
                    plugin='AprilTagNode',
                    name='apriltag',
                    remappings=[
                        # Input topic remappings
                        ('image_rect', [camera_name, TextSubstitution(text='/color/image_raw')]),
                        ('camera_info', [camera_name, TextSubstitution(text='/color/camera_info')]),
                        # Output topic remappings
                        ('detections', [camera_name, TextSubstitution(text='/apriltag/detections')]),
                        ('tf', [camera_name, TextSubstitution(text='/apriltag/tf')]),
                    ],
                    parameters=[{
                        'tag_family': '36h11',
                        'size': 0.174,  # Fixed: 17.4cm converted to meters
                        'max_hamming': 0,  # Add this for better detection
                        'z_up': True,      # Add this if your coordinate frame needs it
                        'publish_tf': True # Ensure TF is published
                    }]
                )
            ],
            output='screen'
        )
    ])