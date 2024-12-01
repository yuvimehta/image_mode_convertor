from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # launch arguments
        DeclareLaunchArgument(
            'input_image_topic',
            default_value='/image_raw',  # Default topic
            description='The input image topic'
        ),
        DeclareLaunchArgument(
            'output_image_topic',
            default_value='/processed_image',
            description='The output image topic'
        ),

        #  To run USB Camera Node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node'
        ),
        
        # Node to run the conversion code
        Node(
            package='image_mode_service',
            executable='image_mode_service',
            name='image_mode_service_node',
            parameters=
            [
                {   
                    'input_image_topic': LaunchConfiguration('input_image_topic'),
                    'output_topic': LaunchConfiguration('output_image_topic')
                }
            ]
        )

    ])
