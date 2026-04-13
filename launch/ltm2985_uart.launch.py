from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    default_params_file = os.path.join(
        get_package_share_directory('ltm2985_uart'),
        'config',
        'ltm2985_uart.params.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the YAML parameters file',
        ),
        Node(
            package='ltm2985_uart',
            executable='ltm2985_uart_node',
            name='ltm2985_uart_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])