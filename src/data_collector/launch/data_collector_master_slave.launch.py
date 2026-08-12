from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    config_path = PathJoinSubstitution([
        FindPackageShare('data_collector'),
        'config',
        'collect_config_master_slave.yaml'
    ])

    return LaunchDescription([
        Node(
            package='data_collector',
            executable='data_collector_node',
            name='data_collector_node',
            output='screen',
            parameters=[{
                'config_path': config_path
            }]
        ),

    ])
