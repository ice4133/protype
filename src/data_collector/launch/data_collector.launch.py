from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # Construct path to config file (relative to package root)
    config_path = PathJoinSubstitution([
        FindPackageShare('data_collector'),  # Replace with your package name
        'config',
        'collect_config.yaml'                # Replace with your config filename
    ])

    return LaunchDescription([

        # Data collector node
        Node(
            package='data_collector',
            executable='data_collector_node',
            name='data_collector_node',
            output='screen',
            parameters=[{
                'config_path': config_path  # Pass path as parameter
            }]
        ),

        # Second node configuration
        Node(
            package='data_collector',
            executable='pedal_pub',
            name='pedal_pub',  # Unique node name
            output='screen',
        )
    ])