import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('yam_damiao_controller'),
        'config',
        'master_slave.yaml'
    )

    use_master_l_arg = DeclareLaunchArgument(
        'use_master_l', default_value='true',
        description='Launch left master arm')
    use_master_r_arg = DeclareLaunchArgument(
        'use_master_r', default_value='true',
        description='Launch right master arm')
    use_slave_l_arg = DeclareLaunchArgument(
        'use_slave_l', default_value='true',
        description='Launch left slave arm')
    use_slave_r_arg = DeclareLaunchArgument(
        'use_slave_r', default_value='true',
        description='Launch right slave arm')

    master_l_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='master_l',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('use_master_l'))
    )

    master_r_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='master_r',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('use_master_r'))
    )

    slave_l_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='slave_l',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('use_slave_l'))
    )

    slave_r_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='slave_r',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('use_slave_r'))
    )

    return LaunchDescription([
        use_master_l_arg,
        use_master_r_arg,
        use_slave_l_arg,
        use_slave_r_arg,
        master_l_node,
        master_r_node,
        slave_l_node,
        slave_r_node,
    ])
