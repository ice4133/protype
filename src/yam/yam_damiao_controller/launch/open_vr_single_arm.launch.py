import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the parameter file
    params_file = os.path.join(
        get_package_share_directory('yam_damiao_controller'), 
        'config', 
        'vr_double_arm.yaml'
    )

    # Declare launch arguments to make the arms optional
    use_left_arm_arg = DeclareLaunchArgument(
        'use_left_arm',
        default_value='false',
        description='Set to true to launch the left arm node.'
    )
    
    use_right_arm_arg = DeclareLaunchArgument(
        'use_right_arm',
        default_value='true',
        description='Set to true to launch the right arm node.'
    )

    # Main command publisher node
    yam_cmd_node = Node(
        package='yam_cmd_pub',
        executable='yam_cmd_pub',
        name='yam_cmd_pub',
        parameters=[
            {'use_left_arm': LaunchConfiguration('use_left_arm')},
            {'use_right_arm': LaunchConfiguration('use_right_arm')}
        ],
        output='screen',
    )

    # Left arm controller node with a condition
    arm_l_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='yam_l',
        output='screen',
        parameters=[params_file],
        # The node will only run if 'use_left_arm' is true
        condition=IfCondition(LaunchConfiguration('use_left_arm'))
    )

    # Right arm controller node with a condition
    arm_r_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='yam_r',
        output='screen',
        parameters=[params_file],
        # The node will only run if 'use_right_arm' is true
        condition=IfCondition(LaunchConfiguration('use_right_arm'))
    )

    return LaunchDescription([
        use_left_arm_arg,
        use_right_arm_arg,
        yam_cmd_node,
        arm_l_node,
        arm_r_node,
    ])