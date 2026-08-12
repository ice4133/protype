from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package path
    xhand_pkg_path = get_package_share_path('yam_description')

    # Path configurations
    default_model_path = xhand_pkg_path / 'urdf/yam.urdf'
    default_rviz_config_path = xhand_pkg_path / 'rviz/yam.rviz'

    # # Launch arguments
    # gui_arg = DeclareLaunchArgument(
    #     name='gui',
    #     default_value='false',
    #     choices=['true', 'false'],
    #     description='Enable joint state publisher GUI'
    # )

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Path to YAM URDF file'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Path to RViz config file'
    )

    # Robot description parameter
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    init_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='init_joint_state_publisher',
        # parameters=[{'source_list': []}],   # don't subscribe to anything
        output='screen'
    )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )

    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        # gui_arg,
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        init_joint_state_publisher_node,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node
    ])
