import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    arm_params_file = os.path.join(
        get_package_share_directory('yam_damiao_controller'),
        'config',
        'master_slave_infer.yaml'
    )

    master_l_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='master_l',
        output='screen',
        parameters=[arm_params_file],
    )

    master_r_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='master_r',
        output='screen',
        parameters=[arm_params_file],
    )

    slave_l_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='slave_l',
        output='screen',
        parameters=[arm_params_file],
    )

    slave_r_node = Node(
        package='yam_damiao_controller',
        executable='YamController',
        name='slave_r',
        output='screen',
        parameters=[arm_params_file],
    )

    # joint_base_node publishes arm status over ZMQ and receives ML commands,
    # forwarding them to /YAM_INFER_L/R (picked up by intervention_relay_node).
    joint_base_node = Node(
        package='inference',
        executable='joint_base_node',
        name='joint_base_node',
        output='screen',
        parameters=[{
            'arm_type': 'yam',
            'left_cmd_topic': '/YAM_INFER_L',
            'right_cmd_topic': '/YAM_INFER_R',
        }],
    )

    intervention_relay_node = Node(
        package='inference',
        executable='intervention_relay_node',
        name='intervention_relay_node',
        output='screen',
    )

    camera_node = Node(
        package='inference',
        executable='camera_node',
        name='camera_node',
        output='screen',
    )

    return LaunchDescription([
        master_l_node,
        master_r_node,
        slave_l_node,
        slave_r_node,
        joint_base_node,
        intervention_relay_node,
        camera_node,
    ])
