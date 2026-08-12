"""
Intervention relay test using a real HDF5 recording as the "inference" source.

Nodes started:
  1. hdf5_playback_pub       — reads a collected .hdf5 file, publishes to /YAM_INFER_L/R
  2. mock_master_pub         — fake master-arm status on /master_l/r_status
  3. intervention_relay_node — the relay node under test

Usage
-----
  pixi run ros2 launch inference test_intervention_playback.launch.py \
      hdf5_path:=/home/user/data/collection_1234.hdf5

Optional arguments:
  loop:=false       Play the episode once instead of looping.
  speed:=0.5        Play back at half speed.

To observe the relay output:
  pixi run ros2 topic echo /YAM_VR_L

Switch modes in the launch terminal:
  S  →  intervention (mock master arm controls slave)
  D  →  inference    (HDF5 playback controls slave)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    hdf5_path_arg = DeclareLaunchArgument(
        'hdf5_path',
        description='Absolute path to the collected .hdf5 file (required)'
    )
    loop_arg = DeclareLaunchArgument(
        'loop', default_value='false',
        description='Loop the episode (true/false)'
    )
    speed_arg = DeclareLaunchArgument(
        'speed', default_value='1.0',
        description='Playback speed multiplier (0.5 = half speed)'
    )

    hdf5_playback_pub = Node(
        package='inference',
        executable='hdf5_playback_pub',
        name='hdf5_playback_pub',
        output='screen',
        parameters=[{
            'hdf5_path': LaunchConfiguration('hdf5_path'),
            'loop':      LaunchConfiguration('loop'),
            'speed':     LaunchConfiguration('speed'),
        }],
    )

    mock_master_pub = Node(
        package='inference',
        executable='mock_master_pub',
        name='mock_master_pub',
        output='screen',
    )

    intervention_relay_node = Node(
        package='inference',
        executable='intervention_relay_node',
        name='intervention_relay_node',
        output='screen',
    )

    return LaunchDescription([
        hdf5_path_arg,
        loop_arg,
        speed_arg,
        hdf5_playback_pub,
        mock_master_pub,
        intervention_relay_node,
    ])
