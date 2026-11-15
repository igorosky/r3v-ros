# The renderer alone: the voxel stream, TF and robot_description in, images out (e.g. on a base station).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('r3v_ros'), 'config', 'r3v.yaml']),
            description='Parameter file of both nodes'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='r3v_ros',
            executable='r3v_renderer_node',
            name='r3v_renderer',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                LaunchConfiguration('config'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
