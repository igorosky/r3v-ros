# Both nodes in one process: point clouds and TF in, rendered images out.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    parameters = [
        LaunchConfiguration('config'),
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('r3v_ros'), 'config', 'r3v.yaml']),
            description='Parameter file of both nodes'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'compression', default_value='none',
            description='voxel_stream.compression of the voxelizer: none (the stream stays '
                        'in the process) or zstd (renderers elsewhere follow it too)'),
        # The voxel stream goes from one node to the other within the
        # process, without the middleware.
        ComposableNodeContainer(
            name='r3v',
            namespace=namespace,
            package='rclcpp_components',
            # Point clouds wait for TF, which arrives on another callback group.
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='r3v_ros',
                    plugin='r3v_ros::VoxelizerNode',
                    name='r3v_voxelizer',
                    namespace=namespace,
                    parameters=parameters + [
                        {'voxel_stream.compression': LaunchConfiguration('compression')},
                    ],
                ),
                ComposableNode(
                    package='r3v_ros',
                    plugin='r3v_ros::RendererNode',
                    name='r3v_renderer',
                    namespace=namespace,
                    parameters=parameters,
                ),
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
