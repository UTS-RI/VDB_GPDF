import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Resolve package share directory to locate config and RViz files
    pkg_share = get_package_share_directory('vdb_gpdf_mapping')

    config_file = os.path.join(pkg_share, 'config', 'vdb_gpdf_mapping_cow.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'vdb_mapping_camera_cow.rviz')

    # Bag file argument (default points to the Docker-mounted path)
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/workspace/data/converted_ros2_bag.db3',
        description='Full path to the ROS2 bag file to play'
    )

    return LaunchDescription([
        bag_file_arg,

        # Main VDB-GPDF mapping node
        Node(
            package='vdb_gpdf_mapping',
            executable='vdb_gpdf_mapping_node',
            name='vdb_gpdf_mapping_node',
            parameters=[ParameterFile(config_file, allow_substs=True)],
            output='screen'
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_mapper',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        # ROS 2 bag playback
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                LaunchConfiguration('bag_file'),
                '--clock',
                '-r', '1.0'
            ],
            output='screen'
        )
    ])
