import os
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    base_path = os.path.expanduser("~/Code/ros2_vdbgpdf/src/vdb_gpdf_mapping")  # <-- points to source, not install
    config_file = os.path.join(base_path, 'config', 'vdb_gpdf_mapping_cow.yaml')
    rviz_config_file = os.path.join(base_path, 'rviz', 'vdb_mapping_camera_cow.rviz')

    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/home/lan/Data/converted_ros2_bag/converted_ros2_bag.db3',
        description='Full path to ROS2 bag file to play'
    )

    return LaunchDescription([
        bag_file_arg,

        Node(
            package='vdb_gpdf_mapping',
            executable='vdb_gpdf_mapping_node',
            name='vdb_gpdf_mapping_node',
            parameters=[ParameterFile(config_file, allow_substs=True)],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_mapper',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file'), '--clock', '-r', '1.0'],
            output='screen'
        )
    ])
