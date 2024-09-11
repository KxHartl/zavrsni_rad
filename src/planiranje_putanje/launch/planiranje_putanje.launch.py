from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz2_base = os.path.join(get_package_share_directory('planiranje_putanje'), 'rviz')
    rviz2_full_config = os.path.join(rviz2_base, 'planiranje_putanje.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('marvelmind_ros2'),
                    'launch',
                    'marvelmind_ros2.launch.py'
                ])
            ]),

            launch_arguments={}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('astro'),
                    'launch',
                    'rsp.launch.py'
                ])
            ]),

            launch_arguments={}.items()
        ),

        Node(
            package='planiranje_putanje',
            executable='hedgehog_to_pose',
            name='hedgehog_to_pose',
            arguments=[]
        ),

        Node(
            package='planiranje_putanje',
            executable='odom_to_tf',
            name='odom_to_tf',
            arguments=[]
        ),

        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz2_full_config]
        )
        
    ])