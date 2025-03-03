from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import os

def launch_setup(context, *args, **kwargs):

    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup')

    nav2_params_file = PathJoinSubstitution(
        # [FindPackageShare('rover_nav'), 'params', 'rgbd_nav2_params.yaml']
        ['/home/fuhua/Rover/src/rover_nav/params/rgbd_nav2_params.yaml']
    )

    # Paths
    
    nav2_launch = PathJoinSubstitution(
    [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])


    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('params_file', nav2_params_file),
            ('remappings', '/map:=/map' )
        ]
    )

    rviz_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'rviz_launch.py'])

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch])
    )
    return [
        # Nodes to launch
        nav2,
        rviz
    ]

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),
        OpaqueFunction(function=launch_setup)
    ])