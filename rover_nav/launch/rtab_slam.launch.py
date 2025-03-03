from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    qos = LaunchConfiguration("qos")
    localization = LaunchConfiguration("localization")

    parameters = {
        "frame_id": "base_link",
        "use_sim_time": use_sim_time,
        "subscribe_depth": True,
        "subscribe_imu": True,
        "use_action_for_goal": True,
        'subscribe_odom': True,
        'odom_topic': '/chassis_odom',
        'use_odom_sensor': True,
        "qos_image": qos,
        "qos_imu": qos,
        "wait_for_transform": 1.0,
        "wait_imu_to_init": True,
        "tf_tolerance": 1.0,
        "Reg/Force3DoF": "false",
        "approx_sync": True,
        "sync_queue_size": 10,
        "Optimizer/GravitySigma": "0",  # Disable imu constraints (we are already in 2D)
    }

    remappings = [
        ("odom", "/chassis_odom"),
        ("imu", "/imu/data"),
        ("rgb/image", "/depth/rgbd_camera/image_raw"),
        ("rgb/camera_info", "/depth/rgbd_camera/depth/camera_info"),
        ("depth/image", "/depth/rgbd_camera/depth/image_raw"),
    ]

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "qos", default_value="2", description="QoS used for input sensor topics"
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="false",
                description="Launch in localization mode.",
            ),
            # Nodes to launch
            # SLAM mode:
            Node(
                condition=UnlessCondition(localization),
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[parameters],
                remappings=remappings,
                arguments=["-d"],
            ),  # This will delete the previous database (~/.ros/rtabmap.db)
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                output="screen",
                parameters=[{"approx_sync": False, "use_sim_time": use_sim_time}],
                remappings=remappings,
            ),
            # Localization mode:
            Node(
                condition=IfCondition(localization),
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[
                    parameters,
                    {
                        "Mem/IncrementalMemory": "False",
                        "Mem/InitWMWithAllNodes": "True",
                    },
                ],
                remappings=remappings,
            ),
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=[parameters],
                remappings=remappings,
            ),
            # Node(
            #     package="rtabmap_odom",
            #     executable="rgbd_odometry",
            #     output="screen",
            #     parameters=[parameters],
            #     remappings=remappings,
            # ),
            # Node(
            #     package="rtabmap_util",
            #     executable="point_cloud_xyz",
            #     output="screen",
            #     parameters=[{"decimation": 2, "max_depth": 3.0, "voxel_size": 0.02}],
            #     remappings=remappings,
            # ),
            # Node(
            #     package="rtabmap_util",
            #     executable="obstacles_detection",
            #     output="screen",
            #     parameters=[parameters],
            #     remappings=[
            #         ("cloud", "/camera/cloud"),
            #         ("obstacles", "/camera/obstacles"),
            #         ("ground", "/camera/ground"),
            #     ],
            # ),
        ]
    )
