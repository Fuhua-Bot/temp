from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")
    rtabmap_viz = LaunchConfiguration("rtabmap_viz")

    # 视觉里程计参数
    vo_parameters = {
        "frame_id": "base_link",
        "odom_frame_id": "visual_odom",
        "publish_tf": True,
        "approx_sync": True,
        "queue_size": 30,
        "subscribe_imu": True,
        "wait_imu_to_init": True,
        "wait_for_transform": 0.3,
        "tf_delay": 0.05,
        "tf_tolerance": 0.1,
    }

    # RTAB-Map参数
    rtabmap_parameters = {
        "subscribe_rgbd": True,
        "subscribe_imu": True,
        "use_action_for_goal": True,
        "odom_sensor_sync": True,
        "wait_for_transform": 0.3,
        "tf_tolerance": 0.1,
        "publish_tf": True,
        # 启用多源里程计融合
        "Odom/Strategy": "1",  # 0=Frame-to-Map (F2M), 1=Frame-to-KeyFrame (F2K)
        "Odom/FusionMode": "1",  # 0=disabled, 1=kalman, 2=complementary
        "Mem/NotLinkedNodesKept": "false",
    }

    # 共享参数
    shared_parameters = {
        "frame_id": "base_link",
        "use_sim_time": use_sim_time,
        "Reg/Strategy": "1",
        "Reg/Force3DoF": "false",
    }

    # 话题重映射
    remappings = [
        ("odom", "/odom"),
        ("imu", "/imu/data"),  # 轮式里程计话题  # IMU话题
        ("rgb/image", "/depth/rgbd_camera/image_raw"),
        ("rgb/camera_info", "/depth/rgbd_camera/depth/camera_info"),
        ("depth/image", "/depth/rgbd_camera/depth/image_raw"),
    ]

    return LaunchDescription(
        [
            # Launch参数
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="false",
                description="Launch in localization mode",
            ),
            DeclareLaunchArgument(
                "rtabmap_viz", default_value="true", description="Launch visualization"
            ),
            # RGBD同步节点
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                output="screen",
                parameters=[{"approx_sync": False, "use_sim_time": use_sim_time}],
                remappings=remappings,
            ),
            # 视觉里程计节点
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="screen",
                parameters=[vo_parameters, shared_parameters],
                remappings=remappings,
            ),
            # SLAM模式
            Node(
                condition=UnlessCondition(localization),
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[rtabmap_parameters, shared_parameters],
                remappings=remappings,
                arguments=["-d"],
            ),
            # 定位模式
            Node(
                condition=IfCondition(localization),
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[
                    rtabmap_parameters,
                    shared_parameters,
                    {
                        "Mem/IncrementalMemory": "False",
                        "Mem/InitWMWithAllNodes": "True",
                    },
                ],
                remappings=remappings,
            ),
            # 可视化
            Node(
                condition=IfCondition(rtabmap_viz),
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=[rtabmap_parameters, shared_parameters],
                remappings=remappings,
            ),
        ]
    )
