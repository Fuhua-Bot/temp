from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 EKF 节点
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                # 基本参数
                {
                    'frequency': 50.0,                 # 发布频率
                    'sensor_timeout': 0.5,             # 传感器超时时间
                    'two_d_mode': False,                # 如果是平面运动设为True
                    'odom_frame': 'filtered_odom',              # 输出里程计帧
                    'base_link_frame': 'base_link',    # 机器人基座帧
                    'world_frame': 'odom',             # 全局参考帧（与传感器一致）
                    
                    # 输入话题配置
                    'odom0': 'chasis_odom',
                    'odom1': 'visual_odom',
                    'imu0': 'imu/data',

                    # 坐标系修正（如果传感器坐标系不同需要设置）
                    # 'odom0_config': [x, y, z, roll, pitch, yaw] 方差控制
                    # 每个传感器的数据使用情况（true表示使用该维度）
                    'odom0_config': [
                        True,  True,  True,   # X, Y, Z position
                        True, True, True,    # Roll, Pitch, Yaw
                        True,  True,  True,   # X vel, Y vel, Z vel
                        False, False, True     # Roll vel, Pitch vel, Yaw vel
                    ],
                    'odom1_config': [
                        True,  True,  True,   # X, Y, Z position
                        True, True, True,    # Roll, Pitch, Yaw
                        True,  True,  True,   # X vel, Y vel, Z vel
                        False, False, True     # Roll vel, Pitch vel, Yaw vel
                    ],
                    'imu0_config': [
                        False, False, False,  # X, Y, Z position (IMU无位置信息)
                        True,  True,  True,    # Roll, Pitch, Yaw（角度或角速度）
                        True,  True,  True,    # 线加速度
                        False, False, False    # Velocity（IMU一般无速度信息）
                    ],

                    # 协方差参数（根据传感器噪声调整）
                    'odom0_differential': False,
                    'odom1_differential': False,
                    'imu0_differential': True,

                    # 初始状态协方差（根据实际情况调整）
                    'initial_state': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    'initial_covariance': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],

                    # 过程噪声协方差（需要根据实际系统调整）
                    'process_noise_covariance': [
                        0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                        0,    0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                        0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                        0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                        0,    0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                        0,    0,    0,    0,    0,    0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,
                    ]
                }
            ],
            # 重映射输入话题（如果需要）
            remappings=[
                ('odom0', '/chasis_odom'),
                ('odom1', '/odom'),
                ('imu0', '/imu/data')
            ]
        )
    ])
