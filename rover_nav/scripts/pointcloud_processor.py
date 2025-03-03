import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d

class KeepoutFilter(Node):
    def __init__(self):
        super().__init__('keepout_filter')
        
        # 订阅器和发布器的初始化
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth/rgbd_camera/points',
            self.callback,
            10)
        self.publisher = self.create_publisher(
            PointCloud2,
            '/keepout_points',
            10)

        # 下采样参数配置
        self.voxel_size = 0.05  # 体素大小（根据实际场景调整）
        self.normal_radius = 0.1  # 法线计算搜索半径
        self.normal_max_nn = 30  # 最大邻近点数
        self.slope_threshold = 10.0  # 坡度阈值（度）

        self.get_logger().info("Keepout filter node initialized")

    def callback(self, msg):
        try:
            # Step 1: 将PointCloud2转换为numpy数组
            points = pc2.read_points_numpy(msg, field_names=("x", "y", "z"))
            xyz = points[:, :3]

            if xyz.shape[0] == 0:
                self.get_logger().warn("Received empty point cloud")
                return

            # Step 2: 创建Open3D点云并下采样
            o3d_pcd = o3d.geometry.PointCloud()
            o3d_pcd.points = o3d.utility.Vector3dVector(xyz)
            down_pcd = o3d_pcd.voxel_down_sample(self.voxel_size)

            if len(down_pcd.points) == 0:
                self.get_logger().warn("Downsampled point cloud is empty")
                return

            # Step 3: 计算法线
            down_pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.normal_radius,
                    max_nn=self.normal_max_nn
                )
            )

            # Step 4: 法线分析和坡度过滤
            normals = np.asarray(down_pcd.normals)
            z_axis = np.array([0, 0, 1])
            cos_theta = np.abs(normals @ z_axis)
            angles_deg = np.degrees(np.arccos(cos_theta))
            mask = angles_deg > self.slope_threshold

            filtered_points = np.asarray(down_pcd.points)[mask]

            # Step 5: 创建并发布结果点云
            if filtered_points.size == 0:
                self.get_logger().debug("No points satisfy keepout condition")
                filtered_points = np.empty((0, 3))  # 发布空点云

            # 转换回PointCloud2格式
            header = msg.header
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            cloud_data = filtered_points.astype(np.float32).tobytes()
            pc2_msg = pc2.create_cloud(header, fields, filtered_points)
            self.publisher.publish(pc2_msg)

        except Exception as e:
            self.get