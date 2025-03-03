import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl_ros
from pcl import PointCloud
from pcl.pcl_visualization import CloudViewing
from pcl.pcl_filter import VoxelGrid
from pcl.pcl_features import NormalEstimation
from pcl.pcl_surface import GreedyProjectionTriangulation

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')

        # 创建订阅器
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth/rgbd_camera/points',
            self.listener_callback,
            10
        )

        # 创建发布器
        self.publisher = self.create_publisher(PointCloud2, '/keepout_points', 10)

    def listener_callback(self, msg):
        # 将PointCloud2消息转换为numpy数组
        cloud_points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(cloud_points))

        # 下采样点云（体素滤波）
        downsampled_points = self.downsample_point_cloud(points)

        # 提取坡度大于10度的点
        keepout_points = self.extract_slopes(downsampled_points)

        # 将结果转化为PointCloud2格式
        result_msg = self.create_point_cloud_msg(keepout_points, msg.header)

        # 发布结果
        self.publisher.publish(result_msg)

    def downsample_point_cloud(self, points):
        """使用体素网格过滤器下采样点云"""
        cloud = pcl.PointCloud()
        cloud.from_array(points.astype(np.float32))

        # 设置体素网格过滤器
        voxel_filter = cloud.make_voxel_grid_filter()
        voxel_filter.set_leaf_size(0.05, 0.05, 0.05)

        # 应用滤波器
        downsampled_cloud = voxel_filter.filter()

        return downsampled_cloud.to_array()

    def extract_slopes(self, points):
        """提取坡度大于10度的点云"""
        cloud = pcl.PointCloud()
        cloud.from_array(points.astype(np.float32))

        # 计算法线
        normal_estimation = cloud.make_normal_estimation()
        normal_estimation.set_k_search(50)
        normals = normal_estimation.compute()

        # 计算坡度
        keepout_points = []
        for i in range(len(normals)):
            normal = normals[i]
            slope_angle = np.arccos(normal[2]) * (180.0 / np.pi)
            if slope_angle > 10.0:
                keepout_points.append(points[i])

        return np.array(keepout_points)

    def create_point_cloud_msg(self, points, header):
        """将numpy数组转换为PointCloud2消息"""
        point_cloud_data = pc2.create_cloud_xyz32(header, points)
        return point_cloud_data

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
