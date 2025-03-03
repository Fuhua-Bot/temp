#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, Quaternion
import sys
import open3d as o3d  # 使用open3d替代pcl，因为pcl的Python绑定不稳定


class CloudProcessor(Node):
    def __init__(self):
        super().__init__("cloud_processor")

        # 创建订阅者，订阅点云数据
        self.subscription = self.create_subscription(
            PointCloud2, "/cloud_map", self.listener_callback, 10
        )

        # 发布 costmap 消息
        self.costmap_publisher = self.create_publisher(OccupancyGrid, "/filtered_costmap", 10)

        # 定时器以控制更新频率
        self.timer = self.create_timer(2.0, self.timer_callback)  # 2秒，0.5Hz

        # 创建一个 OccupancyGrid 用于发布 costmap
        self.costmap = OccupancyGrid()
        self.costmap.header.frame_id = "map"
        
        # 设置costmap原点位置和方向
        self.costmap.info.origin = Pose()
        self.costmap.info.origin.position.x = -5.0  # 根据实际情况调整
        self.costmap.info.origin.position.y = -5.0  # 根据实际情况调整
        self.costmap.info.origin.position.z = 0.0
        self.costmap.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.costmap.info.resolution = 0.1  # 每个像素的分辨率
        self.costmap.info.width = 100  # costmap宽度
        self.costmap.info.height = 100  # costmap高度
        
        # 初始化costmap数据为未知（-1）
        self.costmap.data = [-1] * (self.costmap.info.width * self.costmap.info.height)
        
        # 提前给出日志消息
        self.get_logger().info("CloudProcessor initialized. Waiting for point cloud data...")

    def listener_callback(self, msg):
        self.get_logger().info("Received point cloud data, processing...")
        
        try:
            # 从 PointCloud2 消息中提取点云数据
            cloud_points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                cloud_points.append([point[0], point[1], point[2]])
            
            # 检查点云是否为空
            if len(cloud_points) == 0:
                self.get_logger().warn("Received empty point cloud.")
                return
                
            # 转换为NumPy数组
            cloud_np = np.array(cloud_points, dtype=np.float32)
            self.get_logger().info(f"Processed {len(cloud_points)} points from point cloud.")
            
            # 转换为Open3D点云格式
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(cloud_np)
            
            # 降采样点云（可选）
            pcd = pcd.voxel_down_sample(voxel_size=0.1)
            
            # 计算法向量
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=50))
            
            # 检测坡度大于25度的区域
            steep_areas = self.detect_steep_areas(pcd)
            
            # 更新并发布costmap
            self.update_costmap(steep_areas)
            
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def detect_steep_areas(self, pcd):
        steep_areas = []
        
        # 获取所有点及其法向量
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        
        # 检测每个点的坡度（法向量与z轴的夹角）
        for i in range(len(points)):
            # 计算法向量与z轴的夹角
            normal = normals[i]
            z_axis = np.array([0, 0, 1])
            
            # 确保法向量是单位向量
            normal_norm = np.linalg.norm(normal)
            if normal_norm > 0:
                normal = normal / normal_norm
                
            # 计算夹角（弧度），然后转换为角度
            angle_rad = np.arccos(np.clip(np.dot(normal, z_axis), -1.0, 1.0))
            angle_deg = np.degrees(angle_rad)
            
            # 如果角度大于25度，认为是陡坡或坑
            if angle_deg > 5.0:
                steep_areas.append(points[i])
        
        self.get_logger().info(f"Detected {len(steep_areas)} steep areas (>25°)")
        return steep_areas

    def update_costmap(self, steep_areas):
        # 重置costmap数据为未知（-1）
        self.costmap.data = [-1] * (self.costmap.info.width * self.costmap.info.height)
        
        # 将检测到的陡坡区域标记为障碍物
        for point in steep_areas:
            # 将点的坐标转换为costmap网格索引
            grid_x = int((point[0] - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
            grid_y = int((point[1] - self.costmap.info.origin.position.y) / self.costmap.info.resolution)
            
            # 确保索引在costmap范围内
            if 0 <= grid_x < self.costmap.info.width and 0 <= grid_y < self.costmap.info.height:
                # 计算一维数组索引
                index = grid_y * self.costmap.info.width + grid_x
                self.costmap.data[index] = 100  # 设置为障碍物（100）
        
        # 更新时间戳并发布costmap
        self.costmap.header.stamp = self.get_clock().now().to_msg()
        self.costmap_publisher.publish(self.costmap)
        self.get_logger().info("Published updated costmap")

    def timer_callback(self):
        self.get_logger().info("Timer triggered (0.5Hz)")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CloudProcessor()
        print("Node created successfully, spinning...")
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in main: {str(e)}", file=sys.stderr)
    finally:
        # 清理资源
        rclpy.shutdown()


if __name__ == "__main__":
    main()