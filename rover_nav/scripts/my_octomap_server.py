#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from octomap_msgs.srv import GetOctomap
from octomap_msgs.msg import Octomap
from std_msgs.msg import Header
import time

class OctomapServer(Node):
    def __init__(self):
        super().__init__('octomap_bridge')
        
        # 订阅 /octomap_binary 话题
        self.octomap_subscriber = self.create_subscription(
            Octomap,
            '/octomap_binary',
            self.octomap_callback,
            10)
        
        # 创建服务，服务名字改为 /octomap_binary
        self.srv = self.create_service(GetOctomap, '/octomap_binary', self.get_octomap_callback)
        
        # 存储接收到的八叉树数据
        self.octomap_data = None

    def octomap_callback(self, msg):
        self.get_logger().info('Received octomap binary data')
        self.octomap_data = msg

    def get_octomap_callback(self, request, response):
        self.get_logger().info('Gridmap requests the service')
        if self.octomap_data is None:
            self.get_logger().warn('No octomap data available')
            return response
        
        # 填充返回的 Octomap 数据
        response.map.header = Header()
        response.map.header.stamp = self.get_clock().now().to_msg()
        response.map.header.frame_id = "map"  # 假设使用 "map" 作为坐标系
        
        # 设置一些基础字段
        response.map.binary = True
        response.map.id = "octomap"
        response.map.resolution = 0.05  # 八叉树分辨率，假设是0.05m
        
        # 假设 octomap_data 是接收到的二进制数据
        response.map.data = self.octomap_data.data

        self.get_logger().info('Returning octomap data')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = OctomapServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
