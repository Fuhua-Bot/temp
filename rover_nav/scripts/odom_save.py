import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        
        # 订阅三个话题
        self.chassis_odom_sub = self.create_subscription(Odometry, 'chassis_odom', self.chassis_odom_callback, 10)
        self.visual_odom_sub = self.create_subscription(Odometry, 'visual_odom', self.visual_odom_callback, 10)
        self.gt_odom_sub = self.create_subscription(Odometry, 'gt_odom', self.gt_odom_callback, 10)

        # 存储接收到的 Odometry 数据
        self.chassis_odom = None
        self.visual_odom = None
        self.gt_odom = None

        # 保存数据的文件
        self.file = open('odom_data.txt', 'w')

    def chassis_odom_callback(self, msg: Odometry):
        self.get_logger().info("chassis_odom")
        self.chassis_odom = msg
        self.check_and_save_data()

    def visual_odom_callback(self, msg: Odometry):
        print("visual_odom")
        self.visual_odom = msg
        self.check_and_save_data()

    def gt_odom_callback(self, msg: Odometry):
        print("gt_odom")

        self.gt_odom = msg
        self.check_and_save_data()

    def check_and_save_data(self):
        # 如果三个话题的数据都已接收
        if self.chassis_odom and self.visual_odom and self.gt_odom:
            
            chassis_xyz = self.chassis_odom.pose.pose.position
            visual_xyz = self.visual_odom.pose.pose.position
            gt_xyz = self.gt_odom.pose.pose.position
            self.file.write(f"{chassis_xyz.x-30.0} {chassis_xyz.y-60.0} {chassis_xyz.z} \t")
            self.file.write(f"{visual_xyz.x} {visual_xyz.y+60.0} {visual_xyz.z}\t")
            self.file.write(f"{gt_xyz.x-30.0} {gt_xyz.y-60.0} {gt_xyz.z} \n")
            
            # 清空存储的 odom 数据
            self.chassis_odom = None
            self.visual_odom = None
            self.gt_odom = None

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)
    odom_subscriber.file.close()

if __name__ == '__main__':
    main()

