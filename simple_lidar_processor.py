#!/usr/bin/env python3
"""
简单的激光雷达数据处理节点
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import time

class SimpleLidarProcessor(Node):
    def __init__(self):
        super().__init__('simple_lidar_processor')
        
        # 订阅点云话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',  # 常见的激光雷达话题
            self.point_cloud_callback,
            10
        )
        
        # 也尝试其他可能的话题
        self.subscription2 = self.create_subscription(
            PointCloud2,
            '/points_raw',
            self.point_cloud_callback,
            10
        )
        
        self.subscription3 = self.create_subscription(
            PointCloud2,
            '/cloud',
            self.point_cloud_callback,
            10
        )
        
        self.point_count = 0
        self.start_time = time.time()
        
        self.get_logger().info('简单激光雷达处理器已启动')
    
    def point_cloud_callback(self, msg):
        self.point_count += 1
        
        if self.point_count % 10 == 0:  # 每10帧打印一次
            elapsed = time.time() - self.start_time
            self.get_logger().info(f'已处理 {self.point_count} 帧点云，耗时 {elapsed:.2f} 秒')

def main(args=None):
    rclpy.init(args=args)
    
    processor = SimpleLidarProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
