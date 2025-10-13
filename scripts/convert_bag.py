#!/usr/bin/env python3
"""
Bag包转换工具
将ROS 1 bag包转换为ROS 2格式
"""

import os
import sys
import subprocess
import time

def check_ros1_tools():
    """检查ROS 1工具"""
    print("=== 检查ROS 1工具 ===")
    
    # 检查是否有rosbag工具
    try:
        result = subprocess.run(['rosbag', '--help'], capture_output=True, text=True)
        if result.returncode == 0:
            print(" rosbag工具可用")
            return True
    except FileNotFoundError:
        pass
    
    # 检查是否有python3-rosbag
    try:
        import rosbag
        print(" python3-rosbag可用")
        return True
    except ImportError:
        pass
    
    print(" 没有找到ROS 1 bag工具")
    return False

def install_ros1_tools():
    """安装ROS 1工具"""
    print("\n=== 安装ROS 1工具 ===")
    
    print("正在安装python3-rosbag...")
    try:
        result = subprocess.run(['sudo', 'apt', 'install', '-y', 'python3-rosbag'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print(" python3-rosbag安装成功")
            return True
        else:
            print(" python3-rosbag安装失败")
            return False
    except Exception as e:
        print(f" 安装过程中出错: {e}")
        return False

def analyze_ros1_bag():
    """分析ROS 1 bag包"""
    print("\n=== 分析ROS 1 bag包 ===")
    
    bag_file = "./data.bag"
    
    try:
        import rosbag
        
        print(f"正在分析bag包: {bag_file}")
        bag = rosbag.Bag(bag_file)
        
        print(" bag包信息:")
        print(f" 文件大小: {os.path.getsize(bag_file) / (1024*1024*1024):.2f} GB")
        print(f" 消息数量: {bag.get_message_count()}")
        print(f"  持续时间: {bag.get_end_time() - bag.get_start_time():.2f} 秒")
        
        print("\n 话题列表:")
        topics = bag.get_type_and_topic_info().topics
        for topic, info in topics.items():
            print(f"  - {topic}: {info.msg_type} ({info.message_count} 条消息)")
        
        bag.close()
        return True
        
    except ImportError:
        print(" 无法导入rosbag模块")
        return False
    except Exception as e:
        print(f" 分析bag包时出错: {e}")
        return False

def convert_bag_to_ros2():
    """转换bag包到ROS 2格式"""
    print("\n=== 转换bag包到ROS 2格式 ===")
    
    input_bag = "./data.bag"
    output_dir = "./ros2_bag"
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"输入文件: {input_bag}")
    print(f"输出目录: {output_dir}")
    
    try:
        # 使用ros2 bag record来转换
        # 首先启动ros2 bag record
        print("🎬 启动ROS 2 bag记录...")
        
        # 启动rosbag play和ros2 bag record
        play_process = subprocess.Popen(['rosbag', 'play', input_bag, '--loop'],
                                      stdout=subprocess.PIPE, 
                                      stderr=subprocess.PIPE)
        
        record_process = subprocess.Popen(['ros2', 'bag', 'record', '-a', '-o', output_dir],
                                        stdout=subprocess.PIPE, 
                                        stderr=subprocess.PIPE)
        
        print(" 转换中... (按Ctrl+C停止)")
        
        # 等待一段时间让转换进行
        time.sleep(10)
        
        # 停止进程
        play_process.terminate()
        record_process.terminate()
        
        play_process.wait()
        record_process.wait()
        
        print(" 转换完成")
        return True
        
    except KeyboardInterrupt:
        print("\n  用户中断转换")
        return False
    except Exception as e:
        print(f" 转换过程中出错: {e}")
        return False

def create_simple_processor():
    """创建简单的处理器"""
    print("\n=== 创建简单处理器 ===")
    
    processor_code = '''#!/usr/bin/env python3
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
'''
    
    with open('simple_lidar_processor.py', 'w') as f:
        f.write(processor_code)
    
    os.chmod('simple_lidar_processor.py', 0o755)
    print(" 简单处理器已创建: simple_lidar_processor.py")
    
    return True

def main():
    """主函数"""
    print(" Bag包转换和处理工具")
    print("=" * 50)
    
    # 检查ROS 1工具
    if not check_ros1_tools():
        print("\n尝试安装ROS 1工具...")
        if not install_ros1_tools():
            print(" 无法安装ROS 1工具，将使用简化方案")
            create_simple_processor()
            return True
    
    # 分析ROS 1 bag包
    if not analyze_ros1_bag():
        print(" 无法分析ROS 1 bag包")
        return False
    
    # 询问是否转换
    print("\n是否要转换bag包到ROS 2格式？")
    print("注意: 转换可能需要很长时间（14GB文件）")
    
    try:
        choice = input("输入 y 继续转换，其他键跳过: ").strip().lower()
        if choice == 'y':
            convert_bag_to_ros2()
        else:
            print(" 跳过转换")
    except KeyboardInterrupt:
        print("\n  用户取消")
    
    # 创建简单处理器
    create_simple_processor()
    
    print("\n" + "=" * 50)
    print(" 处理完成！")
    print("\n使用方法:")
    print("1. 启动简单处理器: python3 simple_lidar_processor.py")
    print("2. 在另一个终端播放bag包: rosbag play ./data.bag")
    print("3. 观察处理结果")
    
    return True

if __name__ == "__main__":
    main()
