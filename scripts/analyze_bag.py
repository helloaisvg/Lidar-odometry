#!/usr/bin/env python3
"""
Bag包分析工具
用于分析bag包内容并提取激光雷达数据
"""

import sys
import os
import struct
import time
from pathlib import Path

def analyze_bag_file(bag_file):
    """分析bag包文件"""
    print(f"分析bag包: {bag_file}")
    
    if not os.path.exists(bag_file):
        print(f"错误: 文件不存在 {bag_file}")
        return False
    
    file_size = os.path.getsize(bag_file)
    print(f"文件大小: {file_size / (1024*1024*1024):.2f} GB")
    
    # 尝试读取bag包头部信息
    try:
        with open(bag_file, 'rb') as f:
            # 读取bag包头部
            header = f.read(13)
            if header.startswith(b'#ROSBAG'):
                print("✅ 这是一个有效的ROS bag包")
                return True
            else:
                print("❌ 这不是一个有效的ROS bag包")
                return False
    except Exception as e:
        print(f"❌ 读取文件时出错: {e}")
        return False

def extract_pointcloud_info(bag_file):
    """提取点云信息"""
    print("\n=== 点云数据提取 ===")
    
    print("建议使用以下方法处理bag包:")
    print("1. 使用ROS环境播放bag包")
    print("2. 使用我们的里程计节点处理")
    print("3. 转换为其他格式进行分析")
    
    return True

def main():
    if len(sys.argv) < 2:
        print("用法: python3 analyze_bag.py <bag_file>")
        print("示例: python3 analyze_bag.py /home/sss/data.bag")
        sys.exit(1)
    
    bag_file = sys.argv[1]
    
    print("=== Bag包分析工具 ===")
    print(f"分析文件: {bag_file}")
    
    # 分析bag包
    if analyze_bag_file(bag_file):
        extract_pointcloud_info(bag_file)
        
        print("\n=== 处理建议 ===")
        print("1. 使用process_bag.sh脚本处理:")
        print(f"   ./scripts/process_bag.sh {bag_file} ndt")
        print(f"   ./scripts/process_bag.sh {bag_file} gn_icp")
        print("\n2. 手动启动ROS节点:")
        print("   roslaunch lidar_odometry lidar_odometry.launch")
        print("   rosbag play your_bag.bag")
        
    else:
        print("无法处理此bag包文件")

if __name__ == "__main__":
    main()
