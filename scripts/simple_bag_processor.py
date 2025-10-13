#!/usr/bin/env python3
"""
简化的bag包处理器
不依赖ROS环境，直接处理bag包数据
"""

import sys
import os
import struct
import time
import numpy as np
from pathlib import Path

class SimpleBagProcessor:
    def __init__(self, bag_file):
        self.bag_file = bag_file
        self.output_dir = "simple_processing_results"
        self.trajectory_points = []
        
    def process_bag(self):
        """处理bag包"""
        print(f"开始处理bag包: {self.bag_file}")
        
        if not os.path.exists(self.bag_file):
            print(f"错误: 文件不存在 {self.bag_file}")
            return False
            
        # 创建输出目录
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 分析bag包
        self.analyze_bag()
        
        # 模拟轨迹生成
        self.generate_simulated_trajectory()
        
        # 保存结果
        self.save_results()
        
        return True
        
    def analyze_bag(self):
        """分析bag包"""
        print("\n=== Bag包分析 ===")
        
        file_size = os.path.getsize(self.bag_file)
        print(f"文件大小: {file_size / (1024*1024*1024):.2f} GB")
        
        try:
            with open(self.bag_file, 'rb') as f:
                header = f.read(13)
                if header.startswith(b'#ROSBAG'):
                    print(" 有效的ROS bag包")
                    
                    # 读取更多信息
                    f.seek(0)
                    first_chunk = f.read(1024)
                    print(f"Bag包版本: {header.decode('utf-8')}")
                    
                else:
                    print(" 不是有效的ROS bag包")
                    return False
                    
        except Exception as e:
            print(f" 读取文件时出错: {e}")
            return False
            
        return True
        
    def generate_simulated_trajectory(self):
        """生成模拟轨迹（用于演示）"""
        print("\n=== 生成模拟轨迹 ===")
        
        # 模拟一个简单的圆形轨迹
        num_points = 100
        radius = 10.0
        
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = 0.1 * np.sin(4 * angle)  # 轻微的高度变化
            
            timestamp = i * 0.1  # 10Hz
            
            self.trajectory_points.append({
                'timestamp': timestamp,
                'x': x,
                'y': y,
                'z': z,
                'qx': 0.0,
                'qy': 0.0,
                'qz': 0.0,
                'qw': 1.0
            })
            
        print(f"生成了 {len(self.trajectory_points)} 个轨迹点")
        
    def save_results(self):
        """保存处理结果"""
        print("\n=== 保存结果 ===")
        
        # 保存轨迹文件
        trajectory_file = os.path.join(self.output_dir, "simulated_trajectory.txt")
        with open(trajectory_file, 'w') as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            for point in self.trajectory_points:
                f.write(f"{point['timestamp']:.6f} "
                       f"{point['x']:.6f} {point['y']:.6f} {point['z']:.6f} "
                       f"{point['qx']:.6f} {point['qy']:.6f} {point['qz']:.6f} {point['qw']:.6f}\n")
                       
        print(f"轨迹已保存到: {trajectory_file}")
        
        # 生成处理报告
        report_file = os.path.join(self.output_dir, "processing_report.txt")
        with open(report_file, 'w') as f:
            f.write("Bag包处理报告\n")
            f.write("==============\n\n")
            f.write(f"处理时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Bag包文件: {self.bag_file}\n")
            f.write(f"文件大小: {os.path.getsize(self.bag_file) / (1024*1024*1024):.2f} GB\n")
            f.write(f"轨迹点数: {len(self.trajectory_points)}\n")
            f.write(f"输出目录: {self.output_dir}\n\n")
            f.write("注意: 这是模拟轨迹，实际处理需要ROS环境\n")
            
        print(f"报告已保存到: {report_file}")
        
        # 生成可视化数据
        self.generate_visualization_data()
        
    def generate_visualization_data(self):
        """生成可视化数据"""
        print("生成可视化数据...")
        
        # 生成简单的轨迹图数据
        plot_data = os.path.join(self.output_dir, "trajectory_plot.txt")
        with open(plot_data, 'w') as f:
            f.write("# x y z\n")
            for point in self.trajectory_points:
                f.write(f"{point['x']:.6f} {point['y']:.6f} {point['z']:.6f}\n")
                
        print(f"可视化数据已保存到: {plot_data}")
        
        # 生成简单的Python绘图脚本
        plot_script = os.path.join(self.output_dir, "plot_trajectory.py")
        with open(plot_script, 'w') as f:
            f.write('''#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

# 读取轨迹数据
data = np.loadtxt('trajectory_plot.txt', skiprows=1)
x, y, z = data[:, 0], data[:, 1], data[:, 2]

# 创建3D图
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 绘制轨迹
ax.plot(x, y, z, 'b-', linewidth=2, label='Trajectory')
ax.scatter(x[0], y[0], z[0], color='green', s=100, label='Start')
ax.scatter(x[-1], y[-1], z[-1], color='red', s=100, label='End')

# 设置标签
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Lidar Odometry Trajectory')
ax.legend()

# 保存图片
plt.savefig('trajectory_3d.png', dpi=300, bbox_inches='tight')
plt.show()

print("轨迹图已保存为 trajectory_3d.png")
''')
            
        print(f"绘图脚本已保存到: {plot_script}")

def main():
    if len(sys.argv) < 2:
        print("用法: python3 simple_bag_processor.py <bag_file>")
        print("示例: python3 simple_bag_processor.py ./data.bag")
        sys.exit(1)
        
    bag_file = sys.argv[1]
    
    print("=== 简化Bag包处理器 ===")
    print("注意: 这是演示版本，生成模拟轨迹")
    print("完整功能需要ROS环境")
    
    processor = SimpleBagProcessor(bag_file)
    
    if processor.process_bag():
        print(" 处理完成!")
        print(f"结果保存在: {processor.output_dir}/")
        print("下一步:")
        print("1. 安装ROS环境以获得完整功能")
        print("2. 查看生成的结果文件")
        print("3. 运行绘图脚本可视化轨迹")
    else:
        print(" 处理失败")

if __name__ == "__main__":
    main()
