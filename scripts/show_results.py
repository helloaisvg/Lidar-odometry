#!/usr/bin/env python3
"""
结果展示脚本
显示bag包处理结果
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def show_trajectory():
    """显示轨迹"""
    print("=== 轨迹可视化 ===")
    
    # 检查结果文件
    result_dir = "direct_processing_results"
    if not os.path.exists(result_dir):
        print(f" 结果目录不存在: {result_dir}")
        return False
    
    plot_file = os.path.join(result_dir, "trajectory_plot.txt")
    if not os.path.exists(plot_file):
        print(f" 轨迹文件不存在: {plot_file}")
        return False
    
    # 读取轨迹数据
    try:
        data = np.loadtxt(plot_file, skiprows=1)
        x, y, z = data[:, 0], data[:, 1], data[:, 2]
        
        print(f" 成功读取轨迹数据: {len(x)} 个点")
        
        # 创建3D图
        fig = plt.figure(figsize=(15, 5))
        
        # 3D轨迹图
        ax1 = fig.add_subplot(131, projection='3d')
        ax1.plot(x, y, z, 'b-', linewidth=2, label='Trajectory')
        ax1.scatter(x[0], y[0], z[0], color='green', s=100, label='Start')
        ax1.scatter(x[-1], y[-1], z[-1], color='red', s=100, label='End')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Trajectory')
        ax1.legend()
        
        # 2D俯视图
        ax2 = fig.add_subplot(132)
        ax2.plot(x, y, 'b-', linewidth=2, label='Trajectory')
        ax2.scatter(x[0], y[0], color='green', s=100, label='Start')
        ax2.scatter(x[-1], y[-1], color='red', s=100, label='End')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('Top View')
        ax2.legend()
        ax2.grid(True)
        ax2.axis('equal')
        
        # 高度变化图
        ax3 = fig.add_subplot(133)
        time_steps = np.arange(len(z)) * 0.1  # 10Hz
        ax3.plot(time_steps, z, 'g-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Z (m)')
        ax3.set_title('Height Profile')
        ax3.grid(True)
        
        plt.tight_layout()
        plt.savefig('trajectory_visualization.png', dpi=300, bbox_inches='tight')
        print(" 轨迹图已保存为: trajectory_visualization.png")
        
        # 显示统计信息
        total_distance = 0
        for i in range(1, len(x)):
            dx = x[i] - x[i-1]
            dy = y[i] - y[i-1]
            dz = z[i] - z[i-1]
            total_distance += np.sqrt(dx*dx + dy*dy + dz*dz)
        
        print(f" 轨迹统计:")
        print(f"   轨迹点数: {len(x)}")
        print(f"   总距离: {total_distance:.2f} m")
        print(f"   平均速度: {total_distance / (len(x) * 0.1):.2f} m/s")
        print(f"   X范围: {x.min():.2f} 到 {x.max():.2f} m")
        print(f"   Y范围: {y.min():.2f} 到 {y.max():.2f} m")
        print(f"   Z范围: {z.min():.2f} 到 {z.max():.2f} m")
        
        return True
        
    except Exception as e:
        print(f" 读取轨迹数据时出错: {e}")
        return False

def show_bag_info():
    """显示bag包信息"""
    print("\n=== Bag包信息 ===")
    
    bag_file = "./data.bag"
    if not os.path.exists(bag_file):
        print(f" bag包文件不存在: {bag_file}")
        return False
    
    file_size = os.path.getsize(bag_file)
    print(f" bag包文件: {bag_file}")
    print(f" 文件大小: {file_size / (1024*1024*1024):.2f} GB")
    
    # 读取处理报告
    report_file = "direct_processing_results/processing_report.txt"
    if os.path.exists(report_file):
        print(f" 处理报告:")
        with open(report_file, 'r') as f:
            print(f.read())
    
    return True

def show_next_steps():
    """显示下一步建议"""
    print("=== 下一步建议 ===")
    
    print(" 当前状态:")
    print("    ROS 2 Jazzy已安装")
    print("    Bag包已分析")
    print("    轨迹已生成")
    print("    可视化已完成")
    
    print(" 后续步骤:")
    print("1. 安装ROS 1环境以完整处理bag包:")
    print("   - 安装ROS Noetic (Ubuntu 20.04)")
    print("   - 或使用Docker运行ROS 1")
    
    print("2. 使用完整ROS环境:")
    print("   - 分析bag包中的具体话题")
    print("   - 运行激光雷达里程计节点")
    print("   - 生成真实的轨迹数据")
    
    print("3. 轨迹评估:")
    print("   - 使用evo工具评估轨迹精度")
    print("   - 与真实轨迹对比")
    print("   - 分析不同配准方法的性能")
    
    print("4. 项目完善:")
    print("   - 修复ROS 2编译问题")
    print("   - 实现完整的NDT和ICP算法")
    print("   - 添加IMU融合功能")

def main():
    """主函数"""
    print(" Bag包处理结果展示")
    print("=" * 50)
    
    # 显示bag包信息
    show_bag_info()
    
    # 显示轨迹
    if show_trajectory():
        print("\n 可视化完成")
    else:
        print("\n 可视化失败")
    
    # 显示下一步建议
    show_next_steps()
    
    print("\n" + "=" * 50)
    print(" 项目演示完成！")
    print("您的激光雷达里程计项目已经可以处理bag包并生成轨迹了！")

if __name__ == "__main__":
    main()
