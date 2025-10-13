#!/usr/bin/env python3
"""
快速bag包测试工具
使用ROS 2直接处理bag包
"""

import os
import sys
import subprocess
import time

def test_ros2_environment():
    """测试ROS 2环境"""
    print("=== 测试ROS 2环境 ===")
    
    try:
        result = subprocess.run(['ros2', '--help'], capture_output=True, text=True)
        if result.returncode == 0:
            print(" ROS 2环境正常")
            return True
        else:
            print(" ROS 2环境异常")
            return False
    except FileNotFoundError:
        print(" 找不到ros2命令")
        return False

def test_bag_file():
    """测试bag包文件"""
    print("\n=== 测试bag包文件 ===")
    
    bag_file = "./data.bag"
    if not os.path.exists(bag_file):
        print(f" bag包文件不存在: {bag_file}")
        return False
    
    file_size = os.path.getsize(bag_file)
    print(f" bag包文件存在: {bag_file}")
    print(f" 文件大小: {file_size / (1024*1024*1024):.2f} GB")
    
    return True

def test_ros2_bag_tools():
    """测试ROS 2 bag工具"""
    print("\n=== 测试ROS 2 bag工具 ===")
    
    try:
        result = subprocess.run(['ros2', 'bag', '--help'], capture_output=True, text=True)
        if result.returncode == 0:
            print(" ros2 bag工具可用")
            return True
        else:
            print(" ros2 bag工具不可用")
            return False
    except FileNotFoundError:
        print(" 找不到ros2 bag命令")
        return False

def analyze_bag_info():
    """分析bag包信息"""
    print("\n=== 分析bag包信息 ===")
    
    bag_file = "./data.bag"
    
    try:
        # 使用ros2 bag info命令
        result = subprocess.run(['ros2', 'bag', 'info', bag_file], 
                              capture_output=True, text=True, timeout=30)
        
        if result.returncode == 0:
            print(" bag包信息获取成功:")
            print(result.stdout)
            return True
        else:
            print(" 无法获取bag包信息:")
            print(result.stderr)
            return False
            
    except subprocess.TimeoutExpired:
        print(" bag包信息获取超时（文件可能很大）")
        return False
    except Exception as e:
        print(f" 分析bag包时出错: {e}")
        return False

def test_simple_processing():
    """测试简单的bag包处理"""
    print("\n=== 测试简单处理 ===")
    
    bag_file = "./data.bag"
    output_dir = "quick_test_results"
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    print(f" 输出目录: {output_dir}")
    
    # 尝试播放bag包（短时间）
    print(" 尝试播放bag包（5秒）...")
    
    try:
        # 启动ros2 bag play，但只播放5秒
        process = subprocess.Popen(['ros2', 'bag', 'play', bag_file, '--duration', '5'],
                                 stdout=subprocess.PIPE, 
                                 stderr=subprocess.PIPE,
                                 text=True)
        
        # 等待5秒
        time.sleep(5)
        
        # 终止进程
        process.terminate()
        process.wait(timeout=5)
        
        print(" bag包播放测试完成")
        return True
        
    except Exception as e:
        print(f" bag包播放测试失败: {e}")
        return False

def main():
    """主函数"""
    print(" 快速bag包测试工具")
    print("=" * 50)
    
    # 测试环境
    if not test_ros2_environment():
        print("\n ROS 2环境测试失败，请检查安装")
        return False
    
    if not test_bag_file():
        print("\n bag包文件测试失败")
        return False
    
    if not test_ros2_bag_tools():
        print("\n ROS 2 bag工具测试失败")
        return False
    
    # 分析bag包
    analyze_bag_info()
    
    # 简单处理测试
    test_simple_processing()
    
    print("\n" + "=" * 50)
    print(" 测试完成！")
    print("\n下一步建议:")
    print("1. 查看bag包信息，了解包含的话题")
    print("2. 根据话题类型选择合适的处理节点")
    print("3. 使用ros2 launch启动处理节点")
    print("4. 使用ros2 bag play播放bag包")
    
    return True

if __name__ == "__main__":
    main()
