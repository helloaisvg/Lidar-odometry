#!/bin/bash

# 激光雷达里程计测试脚本

echo "=== 激光雷达里程计测试脚本 ==="

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: 未找到ROS环境，请先source ROS setup文件"
    exit 1
fi

echo "ROS版本: $ROS_DISTRO"

# 检查工作空间
if [ ! -f "devel/setup.bash" ]; then
    echo "错误: 未找到catkin工作空间，请先编译项目"
    exit 1
fi

# 设置环境
source devel/setup.bash

echo "环境设置完成"

# 测试编译
echo "测试编译..."
catkin_make

if [ $? -eq 0 ]; then
    echo "编译成功"
else
    echo "编译失败"
    exit 1
fi

# 测试节点
echo "测试节点..."

# 测试NDT里程计
echo "测试NDT里程计节点..."
timeout 5s rosrun lidar_odometry ndt_odometry --help 2>/dev/null
if [ $? -eq 0 ]; then
    echo "NDT里程计节点正常"
else
    echo "NDT里程计节点测试失败"
fi

# 测试GN-ICP里程计
echo "测试GN-ICP里程计节点..."
timeout 5s rosrun lidar_odometry icp_odometry --help 2>/dev/null
if [ $? -eq 0 ]; then
    echo "GN-ICP里程计节点正常"
else
    echo "GN-ICP里程计节点测试失败"
fi

# 测试轨迹评估器
echo "测试轨迹评估器..."
timeout 5s rosrun lidar_odometry trajectory_evaluator --help 2>/dev/null
if [ $? -eq 0 ]; then
    echo "轨迹评估器正常"
else
    echo "轨迹评估器测试失败"
fi

# 测试ROS节点
echo "测试ROS里程计节点..."
timeout 5s rosrun lidar_odometry ros_odometry_node --help 2>/dev/null
if [ $? -eq 0 ]; then
    echo "ROS里程计节点正常"
else
    echo "ROS里程计节点测试失败"
fi

echo "=== 测试完成 ==="
