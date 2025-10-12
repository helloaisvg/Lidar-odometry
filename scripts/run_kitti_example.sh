#!/bin/bash

# KITTI数据集处理示例脚本

echo "=== KITTI数据集处理示例 ==="

# 检查参数
if [ $# -lt 2 ]; then
    echo "用法: $0 <kitti_data_path> <sequence_number>"
    echo "示例: $0 /path/to/kitti/dataset 0"
    exit 1
fi

KITTI_PATH=$1
SEQUENCE=$2

echo "KITTI数据路径: $KITTI_PATH"
echo "序列号: $SEQUENCE"

# 检查路径是否存在
if [ ! -d "$KITTI_PATH" ]; then
    echo "错误: KITTI数据路径不存在: $KITTI_PATH"
    exit 1
fi

# 设置ROS环境
source devel/setup.bash

echo "开始处理KITTI序列 $SEQUENCE..."

# 运行NDT里程计
echo "运行NDT里程计..."
rosrun lidar_odometry ndt_odometry "$KITTI_PATH" "$SEQUENCE" &
NDT_PID=$!

# 等待NDT完成
wait $NDT_PID
NDT_RESULT=$?

if [ $NDT_RESULT -eq 0 ]; then
    echo "NDT里程计处理完成"
else
    echo "NDT里程计处理失败"
fi

# 运行GN-ICP里程计
echo "运行GN-ICP里程计..."
rosrun lidar_odometry icp_odometry "$KITTI_PATH" "$SEQUENCE" &
ICP_PID=$!

# 等待GN-ICP完成
wait $ICP_PID
ICP_RESULT=$?

if [ $ICP_RESULT -eq 0 ]; then
    echo "GN-ICP里程计处理完成"
else
    echo "GN-ICP里程计处理失败"
fi

# 检查输出文件
echo "检查输出文件..."
if [ -f "ndt_trajectory_${SEQUENCE}.txt" ]; then
    echo "NDT轨迹文件已生成: ndt_trajectory_${SEQUENCE}.txt"
    echo "轨迹点数: $(wc -l < ndt_trajectory_${SEQUENCE}.txt)"
else
    echo "警告: NDT轨迹文件未生成"
fi

if [ -f "gn_icp_trajectory_${SEQUENCE}.txt" ]; then
    echo "GN-ICP轨迹文件已生成: gn_icp_trajectory_${SEQUENCE}.txt"
    echo "轨迹点数: $(wc -l < gn_icp_trajectory_${SEQUENCE}.txt)"
else
    echo "警告: GN-ICP轨迹文件未生成"
fi

echo "=== 处理完成 ==="
