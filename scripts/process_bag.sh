#!/bin/bash

# Bag包处理脚本
# 用于处理激光雷达bag包并运行里程计

echo "=== Bag包激光雷达里程计处理脚本 ==="

# 检查参数
if [ $# -lt 1 ]; then
    echo "用法: $0 <bag_file_path> [registration_method]"
    echo "示例: $0 /home/sss/data.bag ndt"
    echo "示例: $0 /home/sss/data.bag gn_icp"
    exit 1
fi

BAG_FILE=$1
REGISTRATION_METHOD=${2:-ndt}

echo "Bag包文件: $BAG_FILE"
echo "配准方法: $REGISTRATION_METHOD"

# 检查bag包是否存在
if [ ! -f "$BAG_FILE" ]; then
    echo "错误: Bag包文件不存在: $BAG_FILE"
    exit 1
fi

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "警告: 未检测到ROS环境，尝试设置ROS环境..."
    source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null || {
        echo "错误: 无法找到ROS环境，请先安装ROS"
        exit 1
    }
fi

echo "ROS版本: $ROS_DISTRO"

# 设置工作空间环境
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
    echo "工作空间环境已设置"
else
    echo "警告: 未找到工作空间，尝试编译..."
    catkin_make
    if [ -f "devel/setup.bash" ]; then
        source devel/setup.bash
        echo "工作空间编译完成"
    else
        echo "错误: 无法编译工作空间"
        exit 1
    fi
fi

# 创建输出目录
OUTPUT_DIR="bag_processing_results"
mkdir -p $OUTPUT_DIR

echo "输出目录: $OUTPUT_DIR"
echo "Bag包文件: $(basename $BAG_FILE)"

# 启动里程计节点
echo "启动激光雷达里程计节点..."
roslaunch lidar_odometry lidar_odometry.launch \
    registration_method:=$REGISTRATION_METHOD \
    pointcloud_topic:=/velodyne_points \
    imu_topic:=/imu \
    use_imu_prediction:=true \
    publish_transformed_cloud:=true &

ODOMETRY_PID=$!

# 等待节点启动
sleep 5

# 播放bag包
echo "开始播放bag包: $BAG_FILE"
rosbag play $BAG_FILE --clock --rate=1.0

# 等待处理完成
echo "等待里程计处理完成..."
wait $ODOMETRY_PID

# 保存轨迹
echo "保存轨迹..."
if [ -f "trajectory.txt" ]; then
    cp trajectory.txt $OUTPUT_DIR/${REGISTRATION_METHOD}_trajectory.txt
    echo "轨迹已保存到: $OUTPUT_DIR/${REGISTRATION_METHOD}_trajectory.txt"
else
    echo "警告: 未找到轨迹文件"
fi

# 生成处理报告
echo "生成处理报告..."
cat > $OUTPUT_DIR/processing_report.txt << EOF
Bag包处理报告
==============

处理时间: $(date)
Bag包文件: $BAG_FILE
配准方法: $REGISTRATION_METHOD
输出目录: $OUTPUT_DIR

文件列表:
$(ls -la $OUTPUT_DIR/)

处理完成！
EOF

echo "=== 处理完成 ==="
echo "结果保存在: $OUTPUT_DIR/"
echo "查看报告: cat $OUTPUT_DIR/processing_report.txt"
