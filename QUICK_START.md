# 快速开始指南

## 项目结构
```
mapping/
├── data.bag                    # bag包 (15GB)
├── scripts/                    # 处理脚本
├── src/                        # 源代码
├── include/                     # 头文件
├── launch/                     # ROS启动文件
├── config/                     # 配置文件
└── README.md                 # 详细说明
```

## 快速处理

### 1. 使用NDT配准
```bash
./scripts/process_bag.sh ./data.bag ndt
```

### 2. 使用GN-ICP配准
```bash
./scripts/process_bag.sh ./data.bag gn_icp
```

## 手动处理

### 启动里程计
```bash
source devel/setup.bash
roslaunch lidar_odometry lidar_odometry.launch
```

### 播放bag包
```bash
rosbag play ./data.bag --clock --rate=1.0
```

## 输出结果
- `ndt_trajectory.txt` - NDT轨迹
- `gn_icp_trajectory.txt` - GN-ICP轨迹
- `bag_processing_results/` - 处理结果目录

## 可视化
```bash
rviz -d config/lidar_odometry.rviz
```
