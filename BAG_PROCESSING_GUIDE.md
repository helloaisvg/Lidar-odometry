# Bag包处理指南

## 您的Bag包信息
- **文件路径**: `./data.bag`
- **文件大小**: 15 GB
- **格式**: ROS bag包



##  处理方案

### 方案1：自动化处理

#### 使用NDT配准
```bash
./scripts/process_bag.sh ./data.bag ndt
```

#### 使用GN-ICP配准
```bash
./scripts/process_bag.sh ./data.bag gn_icp
```

### 方案2：手动处理

#### 步骤1：启动里程计节点
```bash
# 终端1：启动里程计节点
source devel/setup.bash
roslaunch lidar_odometry lidar_odometry.launch registration_method:=ndt
```

#### 步骤2：播放bag包
```bash
# 终端2：播放bag包
rosbag play ./data.bag --clock --rate=1.0
```

#### 步骤3：可视化（可选）
```bash
# 终端3：启动RViz可视化
rviz -d config/lidar_odometry.rviz
```

##  输出

### 轨迹文件
- `ndt_trajectory.txt` - NDT配准轨迹
- `gn_icp_trajectory.txt` - GN-ICP配准轨迹

### 可视化数据
- `/lidar_pose` - 位姿话题
- `/lidar_odometry` - 里程计话题
- `/lidar_path` - 路径话题
- `/transformed_cloud` - 变换后点云

##  参数调整

### 配准参数
```bash
# 在launch文件中调整参数
roslaunch lidar_odometry lidar_odometry.launch \
    registration_method:=ndt \
    voxel_size:=0.1 \
    max_range:=100.0 \
    min_range:=1.0
```

### 话题映射
```bash
# 如果bag包中的话题名称不同，可以重映射
rosbag play ./data.bag \
    /velodyne_points:=/velodyne_points \
    /imu:=/imu
```

##  结果分析

### 轨迹评估
```bash
# 如果有真值轨迹，可以评估精度
rosrun lidar_odometry trajectory_evaluator \
    ndt_trajectory.txt \
    ground_truth.txt \
    evaluation_result.txt
```

### 可视化轨迹
```bash
# 使用evo工具可视化（如已安装）
evo_traj tum ndt_trajectory.txt --plot
```


