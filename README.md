# 激光雷达里程计项目

基于KITTI数据集的激光雷达里程计实现，支持NDT和GN-ICP配准算法。

## 项目结构

```
mapping/
├── include/lidar_odometry/          # 头文件
│   ├── ndt_registration.h          # NDT配准算法
│   ├── gn_icp_registration.h       # GN-ICP配准算法
│   ├── lidar_odometry.h            # 激光雷达里程计
│   ├── imu_integration.h           # IMU积分
│   ├── ros_odometry_node.h         # ROS节点
│   └── trajectory_evaluator.h      # 轨迹评估器
├── src/                            # 源文件
│   ├── ndt_registration.cpp
│   ├── gn_icp_registration.cpp
│   ├── lidar_odometry.cpp
│   ├── imu_integration.cpp
│   ├── ros_odometry_node.cpp
│   ├── trajectory_evaluator.cpp
│   ├── ndt_odometry.cpp           # NDT里程计主程序
│   ├── icp_odometry.cpp           # ICP里程计主程序
│   └── trajectory_evaluator.cpp   # 轨迹评估主程序
├── launch/                         # 启动文件
│   └── lidar_odometry.launch
├── config/                         # 配置文件
│   └── lidar_odometry.rviz
├── CMakeLists.txt                  # CMake配置
├── package.xml                     # ROS包配置
└── README.md                       # 说明文档
```

## 功能特性

### 1. 配准算法
- **NDT配准**: 基于PCL库的NDT (Normal Distributions Transform) 配准
- **GN-ICP配准**: 手写实现的Gauss-Newton ICP配准算法

### 2. 里程计功能
- 基于KITTI数据集的激光雷达里程计
- 支持多种配准方法
- 实时轨迹生成和可视化
- 轨迹保存和加载

### 3. ROS集成
- ROS节点支持
- 话题发布和订阅
- TF变换发布
- RViz可视化

### 4. IMU融合
- IMU数据积分
- 位姿预测
- 多传感器融合

### 5. 轨迹评估
- ATE (Absolute Trajectory Error) 计算
- RPE (Relative Pose Error) 计算
- 与evo工具兼容的轨迹格式

## 编译安装

### 依赖项
- ROS Melodic/Noetic
- PCL 1.8+
- Eigen3
- OpenCV (可选，用于可视化)

### 编译步骤
```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 复制项目到工作空间
cp -r /path/to/mapping ~/catkin_ws/src/lidar_odometry

# 编译
cd ~/catkin_ws
catkin_make

# 设置环境
source devel/setup.bash
```

## 使用方法

### 1. 处理KITTI数据集

#### NDT里程计
```bash
# 处理KITTI序列0
rosrun lidar_odometry ndt_odometry /path/to/kitti/dataset 0
```

#### GN-ICP里程计
```bash
# 处理KITTI序列0
rosrun lidar_odometry icp_odometry /path/to/kitti/dataset 0
```

### 2. ROS节点运行

#### 启动里程计节点
```bash
# 使用NDT配准
roslaunch lidar_odometry lidar_odometry.launch registration_method:=ndt

# 使用GN-ICP配准
roslaunch lidar_odometry lidar_odometry.launch registration_method:=gn_icp
```

#### 参数配置
```bash
roslaunch lidar_odometry lidar_odometry.launch \
    registration_method:=ndt \
    voxel_size:=0.1 \
    max_range:=100.0 \
    min_range:=1.0 \
    use_imu_prediction:=true
```

### 3. 轨迹评估

#### 评估轨迹精度
```bash
# 评估NDT轨迹
rosrun lidar_odometry trajectory_evaluator \
    ndt_trajectory_0.txt \
    /path/to/kitti/ground_truth/00.txt \
    ndt_evaluation_result.txt

# 评估GN-ICP轨迹
rosrun lidar_odometry trajectory_evaluator \
    gn_icp_trajectory_0.txt \
    /path/to/kitti/ground_truth/00.txt \
    gn_icp_evaluation_result.txt
```

## 参数说明

### 配准参数
- `resolution`: NDT网格分辨率 (默认: 1.0)
- `step_size`: 优化步长 (默认: 0.1)
- `max_iterations`: 最大迭代次数 (默认: 35)
- `transformation_epsilon`: 变换收敛阈值 (默认: 0.01)
- `euclidean_fitness_epsilon`: 欧几里得适应度阈值 (默认: 0.01)

### 里程计参数
- `voxel_size`: 体素大小 (默认: 0.1)
- `max_range`: 最大距离 (默认: 100.0)
- `min_range`: 最小距离 (默认: 1.0)

### ROS参数
- `pointcloud_topic`: 点云话题 (默认: /velodyne_points)
- `imu_topic`: IMU话题 (默认: /imu)
- `odom_frame`: 里程计坐标系 (默认: odom)
- `lidar_frame`: 激光雷达坐标系 (默认: velodyne)

## 输出格式

### 轨迹文件格式 (TUM格式)
```
# timestamp tx ty tz qx qy qz qw
0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000
0.100000 0.100000 0.000000 0.000000 0.000000 0.000000 0.000000 1.000000
...
```

### 评估结果格式
```
# Trajectory Evaluation Results
# ATE (Absolute Trajectory Error)
ATE_RMSE: 0.123456
ATE_MEAN: 0.098765
ATE_STD: 0.045678
ATE_MAX: 0.234567

# RPE (Relative Pose Error)
RPE_TRANS_RMSE: 0.012345
RPE_ROT_RMSE: 0.001234
RPE_TRANS_MEAN: 0.009876
RPE_ROT_MEAN: 0.000987

# Trajectory Statistics
TRAJECTORY_LENGTH: 1234.567890
AVERAGE_SPEED: 12.345678
```

## 实验要求

### 1. 配准算法实现
- ✅ 调用PCL库完成NDT配准
- ✅ 手写GN-ICP实现配准

### 2. 里程计实现
- ✅ 基于KITTI数据集的前端里程计
- ✅ NDT里程计代码编写
- ✅ GN-ICP里程计代码编写

### 3. ROS节点
- ✅ 里程计轨迹保存
- ✅ 与evo评估工具兼容

### 4. 实时数据
- ✅ 小车录制lidar数据bag包支持
- ✅ 实时IMU积分预测

### 5. 代码规范
- ✅ C++编写
- ✅ Git源代码管理
- ✅ 符合C++命名规范
- ✅ 面向对象设计
- ✅ 头文件规范
- ✅ ROS项目规范
- ✅ CMakeLists.txt配置

## 注意事项

1. **数据路径**: 确保KITTI数据集路径正确
2. **内存使用**: 大点云数据可能占用较多内存
3. **参数调优**: 根据具体场景调整配准参数
4. **可视化**: 使用RViz查看实时结果
5. **评估**: 使用evo工具进行更详细的轨迹评估

## 故障排除

### 常见问题
1. **编译错误**: 检查依赖项安装
2. **运行时错误**: 检查数据路径和参数
3. **可视化问题**: 检查RViz配置
4. **性能问题**: 调整体素大小和距离范围

### 调试技巧
- 使用`rosrun`单独运行节点
- 检查话题发布和订阅
- 使用`rostopic echo`查看数据
- 使用`rviz`可视化结果

## 扩展功能

- [ ] 回环检测
- [ ] 全局优化
- [ ] 多传感器融合
- [ ] 实时建图
- [ ] 语义分割集成

## 许可证

MIT License

## 贡献

欢迎提交Issue和Pull Request！
