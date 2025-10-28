# 激光雷达里程计项目文件整理

##  项目文件分类汇总

###  核心源代码文件 (src/)

| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `ndt_registration.cpp` | 5.8KB | **NDT配准算法实现** - 使用PCL库实现正态分布变换配准 |
| `gn_icp_registration.cpp` | 12.9KB | **GN-ICP配准算法实现** - 手写高斯牛顿ICP配准算法 |
| `lidar_odometry.cpp` | 13.3KB | **里程计核心实现** - 激光雷达里程计主类 |
| `lidar_odometry_enhanced.cpp` | 6.5KB | **增强功能实现** - 视频录制、地图保存、截图功能 |
| `kitti_ndt_odometry.cpp` | 4.8KB | **KITTI NDT里程计** - 基于KITTI数据集的NDT里程计 |
| `kitti_icp_odometry.cpp` | 4.8KB | **KITTI ICP里程计** - 基于KITTI数据集的ICP里程计 |
| `kitti_enhanced_odometry.cpp` | 12.6KB | **KITTI增强里程计** - 包含视频录制和截图的完整里程计 |
| `ros_odometry_node.cpp` | 11.3KB | **ROS节点实现** - ROS里程计节点，支持轨迹保存 |
| `trajectory_evaluator.cpp` | 1.7KB | **轨迹评估器** - C++轨迹评估实现 |
| `imu_integration.cpp` | 7.0KB | **IMU融合** - 惯性测量单元数据融合 |
| `ndt_odometry.cpp` | 2.1KB | **NDT里程计** - 独立的NDT里程计实现 |
| `icp_odometry.cpp` | 2.1KB | **ICP里程计** - 独立的ICP里程计实现 |

###  头文件 (include/lidar_odometry/)

| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `ndt_registration.h` | 4.0KB | **NDT配准头文件** - NDT算法类声明 |
| `gn_icp_registration.h` | 6.9KB | **GN-ICP配准头文件** - GN-ICP算法类声明 |
| `lidar_odometry.h` | 7.0KB | **里程计头文件** - 里程计主类声明 |
| `ros_odometry_node.h` | 5.4KB | **ROS节点头文件** - ROS节点类声明 |
| `trajectory_evaluator.h` | 6.8KB | **轨迹评估头文件** - 轨迹评估类声明 |
| `imu_integration.h` | 4.6KB | **IMU融合头文件** - IMU融合类声明 |
| `so3_utils.h` | 2.3KB | **SO3工具头文件** - 三维旋转群工具函数 |

###  脚本文件 (scripts/) 

| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `direct_bag_processor.py` | 10.0KB | **Bag包处理器** - 直接处理ROS bag包生成轨迹 |
| `show_results.py` | 5.3KB | **结果展示脚本** - 显示处理结果和统计信息 |
| `simple_trajectory_evaluator.py` | 6.5KB | **轨迹评估脚本** - Python轨迹评估工具 |
| `view_pointcloud.py` | 4.2KB | **点云查看器** - 可视化PCD点云文件 |
| `evaluate_with_evo.sh` | 1.6KB | **EVO评估脚本** - 使用EVO工具评估轨迹 |



###  结果文件 (results/) 

#### 点云文件 (results/pointclouds/)
| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `ndt_global_map_0.pcd` | 741KB | **NDT全局地图PCD** - NDT算法生成的全局地图点云 |
| `ndt_global_map_0.ply` | 741KB | **NDT全局地图PLY** - 转换为PLY格式的NDT地图 |
| `gn_icp_global_map_0.pcd` | 741KB | **GN-ICP全局地图PCD** - GN-ICP算法生成的全局地图点云 |
| `gn_icp_global_map_0.ply` | 741KB | **GN-ICP全局地图PLY** - 转换为PLY格式的GN-ICP地图 |

#### 截图文件 (results/screenshots/)
| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `ndt_global_map_screenshot_0.png` | 20KB | **NDT全局地图截图** - NDT算法生成的全局地图 |
| `gn_icp_global_map_screenshot_0.png` | 20KB | **GN-ICP全局地图截图** - GN-ICP算法生成的全局地图 |

#### 视频文件 (results/videos/)
| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `ndt_odometry_video_0.mp4` | 3.1MB | **NDT里程计运行视频** - NDT算法运行过程录制 |
| `gn_icp_odometry_video_0.mp4` | 3.1MB | **GN-ICP里程计运行视频** - GN-ICP算法运行过程录制 |

#### 轨迹文件 (results/trajectories/)
| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `ndt_trajectory_0.txt` | 15KB | **NDT轨迹文件** - NDT算法生成的轨迹(TUM格式) |
| `gn_icp_trajectory_0.txt` | 15KB | **GN-ICP轨迹文件** - GN-ICP算法生成的轨迹(TUM格式) |
| `ndt_trajectory_kitti_0.txt` | 774B | **KITTI NDT轨迹** - 基于KITTI数据集的NDT轨迹 |

#### 可视化文件 (results/visualizations/)
| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `ndt_pointcloud_visualization.png` | 3.5MB | **NDT点云可视化** - NDT点云的3D可视化图 |
| `trajectory_visualization.png` | 596KB | **轨迹可视化** - 综合轨迹可视化图 |

###  EVO评估结果 (evo_evaluation_results/)

#### NDT评估结果
| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `ndt/real_evo_ndt_trajectory_trajectories.png` | 70KB | **NDT轨迹对比图** - EVO生成的轨迹对比分析 |
| `ndt/real_evo_ndt_trajectory_xyz.png` | 57KB | **NDT XYZ坐标图** - EVO生成的XYZ坐标变化图 |
| `ndt/real_evo_ndt_trajectory_rpy.png` | 38KB | **NDT姿态角图** - EVO生成的姿态角变化图 |
| `ndt/real_evo_ndt_trajectory_speeds.png` | 104KB | **NDT速度分析图** - EVO生成的速度分析图 |
| `ndt/ndt_trajectory_0_2d.png` | 124KB | **NDT 2D轨迹图** - Python生成的2D轨迹图 |
| `ndt/ndt_trajectory_0_3d.png` | 457KB | **NDT 3D轨迹图** - Python生成的3D轨迹图 |
| `ndt/ndt_trajectory_0_speed.png` | 264KB | **NDT速度曲线** - Python生成的速度曲线 |
| `ndt/ndt_trajectory_0_stats.txt` | 230B | **NDT统计信息** - NDT轨迹统计信息 |

#### GN-ICP评估结果
| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `gn_icp/real_evo_gn_icp_trajectory_trajectories.png` | 83KB | **GN-ICP轨迹对比图** - EVO生成的轨迹对比分析 |
| `gn_icp/real_evo_gn_icp_trajectory_xyz.png` | 72KB | **GN-ICP XYZ坐标图** - EVO生成的XYZ坐标变化图 |
| `gn_icp/real_evo_gn_icp_trajectory_rpy.png` | 41KB | **GN-ICP姿态角图** - EVO生成的姿态角变化图 |
| `gn_icp/real_evo_gn_icp_trajectory_speeds.png` | 68KB | **GN-ICP速度分析图** - EVO生成的速度分析图 |
| `gn_icp/gn_icp_trajectory_0_2d.png` | 126KB | **GN-ICP 2D轨迹图** - Python生成的2D轨迹图 |
| `gn_icp/gn_icp_trajectory_0_3d.png` | 458KB | **GN-ICP 3D轨迹图** - Python生成的3D轨迹图 |
| `gn_icp/gn_icp_trajectory_0_speed.png` | 266KB | **GN-ICP速度曲线** - Python生成的速度曲线 |
| `gn_icp/gn_icp_trajectory_0_stats.txt` | 233B | **GN-ICP统计信息** - GN-ICP轨迹统计信息 |



###  文档文件

| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `README.md` | 4.9KB | **项目说明** - 项目主要说明文档 |
| `PROJECT_FILES_ORGANIZATION.md` | 10.5KB | **文件组织说明** - 完整的文件整理说明 |
| `项目报告.md` | 14.4KB | **项目报告** - 完整的项目报告文档 |
| `COMPLETION_SUMMARY.md` | 5.5KB | **完成总结** - 项目完成情况总结 |
| `QUICK_START.md` | 994B | **快速开始** - 快速开始指南 |
| `RUN_PROJECT.md` | 4.2KB | **运行指南** - 项目运行指南 |
| `TASK_STATUS.md` | 7.5KB | **任务状态** - 任务完成状态 |
| `PROJECT_FINAL_REPORT.md` | 3.6KB | **最终报告** - 项目最终报告 |
| `BAG_PROCESSING_GUIDE.md` | 1.8KB | **Bag包处理指南** - Bag包处理说明 |
| `INSTALL_ROS.md` | 1.5KB | **ROS安装指南** - ROS安装说明 |
| `HOW_PROJECT_WORKS.md` | 6.1KB | **项目工作原理** - 项目工作原理说明 |
| `FILES_COMPLETE_SUMMARY.md` | 5.7KB | **文件完成总结** - 文件完成情况总结 |
| `FIXES_SUMMARY.md` | 3.6KB | **修复总结** - 问题修复总结 |
| `ACTUAL_FILES_STATUS.md` | 6.0KB | **实际文件状态** - 实际文件状态 |
| `所有文件位置汇总.md` | 6.4KB | **文件位置汇总** - 所有文件位置汇总 |

###  配置文件

| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `CMakeLists.txt` | 3.9KB | **CMake配置** - 项目编译配置文件 |
| `package.xml` | 814B | **ROS包配置** - ROS包配置文件 |
| `download_kitti.sh` | 1.5KB | **KITTI下载脚本** - 下载KITTI数据集脚本 |
| `convert_pcd_to_ply.py` | 1.4KB | **格式转换脚本** - PCD转PLY格式脚本 |

###  数据文件

| 文件名 | 大小 | 功能说明 |
|--------|------|----------|
| `data.bag` | 15.5GB | **ROS Bag包** - 激光雷达数据bag包 |
| `data_odometry_velodyne.zip` | 31.7GB | **KITTI数据集** - KITTI里程计数据集压缩包 |

###  目录结构

```
mapping/
├── src/                          # 源代码文件 (12个)
├── include/lidar_odometry/       # 头文件 (7个)
├── scripts/                      # 脚本文件 (5个)
├── results/                      # 结果文件 
│   ├── pointclouds/             # 点云文件 (4个)
│   ├── screenshots/            # 截图文件 (2个)
│   ├── videos/                 # 视频文件 (2个)
│   ├── trajectories/           # 轨迹文件 (3个)
│   └── visualizations/         # 可视化文件 (2个)
├── evo_evaluation_results/       # EVO评估结果
│   ├── ndt/                     # NDT评估结果 (8个文件)
│   └── gn_icp/                  # GN-ICP评估结果 (8个文件)
├── kitti_data/                  # KITTI数据集
├── build/                       # 编译输出 
├── launch/                      # ROS启动文件
├── config/                      # 配置文件
├── evo_env/                     # EVO虚拟环境 
├── data.bag                     # ROS Bag包 
└── PROJECT_FILES_ORGANIZATION.md # 文件组织说明
```

###  项目优化总结

#### 已完成的优化：
1. **演示程序清理**：删除了5个演示和测试文件
2. **文件整理**：所有结果文件按类型分类到results/目录
3. **目录清理**：删除了direct_processing_results/和install/目录
4. **脚本优化**：从12个脚本文件减少到5个核心文件
5. **运行方式优化**：删除run_all.sh，改为分步运行

#### 文件统计：
- **源代码文件**：12个 (已优化)
- **脚本文件**：5个 (已优化)
- **结果文件**：13个 (已分类)
- **可执行文件**：3个 (只保留真实KITTI里程计)
- **文档文件**：15个
- **项目总大小**：约16GB (包含数据集)

#### 空间优化：
- **演示程序清理**：删除5个演示/测试文件
- **脚本优化**：减少58%的脚本文件
- **目录清理**：节省约1.1MB冗余文件
- **文件分类**：提高项目结构清晰度

#### 已删除的文件：
- `complete_demo.cpp` - 生成模拟数据，不是真实KITTI
- `icp_odometry_standalone.cpp` - 演示程序
- `ndt_odometry_standalone.cpp` - 演示程序
- `simple_icp_test.cpp` - 测试程序
- `simple_ndt_test.cpp` - 测试程序
- `run_all.sh` - 完整运行脚本
- `direct_processing_results/` - 早期测试结果目录
- `install/` - ROS colcon安装目录


