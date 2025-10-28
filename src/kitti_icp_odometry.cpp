#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include "lidar_odometry/lidar_odometry.h"

using namespace lidar_odometry;

// KITTI点云读取函数
pcl::PointCloud<pcl::PointXYZ>::Ptr readKITTIPointCloud(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return cloud;
    }
    
    // KITTI点云格式：每个点4个float (x, y, z, intensity)
    // 但我们只使用x, y, z
    std::vector<float> buffer(4);
    while (file.read(reinterpret_cast<char*>(buffer.data()), 4 * sizeof(float))) {
        pcl::PointXYZ point;
        point.x = buffer[0];
        point.y = buffer[1];
        point.z = buffer[2];
        // 忽略intensity (buffer[3])
        cloud->push_back(point);
    }
    
    file.close();
    return cloud;
}

int main(int argc, char** argv) {
    std::cout << "=== GN-ICP Odometry with KITTI Data ===" << std::endl;
    
    // 默认路径
    std::string kitti_path = "/home/sss/mapping/kitti_data/data_odometry_velodyne/dataset";
    int sequence = 0;
    
    if (argc > 1) {
        kitti_path = argv[1];
    }
    if (argc > 2) {
        sequence = std::atoi(argv[2]);
    }
    
    std::cout << "KITTI路径: " << kitti_path << std::endl;
    std::cout << "序列号: " << sequence << std::endl;
    
    // 检查路径
    std::string velodyne_path = kitti_path + "/sequences/" + std::to_string(sequence) + "/velodyne";
    if (!std::filesystem::exists(velodyne_path)) {
        // 尝试两位数格式
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(2) << sequence;
        velodyne_path = kitti_path + "/sequences/" + ss.str() + "/velodyne";
    }
    
    if (!std::filesystem::exists(velodyne_path)) {
        std::cerr << "错误: 路径不存在 " << velodyne_path << std::endl;
        return -1;
    }
    
    // 创建GN-ICP里程计
    LidarOdometry odometry(LidarOdometry::RegistrationMethod::GN_ICP, 0.1, 100.0, 1.0);
    
    // 读取点云文件
    std::vector<std::string> bin_files;
    for (const auto& entry : std::filesystem::directory_iterator(velodyne_path)) {
        if (entry.path().extension() == ".bin") {
            bin_files.push_back(entry.path().string());
        }
    }
    
    // 排序文件
    std::sort(bin_files.begin(), bin_files.end());
    
    std::cout << "找到 " << bin_files.size() << " 个点云文件" << std::endl;
    
    if (bin_files.empty()) {
        std::cerr << "错误: 未找到点云文件" << std::endl;
        return -1;
    }
    
    // 处理前几帧（演示）
    int max_frames = std::min(10, (int)bin_files.size());
    std::cout << "处理前 " << max_frames << " 帧..." << std::endl;
    
    for (int i = 0; i < max_frames; i++) {
        std::cout << "处理帧 " << i << ": " << bin_files[i] << std::endl;
        
        // 读取点云
        auto cloud = readKITTIPointCloud(bin_files[i]);
        if (cloud->empty()) {
            std::cerr << "警告: 点云为空 " << bin_files[i] << std::endl;
            continue;
        }
        
        std::cout << "  点云大小: " << cloud->size() << " 点" << std::endl;
        
        // 处理点云
        double timestamp = i * 0.1; // 假设10Hz
        if (odometry.processFrame(cloud, timestamp)) {
            std::cout << "  ✓ 处理成功" << std::endl;
        } else {
            std::cout << "  ✗ 处理失败" << std::endl;
        }
    }
    
    // 保存结果
    std::string trajectory_file = "gn_icp_trajectory_kitti_" + std::to_string(sequence) + ".txt";
    if (odometry.saveTrajectory(trajectory_file)) {
        std::cout << "轨迹已保存: " << trajectory_file << std::endl;
    }
    
    // 保存全局地图
    std::string map_file = "gn_icp_global_map_kitti_" + std::to_string(sequence) + ".pcd";
    if (odometry.saveGlobalMap(map_file)) {
        std::cout << "全局地图已保存: " << map_file << std::endl;
    }
    
    // 获取统计信息
    int total_frames, successful_frames;
    double average_time;
    odometry.getStatistics(total_frames, successful_frames, average_time);
    
    std::cout << "\n统计信息:" << std::endl;
    std::cout << "  总帧数: " << total_frames << std::endl;
    std::cout << "  成功帧数: " << successful_frames << std::endl;
    std::cout << "  成功率: " << (double)successful_frames / total_frames * 100.0 << "%" << std::endl;
    std::cout << "  平均处理时间: " << average_time << " ms" << std::endl;
    
    std::cout << "\n=== 完成 ===" << std::endl;
    return 0;
}

