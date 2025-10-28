#ifndef LIO_NDT_FRONT_END_HPP_
#define LIO_NDT_FRONT_END_HPP_

#include <deque>
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include "lio_ndt/sensor_data/cloud_data.hpp"
#include "lio_ndt/method/optimized_ICP_GN.h"

namespace lio_ndt
{
    // 配准方法枚举
    enum class RegistrationMethod {
        NDT = 0,
        ICP = 1,
        OPTIMIZED_ICP = 2
    };

    class FrontEnd
    {
    public:
        struct Frame
        {
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            CloudData cloud_data;
        };

    public:
        FrontEnd();

        Eigen::Matrix4f Update(const CloudData& cloud_data);
        bool SetInitPose(const Eigen::Matrix4f& init_pose);
        bool SetPredictPose(const Eigen::Matrix4f& predict_pose);

        bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
        bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
        bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);

        // 设置和获取配准方法
        void SetRegistrationMethod(RegistrationMethod method);
        RegistrationMethod GetRegistrationMethod() const;

    private:
        void UpdateNewFrame(const Frame& new_key_frame);

        // 不同配准方法的实现
        void RegistrationNDT(CloudData::CLOUD_PTR source_cloud, 
                            const Eigen::Matrix4f& predict_pose,
                            CloudData::CLOUD_PTR result_cloud,
                            Eigen::Matrix4f& result_pose);
        
        void RegistrationICP(CloudData::CLOUD_PTR source_cloud,
                            const Eigen::Matrix4f& predict_pose,
                            CloudData::CLOUD_PTR result_cloud,
                            Eigen::Matrix4f& result_pose);
        
        void RegistrationOptimizedICP(CloudData::CLOUD_PTR source_cloud,
                                    const Eigen::Matrix4f& predict_pose,
                                    CloudData::CLOUD_PTR result_cloud,
                                    Eigen::Matrix4f& result_pose);

    private:
        // 配准算法实例
        pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
        pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_;
        OptimizedICPGN icp_opti_;
        RegistrationMethod registration_method_;

        // 滤波器
        pcl::VoxelGrid<CloudData::POINT> cloud_filter_;
        pcl::VoxelGrid<CloudData::POINT> local_map_filter_;
        pcl::VoxelGrid<CloudData::POINT> display_filter_;

        // 局部地图
        std::deque<Frame> local_map_frames_;
        std::deque<Frame> global_map_frames_;

        bool has_new_local_map_ = false;
        bool has_new_global_map_ = false;

        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR result_cloud_ptr_;
        Frame current_frame_;

        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();
        
        // 配准算法参数
        float ndt_resolution_ = 1.0;
        float ndt_step_size_ = 0.1;
        float max_correspond_distance_ = 0.3;
        int max_iterations_ = 30;
    };
}

#endif