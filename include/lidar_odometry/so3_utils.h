#ifndef SO3_UTILS_H
#define SO3_UTILS_H

#include <Eigen/Dense>
#include <Eigen/Core>

namespace lidar_odometry {

/**
 * @brief SO(3) 李群工具函数
 * 来自 code.zip 中的 common.h
 */

/**
 * @brief 计算向量的反对称矩阵（hat 算子）
 * @param v 输入向量
 * @return 3x3 反对称矩阵
 */
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> Hat(const Eigen::MatrixBase<Derived> &v)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> skew_mat; // 反对称矩阵
    skew_mat.setZero();
    skew_mat(0, 1) = -v(2);
    skew_mat(0, 2) = v(1);
    skew_mat(1, 2) = -v(0);
    skew_mat(1, 0) = v(2);
    skew_mat(2, 0) = -v(1);
    skew_mat(2, 1) = v(0);
    return skew_mat;
}

/**
 * @brief SO(3) 指数映射
 * 使用罗德里斯格公式将李代数映射到旋转矩阵
 * @param v 李代数向量 (so(3))
 * @return 旋转矩阵 R (SO(3))
 */
template<typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> SO3Exp(const Eigen::MatrixBase<Derived> &v) 
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
    typename Derived::Scalar theta = v.norm(); // 计算v的模
    Eigen::Matrix<typename Derived::Scalar, 3, 1> v_normalized = v.normalized(); // 归一化
    // 罗德里格斯公式
    R = cos(theta) * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + 
        (typename Derived::Scalar(1.0) - cos(theta)) * v_normalized * v_normalized.transpose() + 
        sin(theta) * Hat(v_normalized);

    return R;
}

/**
 * @brief SO(3) 对数映射
 * 从旋转矩阵提取李代数
 * @param R 旋转矩阵
 * @return 李代数向量
 */
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> SO3Log(const Eigen::Matrix<Scalar, 3, 3>& R)
{
    Eigen::Matrix<Scalar, 3, 1> v;
    Scalar trace = R.trace();
    Scalar theta = acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
    
    if (theta < 1e-6) {
        // 小角度近似
        Eigen::Matrix<Scalar, 3, 3> skew = (R - R.transpose()) / 2.0;
        v << -skew(1, 2), skew(0, 2), -skew(0, 1);
    } else {
        Eigen::Matrix<Scalar, 3, 3> skew = (R - R.transpose()) / 2.0 / sin(theta);
        v << -skew(1, 2), skew(0, 2), -skew(0, 1);
        v *= theta;
    }
    
    return v;
}

} // namespace lidar_odometry

#endif // SO3_UTILS_H

