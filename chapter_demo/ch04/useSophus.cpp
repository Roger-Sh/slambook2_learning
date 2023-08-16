#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

#include "sophus/se3.hpp"

/**
 * @brief 本程序演示sophus的基本用法
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    /**
     * @brief SO3 <---> so3
     *
     */

    // 通过 Eigen 构建一个沿Z轴转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

    // 通过该旋转矩阵构建四元数
    Eigen::Quaterniond q(R);

    // 通过 Eigen 的旋转矩阵或四元数构建 Sophus 的 李群 SO3
    Sophus::SO3d SO3_R(R);  // Sophus::SO3d可以直接从旋转矩阵构造
    Sophus::SO3d SO3_q(q);  // 也可以通过四元数构造

    // 检查通过 R 或 q 构建的 SO3 是相等的
    std::cout << "SO(3) from matrix:\n" << SO3_R.matrix() << std::endl;
    std::cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << std::endl;
    std::cout << "they are equal" << std::endl;

    // 使用对数映射获得它的李代数（导数）
    Eigen::Vector3d so3 = SO3_R.log();
    std::cout << "so3 = " << so3.transpose() << std::endl;

    // 李代数的向量到反对称矩阵 hat
    std::cout << "so3 hat=\n" << SO3_R.hat(SO3_R.log()) << std::endl;
    std::cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << std::endl;

    // 李代数的反对称矩阵到向量 vee
    std::cout << "so3 hat vee= " << SO3_R.vee(SO3_R.hat(SO3_R.log())).transpose() << std::endl;
    std::cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << std::endl;

    // 增量扰动模型的更新
    Eigen::Vector3d update_so3(1e-4, 0, 0);                   // 假设 李代数 so3 的更新量为这么多
    Sophus::SO3d update_SO3 = Sophus::SO3d::exp(update_so3);  // 通过指数映射将更新量转化为 李群 SO3 的更新量
    Sophus::SO3d SO3_updated = update_SO3 * SO3_R;            // 通过左乘更新 SO3
    std::cout << "SO3 updated = \n" << SO3_updated.matrix() << std::endl;
    std::cout << "*******************************" << std::endl;

    /**
     * @brief SE3 <---> se3
     *
     */

    // 通过 Eigen 构建一个变换矩阵的平移向量
    Eigen::Vector3d t(1, 0, 0);  // 沿X轴平移1
    Sophus::SE3d SE3_Rt(R, t);   // 从R,t构造SE(3)
    Sophus::SE3d SE3_qt(q, t);   // 从q,t构造SE(3)
    std::cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << std::endl;
    std::cout << "SE3 from q,t= \n" << SE3_qt.matrix() << std::endl;

    // 李代数 se(3) 是一个六维向量，方便起见先typedef一下
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    // 对数映射获得 se3
    Vector6d se3 = SE3_Rt.log();

    // 观察输出，会发现在 Sophus 中，se(3)的平移在前，旋转在后.
    std::cout << "se3 = " << se3.transpose() << std::endl;

    // 李代数的向量到反对称矩阵 hat
    std::cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << std::endl;

    // 李代数的反对称矩阵到向量 vee
    std::cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << std::endl;

    // 设置更新量 update_se3
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;

    // 指数映射获得 update_SE3
    Sophus::SE3d update_SE3 = Sophus::SE3d::exp(update_se3);

    // 获得更新后的 SE3_updated
    Sophus::SE3d SE3_updated = update_SE3 * SE3_Rt;
    std::cout << "SE3 updated = " << std::endl << SE3_updated.matrix() << std::endl;

    return 0;
}
