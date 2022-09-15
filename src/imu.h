//
// Created by hyj on 18-1-19.
//

#ifndef IMUSIMWITHPOINTLINE_IMU_H
#define IMUSIMWITHPOINTLINE_IMU_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>

#include "param.h"

struct MotionData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp; //采样时间 
    Eigen::Matrix3d Rwb;//旋转矩阵
    Eigen::Vector3d twb;//平移矩阵
    Eigen::Vector3d imu_acc;// 加速度计测量的 加速度
    Eigen::Vector3d imu_gyro;// 陀螺仪测量的 角速度

    Eigen::Vector3d imu_gyro_bias;// 陀螺仪  离散时间的bias随机游走
    Eigen::Vector3d imu_acc_bias;// 加速度计  离散时间的bias随机游走

    Eigen::Vector3d imu_velocity;//imu的速度
};

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles);
Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles);


class IMU
{
public:
    IMU(Param p);//IMU类的有参构造  定义gyro_bias_ 和 acc_bias_ 为3x3的0矩阵
    Param param_;
    Eigen::Vector3d gyro_bias_;
    Eigen::Vector3d acc_bias_;

    Eigen::Vector3d init_velocity_;
    Eigen::Vector3d init_twb_;
    Eigen::Matrix3d init_Rwb_;
    // 指定轨迹方程，求一阶导得到速度,角速度，求二阶导得到加速度
    // 并将求得的 速度  角速度 加速度 时间戳 旋转矩阵 平移矩阵 赋值给结构体MotionData对象data的成员变量
    MotionData MotionModel(double t);

    void addIMUnoise(MotionData& data);//向imu数据data里添加噪声
    void testImu(std::string src, std::string dist);        // imu数据进行积分，用来看imu轨迹

};

#endif //IMUSIMWITHPOINTLINE_IMU_H
