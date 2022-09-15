//
// Created by hyj on 17-6-22.
//

#ifndef IMUSIM_PARAM_H
#define IMUSIM_PARAM_H

#include <eigen3/Eigen/Core>

class Param{

public:

    Param();

    // time
    int imu_frequency = 200;
    int cam_frequency = 30;
    double imu_timestep = 1./imu_frequency; // imu的采样时间  0.005s
    double cam_timestep = 1./cam_frequency; // 相机的采样时间  1/30s
    double t_start = 0.;//起始时间
    double t_end = 20;  //  20 s  结束时间

    // noise
    double gyro_bias_sigma = 1.0e-5;//陀螺仪的 bias 随机游走噪声
    double acc_bias_sigma = 0.0001;//加速度 bias 的随机游走噪声

    double gyro_noise_sigma = 0.015;    // rad/s * 1/sqrt(hz)  陀螺仪的高斯白噪声
    double acc_noise_sigma = 0.019;      //　m/(s^2) * 1/sqrt(hz)  加速度的高斯白噪声

    double pixel_noise = 1;              // 1 pixel noise

    // cam f
    double fx = 460;
    double fy = 460;
    double cx = 255;
    double cy = 255;
    double image_w = 640;
    double image_h = 640;


    // 外参数
    Eigen::Matrix3d R_bc;   // cam to body
    Eigen::Vector3d t_bc;     // cam to body

};


#endif //IMUSIM_PARAM_H
