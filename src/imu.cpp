//
// Created by hyj on 18-1-19.
//

#include <random>
#include "imu.h"
#include "utilities.h"
#include<iostream>
using namespace std;

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;
    return RIb;
}

Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3d R;
    R<<  1,   0,    -sp,
            0,   cr,   sr*cp,
            0,   -sr,  cr*cp;

    return R;
}


IMU::IMU(Param p): param_(p)//IMU类的有参构造  定义gyro_bias_ 和 acc_bias_ 为3x3的0矩阵
{
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();
}
//向imu数据data里添加离散时间的高斯白噪声noise和离散时间的bias随机游走，并更新 加速度和角速度
void IMU::addIMUnoise(MotionData& data)
{
    std::random_device rd;//非确定性(真)随机数生成器  
    std::default_random_engine generator_(rd());//创建随机数引擎对象  传入一个随机数种子
    std::normal_distribution<double> noise(0.0, 1.0);//随机数的正态连续分布。

    Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));//随机生成服从正态分布的值   方差为 1 的白噪声
    // cout <<"noise_gyro:"  << endl;
    // cout<<noise_gyro << endl;
    // noise_gyro:
    // -1.35678
    // -0.33964
    // -0.199193
    // noise_gyro:
    // 0.264621
    // -0.522613
    // 0.0166623
    Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity();// 陀螺仪的高斯白噪声协方差
    //     gyro_sqrt_cov:
    // 0.015     0     0
    //     0 0.015     0
    //     0     0 0.015

    // cout <<"gyro_sqrt_cov:"  << endl;
    // cout<<gyro_sqrt_cov << endl;
    // 忽略 scale 的影响，只考虑白噪声和 bias 随机游走
    // 角速度 w = w + 离散时间的高斯白噪声noise + 离散时间的bias随机游走
    // 高斯白噪声的连续时间到离散时间之间差一个 1/√∆t ,√∆t是采样时间 
    // 离散时间的高斯白噪声noise: nd[k] = sigema/√∆t  * w[k] = gyro_sqrt_cov  / sqrt( param_.imu_timestep ) * noise_gyro
    data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt( param_.imu_timestep ) + gyro_bias_;  //3个轴的角速度一起算
    // IMU的角速度测量值 imu_gyro
    // cout <<"data.imu_gyro:"  << endl;
    // cout<<data.imu_gyro << endl;
    // data.imu_gyro:
    // -0.380758
    // 0.118272
    // 0.0782825
    // data.imu_gyro:
    // -0.275656
    // 0.115311
    // 0.618848

    
    // cout << "2.1174628193741066e-01 / sqrt(200) = " << 2.1174628193741066e-01 / sqrt(200) << endl;
    // cout << "8.5047491324959210e-04 / sqrt(200) = " << 8.5047491324959210e-04 / sqrt(200) << endl;
    // cout << "2.6979648977867060e-01 / sqrt(200) = " << 2.6979648977867060e-01 / sqrt(200) << endl;
    // cout << "3.8990219990271773e-03 / sqrt(200) = " << 3.8990219990271773e-03 / sqrt(200) << endl;

    
    Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));//随机生成服从正态分布的值   方差为 1 的白噪声
    Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();// 加速度计的高斯白噪声协方差
    // 忽略 scale 的影响，只考虑白噪声和 bias 随机游走
    // 加速度 a = a + 离散时间的高斯白噪声noise + 离散时间的bias随机游走
    // 离散时间的高斯白噪声noise: nd[k] = sigema/√∆t  * w[k] = acc_sqrt_cov  / sqrt( param_.imu_timestep ) * noise_acc
    // IMU的加速度测量值 imu_acc
    data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt( param_.imu_timestep ) + acc_bias_;

    // gyro_bias update 陀螺仪 离散时间的bias随机游走 更新
    Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));//随机生成服从正态分布的值   方差为 1 的白噪声
    // bias 随机游走的噪声方差从连续时间到离散之间需要乘以 √∆t 
    // 离散时间的bias随机游走gyro_bias_  ：bd[k] = bd[k-1] + sigema * √∆t * w[k] = param_.gyro_bias_sigma * sqrt(param_.imu_timestep ) * noise_gyro_bias;
    gyro_bias_ += param_.gyro_bias_sigma * sqrt(param_.imu_timestep ) * noise_gyro_bias;
    data.imu_gyro_bias = gyro_bias_;// 陀螺仪 离散时间的bias随机游走 更新

    // acc_bias update 加速度计 离散时间的bias随机游走 更新
    Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));//随机生成服从正态分布的值   方差为 1 的白噪声
    // bias 随机游走的噪声方差从连续时间到离散之间需要乘以 √∆t 
    // 离散时间的bias随机游走acc_bias_  ：bd[k] = bd[k-1] + sigema * √∆t * w[k] = param_.acc_bias_sigma * sqrt(param_.imu_timestep ) * noise_acc_bias;
    acc_bias_ += param_.acc_bias_sigma * sqrt(param_.imu_timestep ) * noise_acc_bias;
    data.imu_acc_bias = acc_bias_;// 加速度计 离散时间的bias随机游走 更新

}
// 指定轨迹方程，求一阶导得到速度,角速度，求二阶导得到加速度
// 并将求得的 速度  角速度 加速度 时间戳 旋转矩阵 平移矩阵 赋值给结构体MotionData对象data的成员变量
MotionData IMU::MotionModel(double t)
{

    MotionData data;//结构体对象data
    // param
    float ellipse_x = 15;
    float ellipse_y = 20;
    float z = 1;           // z轴做sin运动
    float K1 = 10;          // z轴的正弦频率是x，y的k1倍
    float K = M_PI/ 10;    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

    // translation 平移
    // twb:  body frame in world frame
    // 指定轨迹方程，求一阶导得到速度,角速度，求二阶导得到加速度
    Eigen::Vector3d position( ellipse_x * cos( K * t) + 5, ellipse_y * sin( K * t) + 5,  z * sin( K1 * K * t ) + 5);  //t=0 起点 (20,5,5,)
    Eigen::Vector3d dp(- K * ellipse_x * sin(K*t),  K * ellipse_y * cos(K*t), z*K1*K * cos(K1 * K * t));              // position导数　in world frame
    double K2 = K*K;
    Eigen::Vector3d ddp( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), -z*K1*K1*K2 * sin(K1 * K * t));     // position二阶导数

    // Rotation 旋转
    double k_roll = 0.1;
    double k_pitch = 0.2;
    Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数 

//    Eigen::Vector3d eulerAngles(0.0,0.0, K*t );   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
//    Eigen::Vector3d eulerAnglesRates(0.,0. , K);      // euler angles 的导数

    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro  //角速度

    Eigen::Vector3d gn (0,0,-9.81);                                   //  gravity in navigation frame(ENU)   导航系为东北天ENU (0,0,-9.81)  东北地NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs   //加速度
//  struct MotionData的成员赋值
    data.imu_gyro = imu_gyro;//陀螺仪   欧拉角速度     IMU的角速度真实值
    data.imu_acc = imu_acc;//加速度计   加速度         IMU的加速度真实值
    data.Rwb = Rwb; //旋转矩阵
    data.twb = position; //平移矩阵
    data.imu_velocity = dp; //速度
    data.timestamp = t;//时间戳
    return data;

}

//读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testImu(std::string src, std::string dist) // imu数据进行积分，用来看imu轨迹
{
    std::vector<MotionData>imudata;
    LoadPose(src,imudata);//将src文件（如 imu_pose.txt）里面的data数据加载到imudata

    std::ofstream save_points;//写操作
    save_points.open(dist);//打开imu积分以后的轨迹  imu_int_pose.txt 或者 imu_int_pose_noise.txt

    double dt = param_.imu_timestep; // 0.005s
    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements  初始的平移矩阵
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements   初始的旋转矩阵
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements   初始的速度
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame  导航系为东北天
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    for (int i = 1; i < imudata.size(); ++i) {

        MotionData imupose = imudata[i];//imu的位姿

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]； 即qwb′  = qwb ⊗ [1  1/2 * ω∆t ]
        Eigen::Quaterniond dq;
        // 欧拉积分
        Eigen::Vector3d dtheta_half = imupose.imu_gyro  * dt /2.0; // qwb′  = qwb ⊗ [1  1/2 * ω∆t ] 的 1/2 * ω∆t 部分，ω是角速度的真实值imudata[i].imu_gyro 
        // 中值积分
        // Eigen::Vector3d dtheta_half = ( imudata[i - 1].imu_gyro + imupose.imu_gyro)/ 2.0 * dt / 2.0; // 
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();
        dq.normalize();
        
        // imu 动力学模型 欧拉积分
        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
        Qwb = Qwb * dq;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
        Vw = Vw + acc_w * dt;
        
        // /// imu 动力学模型 中值积分
        // Eigen::Vector3d acc_w = (Qwb*(imudata[i - 1].imu_acc) + gw + Qwb*dq*(imupose.imu_acc) + gw) / 2.0;
        // Qwb = Qwb * dq;
        // Vw = Vw + acc_w * dt;
        // Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;

        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points<<imupose.timestamp<<" "//写数据
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<std::endl;

    }


    std::cout<<"test　end"<<std::endl;// 调用两次，输出2次test　end
}
