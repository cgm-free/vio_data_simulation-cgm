//
// Created by hyj on 18-1-19.
//
#include "utilities.h"


void save_points(std::string filename, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points)
{
    std::ofstream save_points;
    save_points.open(filename.c_str());

    for (int i = 0; i < points.size(); ++i) {
        Eigen::Vector4d p = points[i];

        save_points<<p(0)<<" "
                   <<p(1)<<" "
                   <<p(2)<<" "
                   <<p(3)<<std::endl;
    }
}
void save_features(std::string filename,
                   std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points,
                   std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features)
{
    std::ofstream save_points;
    save_points.open(filename.c_str());

    for (int i = 0; i < points.size(); ++i) {
        Eigen::Vector4d p = points[i];
        Eigen::Vector2d f = features[i];
        save_points<<p(0)<<" "
                   <<p(1)<<" "
                   <<p(2)<<" "
                   <<p(3)<<" "
                   <<f(0)<<" "
                   <<f(1)<<" "
                   <<std::endl;
    }
}
void save_lines(std::string filename,
                std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features)
{
    std::ofstream save_points;
    save_points.open(filename.c_str());

    for (int i = 0; i < features.size(); ++i) {
        Eigen::Vector4d f = features[i];
        save_points<<f(0)<<" "
                   <<f(1)<<" "
                   <<f(2)<<" "
                   <<f(3)<<" "
                   <<std::endl;
    }
}

void LoadPose(std::string filename, std::vector<MotionData>& pose)
{

    std::ifstream f;//读操作
    f.open(filename.c_str());//open()成员函数的参数是一个char *类型的量，要通过调用 string 类的 c_str() 函数返回一个C风格的字符串
    // 功能：open 函数将 filename 转换为一个文件描述符，并且返回描述符数字（整型变量0~255）。返回的描述符总是在进程中当前没有打开的最小描述符。

    if(!f.is_open())//is_open()这个函数用于判断文件是否成功打开。它不仅具有判断文件是否成功打开的功能，而且能判断是否文件采用了合适的方式被打开。
    {
        std::cerr << " can't open LoadFeatures file "<<std::endl;
        return;
    }

    while (!f.eof()) { // eof()函数可以帮助我们⽤来判断⽂件是否为空，抑或是判断其是否读到⽂件结尾。

        std::string s;
        std::getline(f,s);

        if(! s.empty())
        {
            std::stringstream ss;//<sstream> 定义了三个类：istringstream：流的输入；ostringstream：流的输出；stringstream：流的输入输出。
            ss << s; //将string类型的值放入输入流中

            MotionData data;
            double time;
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            Eigen::Vector3d gyro;
            Eigen::Vector3d acc;

            ss>>time;// 输出时间戳数据（double型）
            ss>>q.w();
            ss>>q.x();
            ss>>q.y();
            ss>>q.z();
            ss>>t(0);
            ss>>t(1);
            ss>>t(2);
            ss>>gyro(0);
            ss>>gyro(1);
            ss>>gyro(2);
            ss>>acc(0);
            ss>>acc(1);
            ss>>acc(2);


            data.timestamp = time;// 将时间戳数据放在timestamp中
            data.imu_gyro = gyro;
            data.imu_acc = acc;
            data.twb = t;
            data.Rwb = Eigen::Matrix3d(q);
            pose.push_back(data);

        }
    }

}

void save_Pose(std::string filename, std::vector<MotionData> pose)
{
    std::ofstream save_points;//写操作
    save_points.open(filename.c_str());////open()成员函数的参数是一个char *类型的量，要通过调用 string 类的 c_str() 函数返回一个C风格的字符串
    // 功能：open 函数将 filename 转换为一个文件描述符，并且返回描述符数字（整型变量0~255）。返回的描述符总是在进程中当前没有打开的最小描述符。

    for (int i = 0; i < pose.size(); ++i) { 
        MotionData data = pose[i];
        double time = data.timestamp;//imu采样时间 间隔0.005s
        Eigen::Quaterniond q(data.Rwb);
        Eigen::Vector3d t = data.twb;
        Eigen::Vector3d gyro = data.imu_gyro;
        Eigen::Vector3d acc = data.imu_acc;

        save_points<<time<<" " //写数据
                   <<q.w()<<" "
                   <<q.x()<<" "
                   <<q.y()<<" "
                   <<q.z()<<" "
                   <<t(0)<<" "
                   <<t(1)<<" "
                   <<t(2)<<" "
                   <<gyro(0)<<" "
                   <<gyro(1)<<" "
                   <<gyro(2)<<" "
                   <<acc(0)<<" "
                   <<acc(1)<<" "
                   <<acc(2)<<" "
                   <<std::endl;
    }
}

void save_Pose_asTUM(std::string filename, std::vector<MotionData> pose)
{
    std::ofstream save_points;
    save_points.setf(std::ios::fixed, std::ios::floatfield);
    save_points.open(filename.c_str());

    for (int i = 0; i < pose.size(); ++i) {
        MotionData data = pose[i];
        double time = data.timestamp;
        Eigen::Quaterniond q(data.Rwb);
        Eigen::Vector3d t = data.twb;
        Eigen::Vector3d gyro = data.imu_gyro;
        Eigen::Vector3d acc = data.imu_acc;

        save_points.precision(9);
        save_points <<time<<" ";
        save_points.precision(5);
        save_points <<t(0)<<" "
                    <<t(1)<<" "
                    <<t(2)<<" "
                    <<q.x()<<" "
                    <<q.y()<<" "
                    <<q.z()<<" "
                    <<q.w() <<std::endl;
    }

}