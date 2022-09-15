//
// Created by hyj on 17-6-22.
//

#include <fstream>
#include <sys/stat.h>
#include "../src/imu.h"
#include "../src/utilities.h"

using Point = Eigen::Vector4d;// 4x1
using Points = std::vector<Point, Eigen::aligned_allocator<Point> >;//aligned_allocator:STL兼容的分配器，用于需要非标准对齐的类型。
using Line = std::pair<Eigen::Vector4d, Eigen::Vector4d>;
using Lines = std::vector<Line, Eigen::aligned_allocator<Line> >;

void CreatePointsLines(Points& points, Lines& lines)
{
    std::ifstream f;
    f.open("house_model/house.txt");

    while(!f.eof())
    {
        std::string s;
        std::getline(f,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double x,y,z;
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt0( x, y, z, 1 );
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt1( x, y, z, 1 );

            bool isHistoryPoint = false;
            for (int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pt = points[i];
                if(pt == pt0)
                {
                    isHistoryPoint = true;
                }
            }
            if(!isHistoryPoint)
                points.push_back(pt0);

            isHistoryPoint = false;
            for (int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pt = points[i];
                if(pt == pt1)
                {
                    isHistoryPoint = true;
                }
            }
            if(!isHistoryPoint)
                points.push_back(pt1);

            // pt0 = Twl * pt0;
            // pt1 = Twl * pt1;
            lines.emplace_back(pt0, pt1);   // lines
        }
    }

    // create more 3d points, you can comment this code
    int n = points.size();
    for (int j = 0; j < n; ++j) {
        Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.5,0.5,-0.5,0);
        points.push_back(p);
    }

    // save points
    save_points("all_points.txt", points);
}

int main(){

    // Eigen::Quaterniond Qwb;
    // Qwb.setIdentity();
    // Eigen::Vector3d omega (0,0,M_PI/10);
    // double dt_tmp = 0.005;
    // for (double i = 0; i < 20.; i += dt_tmp) {
    //     Eigen::Quaterniond dq;
    //     Eigen::Vector3d dtheta_half =  omega * dt_tmp /2.0;
    //     dq.w() = 1;
    //     dq.x() = dtheta_half.x();
    //     dq.y() = dtheta_half.y();
    //     dq.z() = dtheta_half.z();
    //     Qwb = Qwb * dq;
    // }
    // std::cout << Qwb.coeffs().transpose() <<"\n"<<Qwb.toRotationMatrix() << std::endl;

    // 建立keyframe文件夹
    mkdir("keyframe", 0777);

    // 生成3d points
    Points points;
    Lines lines;
    CreatePointsLines(points, lines);

    // IMU model
    Param params;//定义Param类对象
    IMU imuGen(params);//定义IMU类对象，并调用有参构造

    // create imu data //创建imu数据（速度  角速度 加速度 采样时间 旋转矩阵 平移矩阵）
    // imu pose gyro acc;   位姿pose 陀螺仪的角速度gyro  加速度计的加速度acc
    std::vector< MotionData > imudata;//定义数据类型为MotionData结构体的vector容器imudata
    std::vector< MotionData > imudata_noise;//定义数据类型为MotionData结构体的vector容器imudata_noise
    for (float t = params.t_start; t<params.t_end;) {   //起始时间 params.t_start 到结束时间 params.t_end 即0-20s；imu的采样时间：1/200 = 0.002s
        // 指定轨迹方程，求一阶导得到速度,角速度，求二阶导得到加速度
        // 并将求得的 速度  角速度 加速度 采样时间 旋转矩阵 平移矩阵 赋值给结构体MotionData对象data的成员变量
        MotionData data = imuGen.MotionModel(t); //通过IMU类对象imuGen 调用 IMU类的成员函数 MotionData MotionModel(double t);  MotionData是结构体的数据类型
        imudata.push_back(data);//将imu的数据data（速度  角速度 加速度 采样时间 旋转矩阵 平移矩阵）通过尾插 向（数据类型为MotionData结构体的）vector容器imudata中添加元素

        // add imu noise //向imu中添加噪声
        MotionData data_noise = data;//先把之前的imu数据data 赋值给 data_noise
        //向imu数据data里添加离散时间的高斯白噪声noise和离散时间的bias随机游走，并更新 加速度和角速度
        imuGen.addIMUnoise(data_noise);//通过IMU类对象imuGen 调用  IMU类的公有成员函数 void addIMUnoise(MotionData& data);
        imudata_noise.push_back(data_noise);//将添加噪声的imu的数据data_noise（速度  角速度 加速度 采样时间 旋转矩阵 平移矩阵）通过尾插 向（数据类型为MotionData结构体的）vector容器imudata_noise中添加元素

        t += 1.0/params.imu_frequency;// 1/200 = 0.002s imu的采样时间，注意t是float型的变量
    }
    imuGen.init_velocity_ = imudata[0].imu_velocity;//给IMU类对象imuGen的成员变量init_velocity_赋值   初始的速度
    imuGen.init_twb_ = imudata.at(0).twb;//给IMU类对象imuGen的成员变量init_twb_赋值  初始的平移矩阵
    imuGen.init_Rwb_ = imudata.at(0).Rwb;//给IMU类对象imuGen的成员变量init_Rwb_赋值  初始的旋转矩阵
    save_Pose("imu_pose.txt", imudata);//存储imu的位姿 格式：time q.w() q.x() q.y() q.z() t(0) t(1) t(2) gyro(0) gyro(1) gyro(2) acc(0) acc(1) acc(2)
    save_Pose("imu_pose_noise.txt", imudata_noise);//存储向imu中添加噪声后的位姿 格式：time q.w() q.x() q.y() q.z() t(0) t(1) t(2) gyro(0) gyro(1) gyro(2) acc(0) acc(1) acc(2)

    // 测试imu数据，整合imu数据生成imu轨迹
    // test the imu data, integrate the imu data to generate the imu trajecotry
    // 读取生成的imu数据（imu_pose.txt）并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹（imu_int_pose.txt）
    // 读取生成的imu数据（imu_pose_noise.txt）并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹（imu_int_pose_noise.txt）
    imuGen.testImu("imu_pose.txt", "imu_int_pose.txt"); //通过IMU类对象imuGen 调用  IMU类的公有成员函数 void testImu(std::string src, std::string dist); 
    imuGen.testImu("imu_pose_noise.txt", "imu_int_pose_noise.txt");  
    

    // cam pose
    std::vector< MotionData > camdata;
    for (float t = params.t_start; t<params.t_end;) {

        MotionData imu = imuGen.MotionModel(t);   // imu body frame to world frame motion
        MotionData cam;

        cam.timestamp = imu.timestamp;
        cam.Rwb = imu.Rwb * params.R_bc;    // cam frame in world frame
        cam.twb = imu.twb + imu.Rwb * params.t_bc; //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb

        camdata.push_back(cam);
        t += 1.0/params.cam_frequency;
    }
    save_Pose("cam_pose.txt",camdata);
    save_Pose_asTUM("cam_pose_tum.txt",camdata);

    // points obs in image
    for(int n = 0; n < camdata.size(); ++n)
    {
        MotionData data = camdata[n];
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = data.Rwb;
        Twc.block(0, 3, 3, 1) = data.twb;

        // 遍历所有的特征点，看哪些特征点在视野里
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_cam;    // ３维点在当前cam视野里
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features_cam;  // 对应的２维图像坐标
        for (int i = 0; i < points.size(); ++i) {
            Eigen::Vector4d pw = points[i];          // 最后一位存着feature id
            pw[3] = 1;                               //改成齐次坐标最后一位
            Eigen::Vector4d pc1 = Twc.inverse() * pw; // T_wc.inverse() * Pw  -- > point in cam frame

            if(pc1(2) < 0) continue; // z必须大于０,在摄像机坐标系前方

            Eigen::Vector2d obs(pc1(0)/pc1(2), pc1(1)/pc1(2)) ;
            // if( (obs(0)*460 + 255) < params.image_h && ( obs(0) * 460 + 255) > 0 &&
                   // (obs(1)*460 + 255) > 0 && ( obs(1)* 460 + 255) < params.image_w )
            {
                points_cam.push_back(points[i]);
                features_cam.push_back(obs);
            }
        }

        // save points
        std::stringstream filename1;
        filename1<<"keyframe/all_points_"<<n<<".txt";
        save_features(filename1.str(),points_cam,features_cam);
    }

    // lines obs in image
    for(int n = 0; n < camdata.size(); ++n)
    {
        MotionData data = camdata[n];
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = data.Rwb;
        Twc.block(0, 3, 3, 1) = data.twb;

        // 遍历所有的特征点，看哪些特征点在视野里
        // std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_cam;    // ３维点在当前cam视野里
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features_cam;  // 对应的２维图像坐标
        for (int i = 0; i < lines.size(); ++i) {
            Line linept = lines[i];

            Eigen::Vector4d pc1 = Twc.inverse() * linept.first; // T_wc.inverse() * Pw  -- > point in cam frame
            Eigen::Vector4d pc2 = Twc.inverse() * linept.second; // T_wc.inverse() * Pw  -- > point in cam frame

            if(pc1(2) < 0 || pc2(2) < 0) continue; // z必须大于０,在摄像机坐标系前方

            Eigen::Vector4d obs(pc1(0)/pc1(2), pc1(1)/pc1(2),
                                pc2(0)/pc2(2), pc2(1)/pc2(2));
            //if(obs(0) < params.image_h && obs(0) > 0 && obs(1)> 0 && obs(1) < params.image_w)
            {
                features_cam.push_back(obs);
            }
        }

        // save points
        std::stringstream filename1;
        filename1<<"keyframe/all_lines_"<<n<<".txt";
        save_lines(filename1.str(),features_cam);
    }

    return 0;
}
