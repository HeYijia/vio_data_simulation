//
// Created by hyj on 17-6-22.
//

#include <fstream>

#include "../src/imu.h"
#include "../src/utilities.h"
#include "camodocal/camera_models/CameraFactory.h"
std::string config_file = "../config/sim_config.yaml";
Param param;
std::vector < std::pair< Eigen::Vector4d, Eigen::Vector4d > >
CreatePointsLines(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >& points)
{

    std::vector < std::pair< Eigen::Vector4d, Eigen::Vector4d > > lines;

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
            lines.push_back( std::make_pair(pt0,pt1) );   // lines
        }
    }

    // create more 3d points, you can comment this code
    int n = points.size();
    for (int j = 0; j < n; ++j) {
        Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.5,0.5,-0.5,0);
        points.push_back(p);
    }

    // save points
    std::stringstream filename;
    filename<<"all_points.txt";
    save_points(filename.str(),points);
    return lines;
}
void addCamNoise(Eigen::Vector2d &obs){
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);
    Eigen::Vector2d noise_pixel(noise(generator_),noise(generator_));
    obs += param.pixel_noise * noise_pixel;
}
int main(){

//    Eigen::Quaterniond Qwb;
//    Qwb.setIdentity();
//    Eigen::Vector3d omega (0,0,M_PI/10);
//    double dt_tmp = 0.005;
//    for (double i = 0; i < 20.; i += dt_tmp) {
//        Eigen::Quaterniond dq;
//        Eigen::Vector3d dtheta_half =  omega * dt_tmp /2.0;
//        dq.w() = 1;
//        dq.x() = dtheta_half.x();
//        dq.y() = dtheta_half.y();
//        dq.z() = dtheta_half.z();
//
//        Qwb = Qwb * dq;
//    }
//
//    std::cout << Qwb.coeffs().transpose() <<"\n"<<Qwb.toRotationMatrix() << std::endl;

    // 生成3d points
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points;
    std::vector < std::pair< Eigen::Vector4d, Eigen::Vector4d > > lines;
    lines = CreatePointsLines(points);

    // IMU model
    Param params;
    IMU imuGen(params);
    camodocal::CameraPtr m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file);
    // create imu data
    // imu pose gyro acc
    std::vector< MotionData > imudata;
    std::vector< MotionData > imudata_noise;
    for (float t = params.t_start; t<params.t_end;) {
        MotionData data = imuGen.MotionModel(t);
        imudata.push_back(data);

        // add imu noise
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);
        imudata_noise.push_back(data_noise);

        t += 1.0/params.imu_frequency;
    }
    imuGen.init_velocity_ = imudata[0].imu_velocity;
    imuGen.init_twb_ = imudata.at(0).twb;
    imuGen.init_Rwb_ = imudata.at(0).Rwb;
    save_Pose("imu_pose.txt",imudata);
    save_Pose("imu_pose_noise.txt",imudata_noise);

    imuGen.testImu("imu_pose.txt", "imu_int_pose.txt");     // test the imu data, integrate the imu data to generate the imu trajecotry
    imuGen.testImu("imu_pose_noise.txt", "imu_int_pose_noise.txt");
    imuGen.testImuMidPoint("imu_pose_noise.txt", "imu_int_pose_noise_midpoint.txt");
    imuGen.testImuMidPoint("imu_pose.txt", "imu_int_pose_midpoint.txt");

    // cam pose
    std::vector< MotionData > camdata;
    std::vector< MotionData > imudata_correspond_cam;
    for (float t = params.t_start; t<params.t_end;) {

        MotionData imu = imuGen.MotionModel(t);   // imu body frame to world frame motion
        MotionData cam;

        cam.timestamp = imu.timestamp;
        cam.Rwb = imu.Rwb * params.R_bc;    // cam frame in world frame
        cam.twb = imu.twb + imu.Rwb * params.t_bc; //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb
        imudata_correspond_cam.push_back(imu);
        camdata.push_back(cam);
        t += 1.0/params.cam_frequency;
    }
    save_Pose("cam_pose.txt",camdata);
    save_Pose_asTUM("cam_pose_tum.txt",camdata);
    save_Pose_asTUM("imu_pose_tum_correspondence_cam.txt",imudata_correspond_cam);
    int last_feature_num;
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
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > pixels_cam;  // 对应的２维图像坐标
        for (int i = 0; i < points.size(); ++i) {
            Eigen::Vector4d pw = points[i];          // 最后一位存着feature id
            pw[3] = 1;                               //改成齐次坐标最后一位
            Eigen::Vector4d pc1 = Twc.inverse() * pw; // T_wc.inverse() * Pw  -- > point in cam frame

            if(pc1(2) < 0) continue; // z必须大于０,在摄像机坐标系前方

            Eigen::Vector2d obs(pc1(0)/pc1(2), pc1(1)/pc1(2)) ;
//            if( (obs(0)*460 + 255) < params.image_h && ( obs(0) * 460 + 255) > 0 &&
//                    (obs(1)*460 + 255) > 0 && ( obs(1)* 460 + 255) < params.image_w )
            {
                points_cam.push_back(points[i]);
                features_cam.push_back(obs);
                Eigen::Vector2d obs_pixels;
                m_camera->undistToPlane(obs, obs_pixels);
                addCamNoise(obs_pixels);
                pixels_cam.push_back(obs_pixels);
            }
        }
        if(n == 0)
            last_feature_num = features_cam.size();
        else {
            assert(last_feature_num == features_cam.size());
            last_feature_num = features_cam.size();
        }
        // save points
        std::stringstream filename1;
        std::stringstream filename2;
        filename1<<"keyframe/all_points_"<<n<<".txt";
        filename2<<"keyframe/pixel_all_points_"<<n<<".txt";
        save_features(filename1.str(),points_cam,features_cam, camdata[n].timestamp);
        save_features(filename2.str(),points_cam,pixels_cam, camdata[n].timestamp);
//        save_features(filename1.str(),points_cam,features_cam);
    }

    // lines obs in image
    for(int n = 0; n < camdata.size(); ++n)
    {
        MotionData data = camdata[n];
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = data.Rwb;
        Twc.block(0, 3, 3, 1) = data.twb;

        // 遍历所有的特征点，看哪些特征点在视野里
//        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_cam;    // ３维点在当前cam视野里
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features_cam;  // 对应的２维图像坐标
        for (int i = 0; i < lines.size(); ++i) {
            std::pair< Eigen::Vector4d, Eigen::Vector4d > linept = lines[i];

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
