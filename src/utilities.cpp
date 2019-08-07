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
/**
 * save the 3d points and 2d feature and timestamp of an image
 * @param filename
 * @param points 3d points in camera coordinate
 * @param features (x/z,y/z)
 * @param time timestamp of the image
 */
void save_features(std::string filename,
                   std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points,
                   std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features,
                   double time)
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
                   <<time<<" "
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

    std::ifstream f;
    f.open(filename.c_str());

    if(!f.is_open())
    {
        std::cerr << " can't open LoadFeatures file "<<std::endl;
        return;
    }

    while (!f.eof()) {

        std::string s;
        std::getline(f,s);

        if(! s.empty())
        {
            std::stringstream ss;
            ss << s;

            MotionData data;
            double time;
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            Eigen::Vector3d gyro;
            Eigen::Vector3d acc;

            ss>>time;
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


            data.timestamp = time;
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
    std::ofstream save_points;
    save_points.open(filename.c_str());

    for (int i = 0; i < pose.size(); ++i) {
        MotionData data = pose[i];
        double time = data.timestamp;
        Eigen::Quaterniond q(data.Rwb);
        Eigen::Vector3d t = data.twb;
        Eigen::Vector3d gyro = data.imu_gyro;
        Eigen::Vector3d acc = data.imu_acc;

        save_points<<time<<" "
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