//
// Created by hyj on 18-1-19.
//
#include <iomanip>
#include "utilities.h"
/** 6自由度运动 **/
//void motion_model(Eigen::Vector3d& position, Eigen::Matrix3d& Rwb, Eigen::Vector3d& velocity, Eigen::Vector3d& acc, Eigen::Vector3d& gyro, double t)
//{
//
//    MotionData data;
//    // param
//    float ellipse_x = 15;
//    float ellipse_y = 20;
//    float z = 1;           // z轴做sin运动
//    float K1 = 10;          // z轴的正弦频率是x，y的k1倍
//    float K = M_PI/ 10;    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周
//
//    // translation
//    // twb:  body frame in world frame
//    position = Eigen::Vector3d( ellipse_x * cos( K * t) + 5, ellipse_y * sin( K * t) + 5,  z * sin( K1 * K * t ) + 5);
//    velocity = Eigen::Vector3d(- K * ellipse_x * sin(K*t),  K * ellipse_y * cos(K*t), z*K1*K * cos(K1 * K * t));              // position导数　in world frame
//    double K2 = K*K;
//    acc = Eigen::Vector3d( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), -z*K1*K1*K2 * sin(K1 * K * t));     // position二阶导数
//
//    // Rotation
//    double k_roll = 0.1;
//    double k_pitch = 0.2;
//    Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
//    Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数
//
////    Eigen::Vector3d eulerAngles(0.0,0.0, K*t );   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
////    Eigen::Vector3d eulerAnglesRates(0.,0. , K);      // euler angles 的导数
//
//    Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
//    gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro
//
//}
/** 平面运动 **/
void motion_model(Eigen::Vector3d& position, Eigen::Matrix3d& Rwo, Eigen::Vector3d& velocity, Eigen::Vector3d& acc, Eigen::Vector3d& gyro,  Eigen::Vector3d& dgyro, double t){
    // param
    float ellipse_x = 15;
    float ellipse_y = 20;
    float z = 1;           // 沿椭圆圆心的sin运动振荡
    float K1 = 10;          // z轴的正弦频率是x，y的k1倍
    float K = M_PI/ 10;    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

    // translation
    // twb:  body frame in world frame
    double ox = 5;
    double oy = 5;
    double x0 = ellipse_x * cos( K * t);
    double x1 = x0 + ox;
    double y0 = ellipse_y * sin( K * t);
    double y1 = y0 + oy;
//    double z0 = z * sin( K1 * K * t ) + 5;
    double z0 = 5;
    double offset = z * sin( K1 * K * t );
    double doff = z*K1*K * cos(K1 * K * t);
    double l_2 = x0*x0 + y0*y0;
    double l = sqrt(l_2);
    double l_3_2 = l_2 * l;
    position = Eigen::Vector3d( x1 + x0/l*offset, y1+y0/l*offset, z0 );

    double dx0 = - K * ellipse_x * sin(K*t);
    double K2 = K*K;
    double ddx0 = -K2 * ellipse_x * cos(K*t);
    double dy0 = K * ellipse_y * cos(K*t);
    double ddy0 = -K2 * ellipse_y * sin(K*t);
    double dz0 = 0;
    double dl = 1/l*(x0*dx0+y0*dy0);
    velocity = Eigen::Vector3d (dx0*(1+offset/l)+x0*z*K1*K*cos(K1*K*t)/l-x0*offset*(x0*dx0+y0*dy0)/l_3_2,  dy0*(1+offset/l)+y0*z*K1*K*cos(K1*K*t)/l-y0*offset*(x0*dx0+y0*dy0)/l_3_2, 0);              // position导数　in world frame

    acc = Eigen::Vector3d(ddx0*(1+offset/l)+(dx0*doff*l-2*dx0*offset*dl-2*x0*doff*dl-x0*K1*K1*K2*offset*l)/l_2+z*dx0*K1*K*cos(K1*K*t)/l-3*x0*offset*(x0*dx0+y0*dy0)*dl/pow(l_2,3)-x0*offset*(dx0*dx0+x0*ddx0+dy0*dy0+y0*ddy0)/l_3_2
            ,ddy0*(1+offset/l)+(dy0*doff*l-2*dy0*offset*dl-2*y0*doff*dl-y0*K1*K1*K2*offset*l)/l_2+z*dy0*K1*K*cos(K1*K*t)/l-3*y0*offset*(x0*dx0+y0*dy0)*dl/pow(l_2,3)-y0*offset*(dx0*dx0+x0*ddx0+dy0*dy0+y0*ddy0)/l_3_2
            ,0);

    // 充分旋转
//    Eigen::Vector3d eulerAngles(0.0,0.0, K*t +  0.79 * sin(K1 * K * t));   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
//    Eigen::Vector3d eulerAnglesRates(0.,0. , K + 0.79 * K1 * K* cos(K1 * K * t));      // euler angles 的导数
//    dgyro = Eigen::Vector3d (0.,0. , -0.79 * K1 * K * K1 * K * sin(K1 * K * t));      // euler angles 的导数

    // 普通旋转
    Eigen::Vector3d eulerAngles(0.0,0.0, K*t);   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
    Eigen::Vector3d eulerAnglesRates(0.,0. , K );      // euler angles 的导数
    dgyro = Eigen::Vector3d (0.,0. , 0);      // euler angles 的导数

    Rwo = euler2Rotation(eulerAngles);         // body frame to world frame
    gyro = eulerAnglesRates;   //  euler rates trans to body gyro

}
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &q)
{
    Eigen::Matrix3d ans;
    ans << 0.0, -q(2), q(1),
            q(2), 0.0, -q(0),
            -q(1), q(0), 0.0;
    return ans;
}
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
                   <<f(1)<<" ";
        save_points.setf(std::ios::fixed, std::ios::floatfield);
        save_points<<std::setprecision(6)
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
        std::cerr << " can't open "<<filename<<" file "<<std::endl;
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
void LoadPose(std::string filename, std::vector<WheelMotionData>& pose)
{

    std::ifstream f;
    f.open(filename.c_str());

    if(!f.is_open())
    {
        std::cerr << " can't open "<<filename<<" file "<<std::endl;
        return;
    }

    while (!f.eof()) {

        std::string s;
        std::getline(f,s);

        if(! s.empty())
        {
            std::stringstream ss;
            ss << s;

            WheelMotionData data;
            double time;
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            Eigen::Vector3d wheelGyro;
            Eigen::Vector3d wheelVelocity;

            ss>>time;
            ss>>q.w();
            ss>>q.x();
            ss>>q.y();
            ss>>q.z();
            ss>>t(0);
            ss>>t(1);
            ss>>t(2);
            ss >> wheelGyro(0);
            ss >> wheelGyro(1);
            ss >> wheelGyro(2);
            ss >> wheelVelocity(0);
            ss >> wheelVelocity(1);
            ss >> wheelVelocity(2);


            data.timestamp = time;
            data.wheel_gyro = wheelGyro;
            data.wheel_velocity = wheelVelocity;
            data.two = t;
            data.Rwo = Eigen::Matrix3d(q);
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
void save_Pose(std::string filename, std::vector<WheelMotionData> pose)
{
    std::ofstream save_points;
    save_points.open(filename.c_str());

    for (int i = 0; i < pose.size(); ++i) {
        WheelMotionData data = pose[i];
        double time = data.timestamp;
        Eigen::Quaterniond q(data.Rwo);
        Eigen::Vector3d t = data.two;
        Eigen::Vector3d wheelGyro = data.wheel_gyro;
        Eigen::Vector3d wheelVelocity = data.wheel_velocity;

        save_points << time << " "
                    << q.w() << " "
                    << q.x() << " "
                    << q.y() << " "
                    << q.z() << " "
                    << t(0) << " "
                    << t(1) << " "
                    << t(2) << " "
                    << wheelGyro(0) << " "
                    << wheelGyro(1) << " "
                    << wheelGyro(2) << " "
                    << wheelVelocity(0) << " "
                    << wheelVelocity(1) << " "
                    << wheelVelocity(2) << " "
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

        save_points<<std::setprecision(6);
        save_points <<time<<" ";
        save_points<<std::setprecision(9);
        save_points <<t(0)<<" "
                    <<t(1)<<" "
                    <<t(2)<<" "
                    <<q.x()<<" "
                    <<q.y()<<" "
                    <<q.z()<<" "
                    <<q.w() <<std::endl;
    }

}
void save_Pose_asTUM(std::string filename, std::vector<WheelMotionData> pose)
{
    std::ofstream save_points;
    save_points.setf(std::ios::fixed, std::ios::floatfield);
    save_points.open(filename.c_str());

    for (int i = 0; i < pose.size(); ++i) {
        WheelMotionData data = pose[i];
        double time = data.timestamp;
        Eigen::Quaterniond q(data.Rwo);
        Eigen::Vector3d t = data.two;
//        Eigen::Vector3d gyro = data.imu_gyro;
//        Eigen::Vector3d acc = data.imu_acc;

        save_points<<std::setprecision(6);
        save_points <<time<<" ";
        save_points<<std::setprecision(9);
        save_points <<t(0)<<" "
                    <<t(1)<<" "
                    <<t(2)<<" "
                    <<q.x()<<" "
                    <<q.y()<<" "
                    <<q.z()<<" "
                    <<q.w() <<std::endl;
    }

}
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
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,//ZYX
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