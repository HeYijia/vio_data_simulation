//
// Created by hyj on 18-1-19.
//

#include "wheel.h"
#include "utilities.h"

//// euler2Rotation:   body frame to interitail frame
//Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles)
//{
//    double roll = eulerAngles(0);
//    double pitch = eulerAngles(1);
//    double yaw = eulerAngles(2);
//
//    double cr = cos(roll); double sr = sin(roll);
//    double cp = cos(pitch); double sp = sin(pitch);
//    double cy = cos(yaw); double sy = sin(yaw);
//
//    Eigen::Matrix3d RIb;
//    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
//            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
//            -sp,         cp*sr,           cp*cr;
//    return RIb;
//}

//Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
//{
//    double roll = eulerAngles(0);
//    double pitch = eulerAngles(1);
//
//    double cr = cos(roll); double sr = sin(roll);
//    double cp = cos(pitch); double sp = sin(pitch);
//
//    Eigen::Matrix3d R;
//    R<<  1,   0,    -sp,
//            0,   cr,   sr*cp,
//            0,   -sr,  cr*cp;
//
//    return R;
//}


Wheel::Wheel(Param p): param_(p)
{
//    gyro_bias_ = Eigen::Vector3d::Zero();
//    acc_bias_ = Eigen::Vector3d::Zero();
}

void Wheel::addWheelnoise(WheelMotionData& data)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d wheel_gyro_sqrt_cov = param_.wheel_gyro_noise_sigma * Eigen::Matrix3d::Identity();
    data.wheel_gyro = data.wheel_gyro + wheel_gyro_sqrt_cov * noise_gyro / sqrt(param_.wheel_timestep );
    // ridgeback的轮速输出roll和pitch的角速度始终为0
    data.wheel_gyro[0] = 0;
    data.wheel_gyro[1] = 0;

    Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d wheel_velocity_sqrt_cov = param_.wheel_velocity_noise_sigma * Eigen::Matrix3d::Identity();
    //ridgeback的轮速输出z轴速度始终为0
    data.wheel_velocity = data.wheel_velocity + wheel_velocity_sqrt_cov * noise_acc / sqrt( param_.wheel_timestep );
    data.wheel_velocity[2] = 0;
}

WheelMotionData Wheel::MotionModel(double t)
{

    WheelMotionData data;
    Eigen::Vector3d position;
    Eigen::Vector3d pb;
    Eigen::Vector3d wheel_gyro;
    Eigen::Vector3d imu_gyro;
    Eigen::Vector3d d_imu_gyro;
    Eigen::Vector3d dp;
    Eigen::Vector3d dp_b;
    Eigen::Vector3d ddp;
    Eigen::Vector3d ddp_b;
    Eigen::Matrix3d Rwo;
    Eigen::Matrix3d Rwb;


    motion_model(pb, Rwb, dp_b, ddp_b, imu_gyro, d_imu_gyro, t);

    position = Rwb * param_.t_bo + pb;
    Rwo = Rwb * param_.R_bo;
    dp = Rwb * skewSymmetric(imu_gyro) * param_.t_bo + dp_b;
    ddp =  Rwb * ( skewSymmetric(imu_gyro) *  skewSymmetric(imu_gyro)  + skewSymmetric(d_imu_gyro)) * param_.t_bo + ddp_b;
    wheel_gyro = param_.R_bo.transpose() * imu_gyro;


    data.wheel_gyro = param_.sw_inv * wheel_gyro;
    data.Rwo = Rwo;
    data.two = position;
    data.wheel_velocity = param_.sv_inv * Rwo.transpose() * dp;
    data.timestamp = t;
    return data;

}

//读取生成的wheel数据并用wheel动力学模型对数据进行计算，最后保存wheel积分以后的轨迹，
//用来验证数据以及模型的有效性。
void Wheel::testWheel(std::string src, std::string dist)
{
    std::vector<WheelMotionData>wheeldata;
    LoadPose(src,wheeldata);

    std::ofstream save_points;
    save_points.open(dist);
    double dt = param_.wheel_timestep;
    std::cout<<"time step = "<<dt<<std::endl;
    Eigen::Vector3d Pwo = init_two_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwo(init_Rwo_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = Qwo*init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    for (int i = 1; i < wheeldata.size(); ++i) {

        WheelMotionData wheelpose = wheeldata[i];
//        WheelMotionData imupose_next_k = imudata[i+1];

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half = param_.sw * wheelpose.wheel_gyro * dt / 2.0; //euler integrate
//        Eigen::Vector3d dtheta_half =  (imupose.imu_gyro + imupose_next_k.imu_gyro)/2 * dt /2.0; //mid-point integrate


        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();

        /// wheel 动力学模型 欧拉积分
        Eigen::Vector3d Vo = param_.sv * wheelpose.wheel_velocity;
        Vw = Qwo * Vo;
        Qwo = Qwo * dq;
        Pwo = Pwo + Vw * dt;


        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points << wheelpose.timestamp << " "
                   <<Qwo.w()<<" "
                   <<Qwo.x()<<" "
                   <<Qwo.y()<<" "
                   <<Qwo.z()<<" "
                   <<Pwo(0)<<" "
                   <<Pwo(1)<<" "
                   <<Pwo(2)<<" "
                   <<Qwo.w()<<" "
                   <<Qwo.x()<<" "
                   <<Qwo.y()<<" "
                   <<Qwo.z()<<" "
                   <<Pwo(0)<<" "
                   <<Pwo(1)<<" "
                   <<Pwo(2)<<" "
                   <<std::endl;

    }

}
//读取生成的wheel数据并用wheel动力学模型使用中值积分对数据进行计算，最后保存wheel积分以后的轨迹，
//用来验证数据以及模型的有效性。
void Wheel::testWheelMidPoint(std::string src, std::string dist)
{
    std::vector<WheelMotionData>wheeldata;
    LoadPose(src,wheeldata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.wheel_timestep;
    Eigen::Vector3d Pwo = init_two_;              // position :    from  wheel measurements
    Eigen::Quaterniond Qwo(init_Rwo_);            // quaterniond:  from wheel measurements
    Eigen::Quaterniond Qwo_last_k(init_Rwo_);
    Eigen::Vector3d Vw = Qwo*init_velocity_;          // velocity  :   from wheel measurements
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
//    std::cout<<"motion data initial pose: \n"<<Qwo.coeffs()<<std::endl<<Pwo<<std::endl;
    for (int i = 1; i < wheeldata.size(); ++i) {

        WheelMotionData wheelpose = wheeldata[i];
        WheelMotionData wheelpose_last_k = wheeldata[i-1];

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
//        Eigen::Vector3d dtheta_half =  wheelpose.wheel_gyro * dt /2.0; //euler integrate
        Eigen::Vector3d dtheta_half =  param_.sw * (wheelpose.wheel_gyro + wheelpose_last_k.wheel_gyro)/2 * dt /2.0; //mid-point integrate


        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();

        dq.normalize();

        /// 中值积分

        Qwo = Qwo * dq;
        Vw = 0.5 * (Qwo * param_.sv * wheelpose.wheel_velocity + Qwo_last_k * param_.sv * wheelpose_last_k.wheel_velocity);
        Pwo = Pwo + Vw * dt;
        Qwo_last_k = Qwo;

        //　按着wheel postion, wheel quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以wheel存了两次
        save_points<<wheelpose.timestamp<<" "
                   <<Qwo.w()<<" "
                   <<Qwo.x()<<" "
                   <<Qwo.y()<<" "
                   <<Qwo.z()<<" "
                   <<Pwo(0)<<" "
                   <<Pwo(1)<<" "
                   <<Pwo(2)<<" "
                   <<Qwo.w()<<" "
                   <<Qwo.x()<<" "
                   <<Qwo.y()<<" "
                   <<Qwo.z()<<" "
                   <<Pwo(0)<<" "
                   <<Pwo(1)<<" "
                   <<Pwo(2)<<" "
                   <<std::endl;

    }
}