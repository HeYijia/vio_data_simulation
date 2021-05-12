//
// Created by hyj on 18-1-19.
//

#include "imu.h"
#include "utilities.h"

IMU::IMU(Param p): param_(p)
{
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();
}

void IMU::addIMUnoise(MotionData& data)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt( param_.imu_timestep ) + gyro_bias_;

    Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt( param_.imu_timestep ) + acc_bias_;

    // gyro_bias update
    Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
    gyro_bias_ += param_.gyro_bias_sigma * sqrt(param_.imu_timestep ) * noise_gyro_bias;
    data.imu_gyro_bias = gyro_bias_;

    // acc_bias update
    Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
    acc_bias_ += param_.acc_bias_sigma * sqrt(param_.imu_timestep ) * noise_acc_bias;
    data.imu_acc_bias = acc_bias_;

}

MotionData IMU::MotionModel(double t)
{

    MotionData data;

    Eigen::Vector3d position;
    Eigen::Vector3d imu_gyro;
    Eigen::Vector3d d_imu_gyro;
    Eigen::Vector3d dp;
    Eigen::Vector3d ddp;
    Eigen::Matrix3d Rwb;

    motion_model(position, Rwb, dp, ddp, imu_gyro,d_imu_gyro, t);

    Eigen::Vector3d gn (0,0,-9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t;
    return data;

}

//读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testImu(std::string src, std::string dist)
{
    std::vector<MotionData>imudata;
    LoadPose(src,imudata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    for (int i = 1; i < imudata.size(); ++i) {

        MotionData imupose = imudata[i];
//        MotionData imupose_next_k = imudata[i+1];

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half =  imupose.imu_gyro * dt /2.0; //euler integrate
//        Eigen::Vector3d dtheta_half =  (imupose.imu_gyro + imupose_next_k.imu_gyro)/2 * dt /2.0; //mid-point integrate


        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();

        /// imu 动力学模型 欧拉积分
        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
        Qwb = Qwb * dq;
        Vw = Vw + acc_w * dt;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;

        /// 中值积分
//        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
//        Qwb = Qwb * dq;
//        Eigen::Vector3d acc_w_next_k = Qwb * (imupose_next_k.imu_acc) + gw;
//        Eigen::Vector3d acc_w_mid_point = (acc_w + acc_w_next_k)/2;
//        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w_mid_point;
//        Vw = Vw + acc_w_mid_point * dt;


        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points<<imupose.timestamp<<" "
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

}
//读取生成的imu数据并用imu动力学模型使用中值积分对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testImuMidPoint(std::string src, std::string dist)
{
    std::vector<MotionData>imudata;
    LoadPose(src,imudata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    for (int i = 1; i < imudata.size(); ++i) {

        MotionData imupose = imudata[i];
        MotionData imupose_last_k = imudata[i-1];

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
//        Eigen::Vector3d dtheta_half =  imupose.imu_gyro * dt /2.0; //euler integrate
        Eigen::Vector3d dtheta_half =  (imupose.imu_gyro + imupose_last_k.imu_gyro)/2 * dt /2.0; //mid-point integrate


        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();

        dq.normalize();
        /// imu 动力学模型 欧拉积分
//        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
//        Qwb = Qwb * dq;
//        Vw = Vw + acc_w * dt;
//        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;

        /// 中值积分
        Eigen::Vector3d acc_w_last_k = Qwb * (imupose_last_k.imu_acc) + gw;
        Qwb = Qwb * dq;
        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
        Eigen::Vector3d acc_w_mid_point = (acc_w + acc_w_last_k)/2;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w_mid_point;
        Vw = Vw + acc_w_mid_point * dt;


        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points<<imupose.timestamp<<" "
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
}
