//
// Created by hyj on 17-6-22.
//
#include "param.h"
#include <opencv2/core/persistence.hpp>
#include <cxeigen.hpp>


Param::Param()
{
    Eigen::Matrix3d R;   // 把body坐标系朝向旋转一下,得到相机坐标系，好让它看到landmark,  相机坐标系的轴在body坐标系中的表示
    // 相机朝着轨迹里面看， 特征点在轨迹外部， 这里我们采用这个
    R << 0, 0, -1,
            -1, 0, 0,
            0, 1, 0;
    R_bc = R;
    t_bc = Eigen::Vector3d(0.05,0.04,0.03);

    R_bo = Eigen::Matrix3d::Identity();
    t_bc = Eigen::Vector3d(-0.0208, 0.0290, -0.0168);

}
void Param::readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        std::cout<<"config_file dosen't exist; wrong config_file path"<<std::endl;
        return;
    }
    fclose(fh);

    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    t_end = static_cast<double>(fs["t_end"]);
    imu_frequency = static_cast<int>(fs["imu_frequency"]);
    cam_frequency = static_cast<int>(fs["cam_frequency"]);
    wheel_frequency = static_cast<int>(fs["wheel_frequency"]);
    gyro_bias_sigma = static_cast<double>(fs["gyr_w"]);
    acc_bias_sigma = static_cast<double>(fs["acc_w"]);
    gyro_noise_sigma = static_cast<double>(fs["gyr_n"]);
    acc_noise_sigma = static_cast<double>(fs["acc_n"]);
    wheel_gyro_noise_sigma = static_cast<double>(fs["wheel_gyro_noise_sigma"]);
    wheel_velocity_noise_sigma = static_cast<double>(fs["wheel_velocity_noise_sigma"]);
    sx = static_cast<double>(fs["sx"]);
    sy = static_cast<double>(fs["sy"]);
    sw = static_cast<double>(fs["sw"]);
    sw_inv = 1.0 / sw;
    sv = Eigen::Vector3d(sx, sy, 1).asDiagonal();
    sv_inv = sv.inverse();
    pixel_noise = static_cast<double>(fs["pixel_noise"]);
    cv::FileNode n = fs["distortion_parameters"];
    fx = static_cast<double>(n["fx"]);
    fy = static_cast<double>(n["fy"]);
    cx = static_cast<double>(n["cx"]);
    cy = static_cast<double>(n["cy"]);

    td = static_cast<double>(fs["td"]);
    td_wheel = static_cast<double>(fs["td_wheel"]);
//    n = fs["projection_parameters"];
//    k1 = static_cast<double>(n["k1"]);
//    fy = static_cast<double>(n["k2"]);
//    cx = static_cast<double>(n["p1"]);
//    cy = static_cast<double>(n["p2"]);
    image_w = static_cast<int>(fs["image_w"]);
    image_h = static_cast<int>(fs["image_h"]);

    cv::Mat cv_R_bc;
    fs["body_R_cam"] >> cv_R_bc;
    cv::cv2eigen(cv_R_bc, R_bc);//包含头文件（顺序不能错！！！先包含eigen相关库，再包含opencv库！）
    cv::Mat cv_t_bc;
    fs["body_t_cam"] >> cv_t_bc;
    cv::cv2eigen(cv_t_bc, t_bc);
    cv::Mat cv_R_bo;
    fs["body_R_wheel"] >> cv_R_bo;
    cv::cv2eigen(cv_R_bo, R_bo);//包含头文件（顺序不能错！！！先包含eigen相关库，再包含opencv库！）
    cv::Mat cv_t_bo;
    fs["body_t_wheel"] >> cv_t_bo;
    cv::cv2eigen(cv_t_bo, t_bo);
    imu_timestep = 1./imu_frequency;
    cam_timestep = 1./cam_frequency;
    wheel_timestep = 1. /wheel_frequency;

    wheel_pose_path = "wheel_pose.txt";

    std::cout<<"PARAMETERS:"
             <<"\nimu_frequency: "<<imu_frequency
             <<"\ncam_frequency: "<<cam_frequency
             <<"\nwheel_frequency: "<<wheel_frequency
             <<"\ngyro_bias_sigma: "<<gyro_bias_sigma
             <<"\nacc_bias_sigma: "<<acc_bias_sigma
             <<"\ngyro_noise_sigma: "<<gyro_noise_sigma
             <<"\nacc_noise_sigma: "<<acc_noise_sigma
             <<"\nwheel_gyro_noise_sigma: "<<wheel_gyro_noise_sigma
             <<"\nwheel_velocity_noise_sigma: "<<wheel_velocity_noise_sigma
             <<"\nsx: "<<sx<<"  sy: "<<sy<<"  sw: "<<sw
             <<"\npixel_noise: "<<pixel_noise
             <<"\nfx: "<<fx<<"  fy: "<<fy<<"  cx: "<<cx<<"  cy: "<<cy
             <<"\nimage_w: "<<image_w<<"  image_h: "<<image_h
             <<"\nR_bc: \n"<<R_bc
             <<"\nt_bc: "<<t_bc.transpose()
             <<"\nR_bo: \n"<<R_bo
             <<"\nt_bo: "<<t_bo.transpose()
             <<"\nwheel_pose_path: "<<wheel_pose_path
             <<std::endl;

    fs.release();
}