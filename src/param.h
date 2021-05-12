//
// Created by hyj on 17-6-22.
//

#ifndef IMUSIM_PARAM_H
#define IMUSIM_PARAM_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <iostream>
class Param{

public:

    Param();
    void readParameters(std::string config_file);
    // time
    int imu_frequency = 200;
    int cam_frequency = 30;
    int wheel_frequency = 50;
    double imu_timestep;
    double cam_timestep;
    double wheel_timestep;
    double t_start = 0.;
    double t_end = 20;  //  20 s

    // noise
    double gyro_bias_sigma = 1.0e-5;
    double acc_bias_sigma = 0.0001;

    double gyro_noise_sigma = 0.015;    // rad/s * 1/sqrt(hz)
    double acc_noise_sigma = 0.019;      //　m/(s^2) * 1/sqrt(hz)
    double wheel_gyro_noise_sigma = 0.015;    // rad/s
    double wheel_velocity_noise_sigma = 0.019;      //　m/(s^2)
    double sx = 1.0;
    double sy = 1.0;
    Eigen::Matrix3d sv;
    Eigen::Matrix3d sv_inv;
    double sw = 1.0;
    double sw_inv = 1.0;
    double pixel_noise = 1.5;              // 1 pixel noise

    //time delay
    double td = 0.0;
    double td_wheel = 0.0;

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
    Eigen::Matrix3d R_bo;   // wheel to body
    Eigen::Vector3d t_bo;     // wheel to body

    //path
    std::string wheel_pose_path;
};


#endif //IMUSIM_PARAM_H