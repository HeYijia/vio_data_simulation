//
// Created by hyj on 18-1-19.
//

#ifndef WheelSIMWITHPOINTLINE_Wheel_H
#define WheelSIMWITHPOINTLINE_Wheel_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>

#include "param.h"

struct WheelMotionData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Matrix3d Rwo;
    Eigen::Vector3d two;

    Eigen::Vector3d wheel_gyro;
    Eigen::Vector3d wheel_velocity;
};

//// euler2Rotation:   body frame to interitail frame
//Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles);
//Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles);


class Wheel
{
public:
    Wheel(Param p);
    Param param_;
//    Eigen::Vector3d gyro_bias_;
//    Eigen::Vector3d acc_bias_;

    Eigen::Vector3d init_velocity_;
    Eigen::Vector3d init_two_;
    Eigen::Matrix3d init_Rwo_;

    WheelMotionData MotionModel(double t);

    void addWheelnoise(WheelMotionData& data);
    void testWheel(std::string src, std::string dist);        // imu数据进行积分，用来看imu轨迹
    void testWheelMidPoint(std::string src, std::string dist); // imu数据进行中值积分，用来看imu轨迹

};

#endif //WheelSIMWITHPOINTLINE_Wheel_H
