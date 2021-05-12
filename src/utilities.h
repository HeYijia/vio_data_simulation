//
// Created by hyj on 18-1-19.
//

#ifndef IMUSIMWITHPOINTLINE_UTILITIES_H
#define IMUSIMWITHPOINTLINE_UTILITIES_H

#include "imu.h"
#include "wheel.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <fstream>
//设计运动轨迹
void motion_model(Eigen::Vector3d& position, Eigen::Matrix3d& Rwo, Eigen::Vector3d& velocity, Eigen::Vector3d& acc, Eigen::Vector3d& gyro, Eigen::Vector3d& dgyro, double t);

//反对称矩阵
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &q);

// save 3d points to file
void save_points(std::string filename, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points);

// save 3d points and it's obs in image
void save_features(std::string filename,
                   std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points,
                   std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features);
void save_features(std::string filename,
                   std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points,
                   std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features,
                   double time);

// save line obs
void save_lines(std::string filename,
                std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features);


void LoadPose(std::string filename, std::vector<MotionData>& pose);
void LoadPose(std::string filename, std::vector<WheelMotionData>& pose);
// save imu body data
void save_Pose(std::string filename, std::vector<MotionData> pose);
void save_Pose(std::string filename, std::vector<WheelMotionData> pose);
// save pose as TUM style
void save_Pose_asTUM(std::string filename, std::vector<MotionData> pose);
void save_Pose_asTUM(std::string filename, std::vector<WheelMotionData> pose);
#endif //IMUSIMWITHPOINTLINE_UTILITIES_H

