/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "eskf.h"
#include "parameters.h"
#include "small_ivox.h"
#include <small_point_lio/pch.h>

namespace small_point_lio {

    class Estimator {
    public:
        // for common
        Parameters *parameters = nullptr;
        eskf kf;
        // for h_point
        std::shared_ptr<SmallIVox> ivox;
        Eigen::Matrix<state::value_type, 3, 1> Lidar_T_wrt_IMU;
        Eigen::Matrix<state::value_type, 3, 3> Lidar_R_wrt_IMU;
        Eigen::Vector3f point_lidar_frame;
        Eigen::Vector3f point_odom_frame;
        std::vector<Eigen::Vector3f> nearest_points;
        // for h_imu
        Eigen::Matrix<state::value_type, 3, 1> angular_velocity;
        Eigen::Matrix<state::value_type, 3, 1> linear_acceleration;
        double imu_acceleration_scale;
        
        // for h_point_batch
        std::vector<Eigen::Vector3f> batch_points_lidar_frame;  // Batch输入点（雷达坐标系）
        std::vector<double> batch_points_timestamps;             // 各点时间戳
        std::vector<Eigen::Vector3f> batch_points_undistorted;   // 去畸变后的点（投影到最后一帧）
        std::vector<Eigen::Vector3f> batch_points_odom_frame;    // 变换到odom坐标系的点

        Estimator();

        void reset();

        [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> process_noise_cov() const;

        void h_point(const state &s, point_measurement_result &measurement_result);

        void h_imu(const state &s, imu_measurement_result &measurement_result);
        
        void h_point_batch(const state &s, batch_point_measurement_result &measurement_result);
        
        // 点云去畸变：将Batch中每个点投影到最后一个点的时刻（参考公式3.44-3.47）
        Eigen::Vector3f undistort_point(const Eigen::Vector3f &point, 
                                         double point_time, 
                                         double last_time,
                                         const Eigen::Vector3f &velocity, 
                                         const Eigen::Vector3f &angular_velocity);
    };

}// namespace small_point_lio
