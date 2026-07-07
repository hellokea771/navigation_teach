/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <small_point_lio/pch.h>

namespace small_point_lio {

    class Parameters {
    public:
        int point_filter_num;
        float min_distance_squared;
        float max_distance_squared;
        Eigen::Vector3f base_link_to_lidar_xyz = Eigen::Vector3f::Zero();
        bool space_downsample;
        float space_downsample_leaf_size;

        Eigen::Vector3d gravity;
        bool check_satu;
        bool fix_gravity_direction;
        double satu_acc;
        double satu_gyro;
        double acc_norm;

        double map_resolution;
        size_t init_map_size;

        bool extrinsic_est_en;
        Eigen::Vector3d extrinsic_T;
        Eigen::Matrix3d extrinsic_R;

        double laser_point_cov;
        double imu_meas_acc_cov;
        double imu_meas_omg_cov;
        double velocity_cov;
        double omg_cov;
        double acceleration_cov;
        double bg_cov;
        double ba_cov;
        double plane_threshold;
        double match_sqaured;

        bool publish_odometry_without_downsample = false;
        
        // Batch点云更新相关参数
        bool use_batch_update = true;   // 是否启用Batch更新模式
        int batch_point_size = 50;      // Batch中的点数（一次性更新的点数）
        int batch_max_points = 500;     // Batch最大累积点数
        double batch_max_duration = 0.002;  // Batch最大时间跨度（秒）
        
        // 性能调试参数
        bool enable_performance_debug = false;  // 是否启用性能调试（打印各环节耗时）

        void read_parameters(rclcpp::Node &node);
    };

}// namespace small_point_lio
