/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "../common/common.h"
#include "estimator.h"
#include "parameters.h"
#include "preprocess.h"
#include <small_point_lio/pch.h>

namespace small_point_lio {

    class SmallPointLio {
    private:
        Parameters parameters;
        Preprocess preprocess;
        Estimator estimator;
        double time_current = 0.0;
        std::vector<Eigen::Vector3f> pointcloud_odom_frame;
        std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> pointcloud_callback;
        std::function<void(const common::Odometry &odometry)> odometry_callback;
        bool is_init = false;
        
        // Batch更新相关成员变量
        std::vector<common::Point> batch_points_buffer;  // Batch点云缓存
        int batch_point_count = 0;                        // 当前Batch中的点数

    public:
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> Q;

        explicit SmallPointLio(rclcpp::Node &node);

        void reset();

        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);

        void on_imu_callback(const common::ImuMsg &imu_msg);

        void handle_once();

        void set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback);

        void set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback);

    private:
        void publish_odometry(double timestamp);
        
        // Batch更新辅助函数
        void handle_once_original();  // 原始逐点更新逻辑
        void handle_once_batch();     // Batch更新逻辑
        void flush_batch_points();    // 提交当前Batch
    };

}// namespace small_point_lio
