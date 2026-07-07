/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "lidar_adapter/base_lidar.h"
#include "small_point_lio/small_point_lio.h"
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_point_lio/pch.h>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.h>

namespace small_point_lio {

    class SmallPointLioNode : public rclcpp::Node {
    private:
        std::unique_ptr<small_point_lio::SmallPointLio> small_point_lio;
        std::vector<common::Point> pointcloud;
        std::vector<Eigen::Vector3f> pointcloud_to_save;
        std::unique_ptr<LidarAdapterBase> lidar_adapter;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subsciber;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger;
        common::Odometry last_odometry;

        // 缓存的静态外参变换（只在启动时计算一次）
        bool extrinsic_valid_{false};
        bool use_dynamic_lidar_extrinsic_{false};
        std::string lidar_frame_;
        Eigen::Isometry3f T_base_to_lidar_{Eigen::Isometry3f::Identity()};  // base_link -> lidar_frame
        Eigen::Isometry3f T_lidar_to_base_{Eigen::Isometry3f::Identity()};  // 点云变换用
        tf2::Transform tf_base_link_to_lidar_;  // TF 广播用
        rclcpp::TimerBase::SharedPtr extrinsic_init_timer_;  // 外参初始化定时器

        // 重力对齐：将lidar_odom系旋转至Z轴竖直向上的odom系
        bool gravity_alignment_enabled_{false};  // 是否启用重力对齐
        bool gravity_alignment_done_{false};     // 重力对齐是否已完成
        bool gravity_alignment_debug_{false};    // 是否输出调试信息
        Eigen::Quaternionf q_gravity_align_{Eigen::Quaternionf::Identity()};  // lidar_odom → odom 的纯旋转
        std::vector<Eigen::Vector3d> imu_accel_buffer_;  // 用于收集IMU加速度数据
        static constexpr size_t GRAVITY_INIT_SAMPLES = 200;  // 用于初始化重力的IMU样本数

        // 速度估计：线速度用位置差分+低通滤波，角速度用刚体变换
        bool velocity_initialized_{false};
        Eigen::Vector3f prev_position_{Eigen::Vector3f::Zero()};
        Eigen::Quaternionf prev_orientation_{Eigen::Quaternionf::Identity()};
        double prev_timestamp_{0.0};
        Eigen::Vector3f filtered_linear_velocity_{Eigen::Vector3f::Zero()};
        static constexpr float VELOCITY_FILTER_ALPHA = 0.3f;  // 低通滤波系数 (0-1, 越小越平滑)

        // 计算重力对齐变换
        void computeGravityAlignment(const Eigen::Vector3d& measured_gravity);
        void cacheExtrinsicTransform(const geometry_msgs::msg::TransformStamped& transform);
        bool updateDynamicExtrinsic(const builtin_interfaces::msg::Time& stamp);

    public:
        explicit SmallPointLioNode(const rclcpp::NodeOptions &options);
    };

}// namespace small_point_lio
