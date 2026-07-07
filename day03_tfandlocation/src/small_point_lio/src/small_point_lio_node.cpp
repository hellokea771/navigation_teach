/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio_node.hpp"
#include "lidar_adapter/livox_lidar.h"
#include "lidar_adapter/unitree_lidar.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace small_point_lio {

    void SmallPointLioNode::cacheExtrinsicTransform(const geometry_msgs::msg::TransformStamped& transform) {
        // lookupTransform(lidar_frame, "base_link", t) 返回的是 T_lidar_base。
        // 保持原变量名和后续计算兼容：T_base_to_lidar_ 实际缓存 T_lidar_base。
        T_base_to_lidar_ = Eigen::Isometry3f::Identity();
        T_base_to_lidar_.translation() <<
                static_cast<float>(transform.transform.translation.x),
                static_cast<float>(transform.transform.translation.y),
                static_cast<float>(transform.transform.translation.z);
        T_base_to_lidar_.linear() = Eigen::Quaternionf(
                static_cast<float>(transform.transform.rotation.w),
                static_cast<float>(transform.transform.rotation.x),
                static_cast<float>(transform.transform.rotation.y),
                static_cast<float>(transform.transform.rotation.z))
                .toRotationMatrix();

        T_lidar_to_base_ = T_base_to_lidar_.inverse();
        tf2::fromMsg(transform.transform, tf_base_link_to_lidar_);
    }

    bool SmallPointLioNode::updateDynamicExtrinsic(const builtin_interfaces::msg::Time& stamp) {
        try {
            const auto transform = tf_buffer->lookupTransform(
                    lidar_frame_, "base_link", rclcpp::Time(stamp));
            cacheExtrinsicTransform(transform);
            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Dynamic extrinsic TF %s -> base_link at stamp %d.%09u unavailable: %s",
                lidar_frame_.c_str(), stamp.sec, stamp.nanosec, ex.what());
            return false;
        }
    }

    SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions &options)
        : Node("small_point_lio", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        std::string imu_topic = declare_parameter<std::string>("imu_topic");
        std::string lidar_type = declare_parameter<std::string>("lidar_type");
        std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
        lidar_frame_ = lidar_frame;
        std::string lidar_mount_mode = declare_parameter<std::string>("lidar_mount_mode", "fixed");
        if (lidar_mount_mode == "gimbal_yaw") {
            use_dynamic_lidar_extrinsic_ = true;
        } else if (lidar_mount_mode != "fixed") {
            RCLCPP_WARN(get_logger(),
                "Unknown lidar_mount_mode '%s', falling back to fixed",
                lidar_mount_mode.c_str());
        }
        bool save_pcd = declare_parameter<bool>("save_pcd");
        gravity_alignment_enabled_ = declare_parameter<bool>("gravity_alignment", true);
        gravity_alignment_debug_ = declare_parameter<bool>("gravity_alignment_debug", false);
        RCLCPP_INFO(get_logger(), "LiDAR mount mode: %s", use_dynamic_lidar_extrinsic_ ? "gimbal_yaw" : "fixed");
        
        RCLCPP_INFO(get_logger(), "Gravity alignment enabled: %s", gravity_alignment_enabled_ ? "true" : "false");
        if (gravity_alignment_debug_) {
            RCLCPP_INFO(get_logger(), "Gravity alignment debug output: enabled");
        }
        
        // 如果不启用重力对齐，直接标记为完成（使用单位四元数）
        if (!gravity_alignment_enabled_) {
            gravity_alignment_done_ = true;
            RCLCPP_INFO(get_logger(), "Gravity alignment disabled, using identity transform");
        }
        
        small_point_lio = std::make_unique<small_point_lio::SmallPointLio>(*this);
        odometry_publisher = create_publisher<nav_msgs::msg::Odometry>("/Odometry", 1000);
        pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 1000);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // 创建定时器，在 TF 可用后初始化静态外参缓存
        extrinsic_init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this, lidar_frame]() {
                if (extrinsic_valid_) return;  // 已初始化则跳过

                static int retry_count = 0;  // 静态局部变量，只在此 lambda 内使用

                try {
                    auto transform = tf_buffer->lookupTransform(
                        lidar_frame, "base_link", tf2::TimePointZero);

                    cacheExtrinsicTransform(transform);

                    extrinsic_valid_ = true;
                    if (use_dynamic_lidar_extrinsic_) {
                        RCLCPP_INFO(get_logger(),
                            "Dynamic extrinsic TF chain is available: base_link -> ... -> %s",
                            lidar_frame.c_str());
                    } else {
                        RCLCPP_INFO(get_logger(), "Extrinsic calibration cached: base_link -> %s", lidar_frame.c_str());
                    }
                } catch (tf2::TransformException &ex) {
                    retry_count++;
                    if (retry_count >= 10) {
                        // 连续10次失败，默认两坐标系重合（单位变换）
                        T_base_to_lidar_ = Eigen::Isometry3f::Identity();
                        T_lidar_to_base_ = Eigen::Isometry3f::Identity();
                        tf_base_link_to_lidar_.setIdentity();

                        extrinsic_valid_ = true;
                        RCLCPP_WARN(get_logger(),
                            "Extrinsic TF base_link -> %s not found after %d retries, assuming identity transform",
                            lidar_frame.c_str(), retry_count);
                    } else {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Waiting for extrinsic TF base_link -> %s (%d/10): %s",
                            lidar_frame.c_str(), retry_count, ex.what());
                    }
                }
            });

        map_save_trigger = create_service<std_srvs::srv::Trigger>(
                "map_save",
                [this, save_pcd, lidar_frame](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
                    if (!save_pcd) {
                        RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "pcd save is disabled");
                        return;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "waiting for pcd saving ...");
                    auto pointcloud_to_save_copy = std::make_shared<std::vector<Eigen::Vector3f>>(pointcloud_to_save);
                    std::thread([this, pointcloud_to_save_copy, lidar_frame]() {
                        voxelgrid_sampling::VoxelgridSampling downsampler;
                        std::vector<Eigen::Vector3f> downsampled;
                        downsampler.voxelgrid_sampling_omp(*pointcloud_to_save_copy, downsampled, 0.02);
                        pcl::PointCloud<pcl::PointXYZI> pcl_pointcloud;
                        pcl_pointcloud.header.frame_id = lidar_frame;
                        pcl_pointcloud.header.stamp = static_cast<uint64_t>(last_odometry.timestamp * 1e6);
                        pcl_pointcloud.points.reserve(downsampled.size());
                        for (const auto &point: downsampled) {
                            pcl::PointXYZI new_point;
                            new_point.x = point.x();
                            new_point.y = point.y();
                            new_point.z = point.z();
                            pcl_pointcloud.points.push_back(new_point);
                        }
                        pcl_pointcloud.width = pcl_pointcloud.points.size();
                        pcl_pointcloud.height = 1;
                        pcl_pointcloud.is_dense = true;
                        pcl::PCDWriter writer;
                        writer.writeBinary(ROOT_DIR + "/pcd/scan.pcd", pcl_pointcloud);
                        RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "save pcd success");
                    }).detach();
                });
        small_point_lio->set_odometry_callback([this, lidar_frame](const common::Odometry &odometry) {
            if (!extrinsic_valid_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Extrinsic not ready, skipping odometry callback");
                return;
            }
            
            // 如果启用重力对齐但尚未完成，跳过发布
            if (gravity_alignment_enabled_ && !gravity_alignment_done_) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Waiting for gravity alignment initialization...");
                return;
            }
            
            last_odometry = odometry;

            builtin_interfaces::msg::Time time_msg;
            time_msg.sec = std::floor(odometry.timestamp);
            time_msg.nanosec = static_cast<uint32_t>((odometry.timestamp - time_msg.sec) * 1e9);

            if (use_dynamic_lidar_extrinsic_ && !updateDynamicExtrinsic(time_msg)) {
                return;
            }

            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = time_msg;
            transform_stamped.header.frame_id = "odom";
            transform_stamped.child_frame_id = "base_link";

            // ============================================================
            // 变换推导：
            // 
            // 两种模式：
            // 
            // 模式1：重力对齐启用 (gravity_alignment: true)
            //   - LIO输出: T_lidar_odom→lidar (lidar在lidar_odom系下的位姿)
            //   - 重力对齐: q_gravity_align (将lidar_odom系的Z轴旋转到指向天)
            //   - odom = 重力对齐后的坐标系（Z轴竖直向上）
            //   - 公式：q_base_in_odom = q_gravity_align * q_lidar_odom * q_base_to_lidar
            //
            // 模式2：重力对齐禁用 (gravity_alignment: false)
            //   - odom = lidar_odom（直接使用LIO输出的坐标系）
            //   - 需要应用相似变换将 lidar 位姿转换到 base_link
            //   - 公式：T_odom→base = T_lidar→base * T_odom→lidar * T_base→lidar
            //          (相似变换，考虑外参的旋转中心)
            // ============================================================
            
            // LIO输出的位姿（内部 odom -> lidar/IMU body）
            Eigen::Vector3f pos_lidar_odom = odometry.position.cast<float>();
            Eigen::Quaternionf q_lidar_odom(odometry.orientation.cast<float>());
            
            Eigen::Vector3f pos_base_in_odom;
            Eigen::Quaternionf q_base_in_odom;
            
            if (gravity_alignment_enabled_) {
                // ============================================================
                // 模式1：重力对齐模式
                // ============================================================

                // 使用完整 SE(3) 链乘，避免手动拆分旋转和平移时把外参方向用反。
                // lookupTransform(lidar_frame, "base_link") 缓存的是 T_lidar_base，
                // 因此 T_odom_base = T_align * T_lio_odom_lidar * T_lidar_base。
                Eigen::Isometry3f T_align = Eigen::Isometry3f::Identity();
                T_align.linear() = q_gravity_align_.toRotationMatrix();

                Eigen::Isometry3f T_lio_odom_lidar = Eigen::Isometry3f::Identity();
                T_lio_odom_lidar.translation() = pos_lidar_odom;
                T_lio_odom_lidar.linear() = q_lidar_odom.normalized().toRotationMatrix();

                Eigen::Isometry3f T_odom_base = T_align * T_lio_odom_lidar * T_base_to_lidar_;
                pos_base_in_odom = T_odom_base.translation();
                q_base_in_odom = Eigen::Quaternionf(T_odom_base.rotation()).normalized();
                
            } else {
                // ============================================================
                // 模式2：无重力对齐，使用相似变换
                // ============================================================
                
                // 相似变换公式：T_new = T_lidar_to_base * T_old * T_base_to_lidar
                // 
                // 物理意义：把"lidar的运动"转换成"base_link的运动"
                // - T_base_to_lidar: 先把世界坐标系变换到base_link的视角
                // - T_old: lidar的运动
                // - T_lidar_to_base: 再变换回世界坐标系，但现在描述的是base_link的运动
                
                // 构建 tf2::Transform 进行相似变换
                tf2::Transform tf_lidar_odom_to_lidar;
                tf_lidar_odom_to_lidar.setOrigin(tf2::Vector3(pos_lidar_odom.x(), pos_lidar_odom.y(), pos_lidar_odom.z()));
                tf_lidar_odom_to_lidar.setRotation(tf2::Quaternion(q_lidar_odom.x(), q_lidar_odom.y(), q_lidar_odom.z(), q_lidar_odom.w()));
                
                // 外参变换（Eigen::Isometry3f 转换为 tf2::Transform）
                tf2::Transform tf_lidar_to_base;
                tf_lidar_to_base.setOrigin(tf2::Vector3(
                    T_lidar_to_base_.translation().x(),
                    T_lidar_to_base_.translation().y(),
                    T_lidar_to_base_.translation().z()));
                Eigen::Quaternionf q_lidar_to_base_eigen(T_lidar_to_base_.rotation());
                tf_lidar_to_base.setRotation(tf2::Quaternion(
                    q_lidar_to_base_eigen.x(),
                    q_lidar_to_base_eigen.y(),
                    q_lidar_to_base_eigen.z(),
                    q_lidar_to_base_eigen.w()));
                
                tf2::Transform tf_base_to_lidar = tf_lidar_to_base.inverse();
                
                // 相似变换
                tf2::Transform tf_odom_to_base = tf_lidar_to_base * tf_lidar_odom_to_lidar * tf_base_to_lidar;
                
                // 提取结果
                auto origin = tf_odom_to_base.getOrigin();
                pos_base_in_odom = Eigen::Vector3f(origin.x(), origin.y(), origin.z());
                
                auto rotation = tf_odom_to_base.getRotation();
                q_base_in_odom = Eigen::Quaternionf(rotation.w(), rotation.x(), rotation.y(), rotation.z());
            }
            
            tf2::Transform tf_odom_to_base_final;
            tf_odom_to_base_final.setOrigin(tf2::Vector3(pos_base_in_odom.x(), pos_base_in_odom.y(), pos_base_in_odom.z()));
            tf_odom_to_base_final.setRotation(tf2::Quaternion(q_base_in_odom.x(), q_base_in_odom.y(), q_base_in_odom.z(), q_base_in_odom.w()));
            
            // DEBUG: 输出各个变换的四元数（可通过 gravity_alignment_debug 参数控制）
            if (gravity_alignment_debug_) {
                static int debug_counter = 0;
                if (debug_counter++ % 100 == 0) {
                    RCLCPP_INFO(get_logger(), "DEBUG transforms (mode: %s):",
                        gravity_alignment_enabled_ ? "gravity_aligned" : "similarity_transform");
                    RCLCPP_INFO(get_logger(), "  q_lidar_odom (LIO output): [w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
                        q_lidar_odom.w(), q_lidar_odom.x(), q_lidar_odom.y(), q_lidar_odom.z());
                    if (gravity_alignment_enabled_) {
                        RCLCPP_INFO(get_logger(), "  q_gravity_align: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
                            q_gravity_align_.w(), q_gravity_align_.x(), q_gravity_align_.y(), q_gravity_align_.z());
                    }
                    RCLCPP_INFO(get_logger(), "  q_base_in_odom (final): [w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
                        q_base_in_odom.w(), q_base_in_odom.x(), q_base_in_odom.y(), q_base_in_odom.z());
                }
            }
            
            transform_stamped.transform = tf2::toMsg(tf_odom_to_base_final);

            nav_msgs::msg::Odometry odometry_msg;
            odometry_msg.header.stamp = time_msg;
            odometry_msg.header.frame_id = "odom";
            odometry_msg.child_frame_id = "base_link";
            odometry_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
            odometry_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
            odometry_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
            odometry_msg.pose.pose.orientation.x = transform_stamped.transform.rotation.x;
            odometry_msg.pose.pose.orientation.y = transform_stamped.transform.rotation.y;
            odometry_msg.pose.pose.orientation.z = transform_stamped.transform.rotation.z;
            odometry_msg.pose.pose.orientation.w = transform_stamped.transform.rotation.w;

            // slh: ============================================================
            // 填充 twist（速度信息），按照 ROS 约定在 child_frame_id (base_link) 系下表示
            //
            // 我们先约定：
            // W = imu_odom（LIO 的 world / imu 第一帧）
            // I = imu_link（IMU 机体）
            // B = base_link（车体中心，ROS 惯例中的 child）
            // 在机器人学中，T_A_to_B 这种命名存在两种相反的解读习惯，我们尽量采用第一种
            //  T_A_to_B = 把点从A系变到B系的变换矩阵 使用方式：p_B = T_A_to_B * p_A （更常见，Eigen/ROS惯例）
            // 所以：T_lidar_to_base_ 用于p_base = T_lidar_to_base_ * p_lidar
            //
            // LIO 输出:
            //   - odometry.velocity: v_W_I （world系下IMU的速度，经验证不准确）
            //   - odometry.angular_velocity: ω_I （IMU body系下的角速度）
            // 
            // 目标：生成 ROS Odometry 的 twist（线速度 linear.x,y,z 与角速度 angular.x,y,z）
            // 通常按惯例把 twist 表示在 child frame（base_link）下
            // 同时 header.frame_id=odom（或imu_odom），child_frame_id=base_link
            //
            // 注意：LIO输出的velocity不准确（尤其在旋转时），因此：
            //   - 角速度：使用刚体变换（ω_B = R_IB * ω_I）
            //   - 线速度：改用位置差分法（v = Δp / Δt）
            // ============================================================
            
            // LIO 输出的原始数据
            Eigen::Vector3f omg_lidar = odometry.angular_velocity.cast<float>(); // lidar body 系下的角速度

            // 外参: lidar → base_link
            // T_lidar_to_base_ 用于 p_base = T_lidar_to_base_ * p_lidar
            // 其旋转部分的列向量是 lidar 系的基向量在 base 系中的坐标
            Eigen::Matrix3f R_lidar_to_base = T_lidar_to_base_.rotation();

            // ============================================================
            // 角速度估计：使用刚体变换
            // 刚体角速度是绑定向量，坐标变换公式：ω_B = R_IB * ω_I
            // ============================================================
            Eigen::Vector3f angular_vel_base = R_lidar_to_base * omg_lidar;

            // DEBUG: 定期打印角速度变换
            // static int omg_debug_counter = 0;
            // if (omg_debug_counter++ % 50 == 0) {
            //     RCLCPP_INFO(get_logger(), "[OMG_DEBUG] omg_lidar (raw): [%.4f, %.4f, %.4f] rad/s",
            //         omg_lidar.x(), omg_lidar.y(), omg_lidar.z());
            //     RCLCPP_INFO(get_logger(), "[OMG_DEBUG] omg_base (transformed): [%.4f, %.4f, %.4f] rad/s",
            //         angular_vel_base.x(), angular_vel_base.y(), angular_vel_base.z());
            //     RCLCPP_INFO(get_logger(), "[OMG_DEBUG] |omg_lidar| = %.4f, |omg_base| = %.4f (should be equal)",
            //         omg_lidar.norm(), angular_vel_base.norm());
            // }

            // ============================================================
            // 线速度估计：使用位置差分+低通滤波
            // 原因：LIO的velocity输出在旋转时不准确，改用数值微分
            // 方法：v = (p_current - p_previous) / dt，然后低通滤波平滑
            // ============================================================
            Eigen::Vector3f linear_vel_base = Eigen::Vector3f::Zero();
            
            if (velocity_initialized_) {
                double dt = odometry.timestamp - prev_timestamp_;
                
                if (dt > 0.001 && dt < 0.5) {  // 有效时间间隔：1ms ~ 500ms
                    // 位置差分（在odom系下）
                    Eigen::Vector3f pos_diff_odom = pos_base_in_odom - prev_position_;
                    Eigen::Vector3f instant_linear_vel_odom = pos_diff_odom / static_cast<float>(dt);
                    
                    // 转到 base_link 系
                    Eigen::Vector3f instant_linear_vel_base = q_base_in_odom.inverse() * instant_linear_vel_odom;
                    
                    // 低通滤波：v_filtered = alpha * v_instant + (1 - alpha) * v_filtered_prev
                    filtered_linear_velocity_ = VELOCITY_FILTER_ALPHA * instant_linear_vel_base + 
                                                (1.0f - VELOCITY_FILTER_ALPHA) * filtered_linear_velocity_;
                    
                    linear_vel_base = filtered_linear_velocity_;
                     
                }
            } else {
                // 第一帧，初始化
                velocity_initialized_ = true;
                RCLCPP_INFO(get_logger(), "Velocity estimator initialized: angular (rigid body), linear (differential + LPF)");
            }
            
            // 更新历史数据
            prev_position_ = pos_base_in_odom;
            prev_orientation_ = q_base_in_odom;
            prev_timestamp_ = odometry.timestamp;
            
            // 填充 twist
            odometry_msg.twist.twist.linear.x = linear_vel_base.x();
            odometry_msg.twist.twist.linear.y = linear_vel_base.y();
            odometry_msg.twist.twist.linear.z = linear_vel_base.z();
            odometry_msg.twist.twist.angular.x = angular_vel_base.x();
            odometry_msg.twist.twist.angular.y = angular_vel_base.y();
            odometry_msg.twist.twist.angular.z = angular_vel_base.z();

            tf_broadcaster->sendTransform(transform_stamped);
            odometry_publisher->publish(odometry_msg);
        });
        small_point_lio->set_pointcloud_callback([this, save_pcd, lidar_frame](const std::vector<Eigen::Vector3f> &pointcloud) {
            if (pointcloud_publisher->get_subscription_count() > 0) {
                if (!extrinsic_valid_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "Extrinsic not ready, skipping pointcloud callback");
                    return;
                }
                
                // 如果启用重力对齐但尚未完成，跳过发布
                if (gravity_alignment_enabled_ && !gravity_alignment_done_) {
                    return;
                }

                builtin_interfaces::msg::Time time_msg;
                time_msg.sec = std::floor(last_odometry.timestamp);
                time_msg.nanosec = static_cast<uint32_t>((last_odometry.timestamp - time_msg.sec) * 1e9);

                if (use_dynamic_lidar_extrinsic_ && !updateDynamicExtrinsic(time_msg)) {
                    return;
                }

                // ============================================================
                // 点云变换流程：
                // 
                // 模式1：重力对齐启用 (gravity_alignment: true)
                //   - 原始点云在 lidar_odom 系下（LIO内部全局地图）
                //   - 应用重力对齐旋转，变换到水平的 odom 系
                //   - frame_id = "odom"（Z轴竖直向上的世界坐标系）
                //
                // 模式2：重力对齐禁用 (gravity_alignment: false)
                //   - 原始点云在 lidar_odom 系下
                //   - 应用相似变换，将点云转换到 base_link 为中心的 odom 系
                //   - frame_id = "odom"（但实际是以 base_link 为参考的坐标系）
                // ============================================================
                
                pcl::PointCloud<pcl::PointXYZI> pcl_pointcloud;
                pcl_pointcloud.points.reserve(pointcloud.size());
                
                if (gravity_alignment_enabled_) {
                    // 模式1：仅应用重力对齐旋转（点云是全局地图，只需要姿态对齐）
                    for (const auto &point: pointcloud) {
                        Eigen::Vector3f aligned_point = q_gravity_align_ * point;
                        pcl::PointXYZI new_point;
                        new_point.x = aligned_point.x();
                        new_point.y = aligned_point.y();
                        new_point.z = aligned_point.z();
                        pcl_pointcloud.points.push_back(new_point);
                    }
                } else {
                    // 模式2：应用相似变换
                    // 将点云从 lidar_odom 系转换到以 base_link 为参考的 odom 系
                    
                    // 对于全局点云地图，我们只需要应用静态的外参变换
                    // 把地图从"以初始 lidar 为原点"变换到"以初始 base_link 为原点"
                    for (const auto &point_in_lidar_odom: pointcloud) {
                        Eigen::Vector3f point_transformed = T_lidar_to_base_ * point_in_lidar_odom;
                        
                        pcl::PointXYZI new_point;
                        new_point.x = point_transformed.x();
                        new_point.y = point_transformed.y();
                        new_point.z = point_transformed.z();
                        pcl_pointcloud.points.push_back(new_point);
                    }
                }
                
                pcl_pointcloud.width = pcl_pointcloud.points.size();
                pcl_pointcloud.height = 1;
                pcl_pointcloud.is_dense = true;
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(pcl_pointcloud, msg);
                msg.header.stamp = time_msg;
                msg.header.frame_id = "odom";
                pointcloud_publisher->publish(msg);
            }
            if (save_pcd) {
                pointcloud_to_save.insert(pointcloud_to_save.end(), pointcloud.begin(), pointcloud.end());
            }
        });
        if (lidar_type == "livox") {
#ifdef HAVE_LIVOX_DRIVER
            lidar_adapter = std::make_unique<LivoxLidarAdapter>();
#else
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "Livox driver requested but not available!");
            rclcpp::shutdown();
            return;
#endif
        } else if (lidar_type == "unilidar") {
            lidar_adapter = std::make_unique<UnilidarAdapter>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "unknwon lidar type");
            rclcpp::shutdown();
            return;
        }
        lidar_adapter->setup_subscription(this, lidar_topic, [this](const std::vector<common::Point> &pointcloud) {
            small_point_lio->on_point_cloud_callback(pointcloud);
            small_point_lio->handle_once();
        });
        imu_subsciber = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu &msg) {
                    common::ImuMsg imu_msg;
                    imu_msg.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                    imu_msg.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                    imu_msg.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                    
                    // 收集IMU数据用于重力对齐初始化
                    if (gravity_alignment_enabled_ && !gravity_alignment_done_) {
                        imu_accel_buffer_.push_back(imu_msg.linear_acceleration);
                        if (imu_accel_buffer_.size() % 50 == 0) {
                            RCLCPP_INFO(get_logger(), "Collecting IMU data for gravity alignment: %zu/%zu",
                                imu_accel_buffer_.size(), GRAVITY_INIT_SAMPLES);
                        }
                        if (imu_accel_buffer_.size() >= GRAVITY_INIT_SAMPLES) {
                            // 计算平均加速度
                            Eigen::Vector3d avg_accel = Eigen::Vector3d::Zero();
                            for (const auto& accel : imu_accel_buffer_) {
                                avg_accel += accel;
                            }
                            avg_accel /= static_cast<double>(imu_accel_buffer_.size());
                            computeGravityAlignment(avg_accel);
                            imu_accel_buffer_.clear();
                        }
                    }
                    
                    small_point_lio->on_imu_callback(imu_msg);
                    small_point_lio->handle_once();
                });
    }

    void SmallPointLioNode::computeGravityAlignment(const Eigen::Vector3d& measured_gravity) {
        // measured_gravity 是IMU测量的加速度（静止时指向上方，约[0,0,9.81]或根据IMU安装而不同）
        // 
        // LIO的lidar_odom系定义：以第一帧IMU的姿态为原点，Z轴方向由IMU测量决定
        // 如果IMU测到的加速度是 [ax, ay, az]（归一化后），这就是lidar_odom系的"上"方向
        //
        // 我们想要odom系的Z轴严格向上，所以需要计算一个旋转：
        // R_align * gravity_dir_in_lidar_odom = [0, 0, 1]_in_odom
        //
        // 这个 R_align 应用到 lidar_odom 系下的所有向量，就能得到 odom 系下的表示
        
        Eigen::Vector3d gravity_dir = measured_gravity.normalized();  // lidar_odom系中"上"的方向
        Eigen::Vector3d target_up(0.0, 0.0, 1.0);  // odom系中"上"的方向
        
        // FromTwoVectors(a, b) 返回将 a 旋转到 b 的四元数
        Eigen::Quaterniond q_align = Eigen::Quaterniond::FromTwoVectors(gravity_dir, target_up);
        q_gravity_align_ = q_align.cast<float>();
        
        gravity_alignment_done_ = true;
        
        RCLCPP_INFO(get_logger(), "Gravity alignment computed:");
        RCLCPP_INFO(get_logger(), "  Measured 'up' direction in lidar_odom: [%.4f, %.4f, %.4f]",
            gravity_dir.x(), gravity_dir.y(), gravity_dir.z());
        RCLCPP_INFO(get_logger(), "  Alignment quaternion (lidar_odom → odom): [w=%.4f, x=%.4f, y=%.4f, z=%.4f]",
            q_gravity_align_.w(), q_gravity_align_.x(), q_gravity_align_.y(), q_gravity_align_.z());
        
        // 验证：对齐后的"上"方向应该是 [0, 0, 1]
        Eigen::Vector3f aligned_up = q_gravity_align_ * gravity_dir.cast<float>();
        RCLCPP_INFO(get_logger(), "  Aligned 'up' direction in odom: [%.4f, %.4f, %.4f]",
            aligned_up.x(), aligned_up.y(), aligned_up.z());
    }

}// namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)
