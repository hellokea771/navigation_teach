/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio.h"
#include "performance_timer.h"

namespace small_point_lio {

    SmallPointLio::SmallPointLio(rclcpp::Node &node) {
        // init param
        parameters.read_parameters(node);
        preprocess.parameters = &parameters;
        estimator.parameters = &parameters;
        estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T.cast<state::value_type>();
        estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R.cast<state::value_type>();
        if (parameters.extrinsic_est_en) {
            estimator.kf.x.offset_T_L_I = parameters.extrinsic_T.cast<state::value_type>();
            estimator.kf.x.offset_R_L_I = parameters.extrinsic_R.cast<state::value_type>();
        }
        Q = estimator.process_noise_cov();
        estimator.imu_acceleration_scale = parameters.gravity.norm() / parameters.acc_norm;

        // init data
        reset();
    }

    void SmallPointLio::reset() {
        preprocess.reset();
        estimator.reset();
        is_init = false;
    }

    void SmallPointLio::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        preprocess.on_imu_callback(imu_msg);
    }

    void SmallPointLio::handle_once() {
        // 根据参数选择使用原始逐点更新或Batch更新
        if (parameters.use_batch_update) {
            handle_once_batch();
        } else {
            handle_once_original();
        }
    }

    void SmallPointLio::handle_once_original() {
        // 统计一帧点云的总用时
        static AccumulativeTimer frame_timer("0.frame_total", 10000, parameters.enable_performance_debug);
        ScopedTimer frame_scope_timer(frame_timer, parameters.enable_performance_debug);
        
        // we need to init small point lio
        if (!is_init) {
            if ((!preprocess.point_deque.empty() || !preprocess.imu_deque.empty()) &&
                preprocess.point_deque.size() >= parameters.init_map_size &&
                (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // init map
                for (const auto &point: preprocess.point_deque) {
                    estimator.ivox->add_point(point.position);
                }
                // fix gravity direction
                if (parameters.fix_gravity_direction) {
                    estimator.kf.x.gravity = Eigen::Matrix<state::value_type, 3, 1>::Zero();
                    for (const auto &imu_msg: preprocess.imu_deque) {
                        estimator.kf.x.gravity += imu_msg.linear_acceleration.cast<state::value_type>();
                    }
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "Estimated gravity before norm: [%f, %f, %f]", estimator.kf.x.gravity(0), estimator.kf.x.gravity(1), estimator.kf.x.gravity(2));
                    state::value_type scale = -static_cast<state::value_type>(parameters.gravity.norm()) / estimator.kf.x.gravity.norm();
                    estimator.kf.x.gravity *= scale;
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "Gravity direction estimated in lidar_odom frame: [%f, %f, %f]", estimator.kf.x.gravity(0), estimator.kf.x.gravity(1), estimator.kf.x.gravity(2));
                } else {
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                }
                estimator.kf.x.acceleration = -estimator.kf.x.gravity;
                // init time
                if (preprocess.point_deque.empty()) {
                    time_current = preprocess.imu_deque.back().timestamp;
                } else if (preprocess.imu_deque.empty()) {
                    time_current = preprocess.point_deque.back().timestamp;
                } else {
                    time_current = std::max(preprocess.point_deque.back().timestamp, preprocess.imu_deque.back().timestamp);
                }
                estimator.kf.init_timestamp(time_current);
                // clear data
                preprocess.point_deque.clear();
                preprocess.dense_point_deque.clear();
                preprocess.imu_deque.clear();
                is_init = true;
            }
            return;
        }

        // judge we should do point update or imu update
        bool is_publish_odometry = !preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty() &&
                                   preprocess.imu_deque.front().timestamp < preprocess.point_deque.back().timestamp;
        while (!preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty()) {
            const common::Point &point_lidar_frame = preprocess.point_deque.front();
            const common::Point &dense_point_lidar_frame = preprocess.dense_point_deque.front();
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            if (dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp && dense_point_lidar_frame.timestamp < imu_msg.timestamp) {
                // collect odom frame pointcloud
                Eigen::Matrix<state::value_type, 3, 1> dense_point_imu_frame;
                if (parameters.extrinsic_est_en) {
                    dense_point_imu_frame = estimator.kf.x.offset_R_L_I * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                } else {
                    dense_point_imu_frame = estimator.Lidar_R_wrt_IMU * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                }
                pointcloud_odom_frame.emplace_back((estimator.kf.x.rotation * dense_point_imu_frame + estimator.kf.x.position).cast<float>());

                preprocess.dense_point_deque.pop_front();
            } else if (point_lidar_frame.timestamp < imu_msg.timestamp) {
                // point update
                if (point_lidar_frame.timestamp < time_current) {
                    preprocess.point_deque.pop_front();
                    continue;
                }
                time_current = point_lidar_frame.timestamp;

                // predict
                {
                    static AccumulativeTimer acc_timer("1.predict_state", 10000, parameters.enable_performance_debug);
                    ScopedTimer timer(acc_timer, parameters.enable_performance_debug);
                    estimator.kf.predict_state(time_current);
                }

                // update
                estimator.point_lidar_frame = point_lidar_frame.position;
                {
                    static AccumulativeTimer acc_timer("2.update_point", 10000, parameters.enable_performance_debug);
                    ScopedTimer timer(acc_timer, parameters.enable_performance_debug);
                    estimator.kf.update_point();
                }

                // publish odometry
                if (parameters.publish_odometry_without_downsample) {
                    publish_odometry(time_current);
                }

                // map incremental
                {
                    static AccumulativeTimer acc_timer("3.map_add_point", 10000, parameters.enable_performance_debug);
                    ScopedTimer timer(acc_timer, parameters.enable_performance_debug);
                    estimator.ivox->add_point(estimator.point_odom_frame);
                }

                preprocess.point_deque.pop_front();
            } else {
                // imu update
                if (imu_msg.timestamp < time_current) {
                    preprocess.imu_deque.pop_front();
                    continue;
                }
                time_current = imu_msg.timestamp;

                // predict
                {
                    static AccumulativeTimer acc_timer("4.predict_state_imu", 10000, parameters.enable_performance_debug);
                    ScopedTimer timer(acc_timer, parameters.enable_performance_debug);
                    estimator.kf.predict_state(time_current);
                }
                {
                    static AccumulativeTimer acc_timer("5.predict_cov", 10000, parameters.enable_performance_debug);
                    ScopedTimer timer(acc_timer, parameters.enable_performance_debug);
                    estimator.kf.predict_cov(time_current, Q);
                }

                // update
                estimator.angular_velocity = imu_msg.angular_velocity.cast<state::value_type>();
                estimator.linear_acceleration = imu_msg.linear_acceleration.cast<state::value_type>();
                {
                    static AccumulativeTimer acc_timer("6.update_imu", 10000, parameters.enable_performance_debug);
                    ScopedTimer timer(acc_timer, parameters.enable_performance_debug);
                    estimator.kf.update_imu();
                }

                preprocess.imu_deque.pop_front();
            }
        }

        if (is_publish_odometry) {
            if (!parameters.publish_odometry_without_downsample) {
                publish_odometry(time_current);
            }
            if (!pointcloud_odom_frame.empty()) {
                if (pointcloud_callback) {
                    pointcloud_callback(pointcloud_odom_frame);
                }
                pointcloud_odom_frame.clear();
            }
        }
    }

    void SmallPointLio::set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback) {
        this->pointcloud_callback = pointcloud_callback;
    }

    void SmallPointLio::set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback) {
        this->odometry_callback = odometry_callback;
    }

    void SmallPointLio::publish_odometry(double timestamp) {
        if (odometry_callback) {
            common::Odometry odometry;
            odometry.timestamp = timestamp;
            odometry.position = estimator.kf.x.position.cast<double>();
            odometry.velocity = estimator.kf.x.velocity.cast<double>();
            odometry.orientation = estimator.kf.x.rotation.cast<double>();
            odometry.angular_velocity = estimator.kf.x.omg.cast<double>();
            odometry_callback(odometry);
        }
    }

    void SmallPointLio::flush_batch_points() {
        if (batch_points_buffer.empty()) {
            return;
        }

        // 更新时间到最后一个点的时间
        time_current = batch_points_buffer.back().timestamp;

        // 状态预测
        {
            static AccumulativeTimer acc_timer("B1.predict_state", 10000, parameters.enable_performance_debug);
            ScopedTimer timer(acc_timer, parameters.enable_performance_debug);
            estimator.kf.predict_state(time_current);
        }

        // 准备Batch数据
        {
            static AccumulativeTimer acc_timer("B2.prepare_batch_data", 10000, parameters.enable_performance_debug);
            ScopedTimer timer(acc_timer, parameters.enable_performance_debug);
            estimator.batch_points_lidar_frame.clear();
            estimator.batch_points_timestamps.clear();
            estimator.batch_points_lidar_frame.reserve(batch_points_buffer.size());
            estimator.batch_points_timestamps.reserve(batch_points_buffer.size());

            for (const auto &pt: batch_points_buffer) {
                estimator.batch_points_lidar_frame.push_back(pt.position);
                estimator.batch_points_timestamps.push_back(pt.timestamp);
            }
        }

        // Batch更新。更新失败只代表本Batch没有形成有效残差，不代表这些点不应注册到地图。
        {
            static AccumulativeTimer acc_timer("B3.update_point_batch", 10000, parameters.enable_performance_debug);
            ScopedTimer timer(acc_timer, parameters.enable_performance_debug);
            estimator.kf.update_point_batch();
        }

        // 将Batch中能完成坐标转换的点添加到地图
        if (!estimator.batch_points_odom_frame.empty()) {
            static AccumulativeTimer acc_timer("B4.map_add_points", 10000, parameters.enable_performance_debug);
            ScopedTimer timer(acc_timer, parameters.enable_performance_debug);
            for (const auto &pt_odom: estimator.batch_points_odom_frame) {
                estimator.ivox->add_point(pt_odom);
            }
        }

        // 发布高频里程计（如果启用）
        if (parameters.publish_odometry_without_downsample) {
            publish_odometry(time_current);
        }

        // 清空Batch缓存
        batch_points_buffer.clear();
        batch_point_count = 0;
    }

    void SmallPointLio::handle_once_batch() {
        // 统计一帧点云的总用时（Batch模式）
        static AccumulativeTimer frame_timer("0.frame_total_batch", 10000, parameters.enable_performance_debug);
        ScopedTimer frame_scope_timer(frame_timer, parameters.enable_performance_debug);
        
        // 初始化逻辑与原始版本完全相同
        if (!is_init) {
            if ((!preprocess.point_deque.empty() || !preprocess.imu_deque.empty()) &&
                preprocess.point_deque.size() >= parameters.init_map_size &&
                (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // init map
                for (const auto &point: preprocess.point_deque) {
                    estimator.ivox->add_point(point.position);
                }
                // fix gravity direction
                if (parameters.fix_gravity_direction) {
                    estimator.kf.x.gravity = Eigen::Matrix<state::value_type, 3, 1>::Zero();
                    for (const auto &imu_msg: preprocess.imu_deque) {
                        estimator.kf.x.gravity += imu_msg.linear_acceleration.cast<state::value_type>();
                    }
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "Estimated gravity before norm: [%f, %f, %f]", estimator.kf.x.gravity(0), estimator.kf.x.gravity(1), estimator.kf.x.gravity(2));
                    state::value_type scale = -static_cast<state::value_type>(parameters.gravity.norm()) / estimator.kf.x.gravity.norm();
                    estimator.kf.x.gravity *= scale;
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "Gravity direction estimated in lidar_odom frame: [%f, %f, %f]", estimator.kf.x.gravity(0), estimator.kf.x.gravity(1), estimator.kf.x.gravity(2));
                } else {
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                }
                estimator.kf.x.acceleration = -estimator.kf.x.gravity;
                // init time
                if (preprocess.point_deque.empty()) {
                    time_current = preprocess.imu_deque.back().timestamp;
                } else if (preprocess.imu_deque.empty()) {
                    time_current = preprocess.point_deque.back().timestamp;
                } else {
                    time_current = std::max(preprocess.point_deque.back().timestamp, preprocess.imu_deque.back().timestamp);
                }
                estimator.kf.init_timestamp(time_current);
                // clear data
                preprocess.point_deque.clear();
                preprocess.dense_point_deque.clear();
                preprocess.imu_deque.clear();
                batch_points_buffer.clear();
                batch_point_count = 0;
                is_init = true;
            }
            return;
        }

        // Batch模式：收集点云并批量处理
        bool is_publish_odometry = !preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty() &&
                                   preprocess.imu_deque.front().timestamp < preprocess.point_deque.back().timestamp;
        
        while (!preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty()) {
            const common::Point &point_lidar_frame = preprocess.point_deque.front();
            const common::Point &dense_point_lidar_frame = preprocess.dense_point_deque.front();
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            
            if (dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp && dense_point_lidar_frame.timestamp < imu_msg.timestamp) {
                // collect odom frame pointcloud (与原版相同)
                Eigen::Matrix<state::value_type, 3, 1> dense_point_imu_frame;
                if (parameters.extrinsic_est_en) {
                    dense_point_imu_frame = estimator.kf.x.offset_R_L_I * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                } else {
                    dense_point_imu_frame = estimator.Lidar_R_wrt_IMU * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                }
                pointcloud_odom_frame.emplace_back((estimator.kf.x.rotation * dense_point_imu_frame + estimator.kf.x.position).cast<float>());
                preprocess.dense_point_deque.pop_front();
                
            } else if (point_lidar_frame.timestamp < imu_msg.timestamp) {
                // Batch点云收集
                if (point_lidar_frame.timestamp < time_current) {
                    preprocess.point_deque.pop_front();
                    continue;
                }
                
                if (!batch_points_buffer.empty() && parameters.batch_max_duration > 0.0 &&
                    point_lidar_frame.timestamp - batch_points_buffer.front().timestamp >= parameters.batch_max_duration) {
                    flush_batch_points();
                }

                // 收集点到Batch缓存
                batch_points_buffer.push_back(point_lidar_frame);
                batch_point_count++;
                preprocess.point_deque.pop_front();
                
                // 当Batch达到设定大小或超过最大限制时，进行批量更新
                if (batch_point_count >= parameters.batch_point_size || 
                    batch_point_count >= parameters.batch_max_points ||
                    preprocess.point_deque.empty()) {
                    
                    flush_batch_points();
                }
                
            } else {
                // IMU更新（与原版完全相同）
                if (!batch_points_buffer.empty()) {
                    flush_batch_points();
                    continue;
                }

                if (imu_msg.timestamp < time_current) {
                    preprocess.imu_deque.pop_front();
                    continue;
                }
                time_current = imu_msg.timestamp;

                // predict
                estimator.kf.predict_state(time_current);
                estimator.kf.predict_cov(time_current, Q);

                // update
                estimator.angular_velocity = imu_msg.angular_velocity.cast<state::value_type>();
                estimator.linear_acceleration = imu_msg.linear_acceleration.cast<state::value_type>();
                estimator.kf.update_imu();

                preprocess.imu_deque.pop_front();
            }
        }

        // 发布里程计和点云（与原版相同）
        if (is_publish_odometry) {
            if (!parameters.publish_odometry_without_downsample) {
                publish_odometry(time_current);
            }
            if (!pointcloud_odom_frame.empty()) {
                if (pointcloud_callback) {
                    pointcloud_callback(pointcloud_odom_frame);
                }
                pointcloud_odom_frame.clear();
            }
        }
    }

}// namespace small_point_lio
