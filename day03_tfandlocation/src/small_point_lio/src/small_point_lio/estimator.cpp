/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "estimator.h"

namespace small_point_lio {

    constexpr int NUM_MATCH_POINTS = 5;

    Estimator::Estimator() {// NOLINT(cppcoreguidelines-pro-type-member-init)
        kf.init(
                [this](auto &&s, auto &&measurement_result) {
                    return h_point(s, measurement_result);
                },
                [this](auto &&s, auto &&measurement_result) {
                    return h_imu(s, measurement_result);
                });
        kf.init_batch(
                [this](auto &&s, auto &&measurement_result) {
                    return h_point_batch(s, measurement_result);
                });
    }

    void Estimator::reset() {
        ivox = std::make_shared<SmallIVox>(parameters->map_resolution, 1000000);
        kf.P = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Identity() * 0.01;
        kf.P.block<3, 3>(state::gravity_index, state::gravity_index).diagonal().fill(0.0001);
        kf.P.block<3, 3>(state::bg_index, state::bg_index).diagonal().fill(0.001);
        kf.P.block<3, 3>(state::ba_index, state::ba_index).diagonal().fill(0.001);

        if (parameters->use_batch_update && parameters->batch_max_points > 0) {
            kf.reserve_batch_buffers(parameters->batch_max_points);
            batch_points_lidar_frame.reserve(parameters->batch_max_points);
            batch_points_timestamps.reserve(parameters->batch_max_points);
            batch_points_undistorted.reserve(parameters->batch_max_points);
            batch_points_odom_frame.reserve(parameters->batch_max_points);
        }
    }

    [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> Estimator::process_noise_cov() const {
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> cov = Eigen::Matrix<state::value_type, state::DIM, state::DIM>::Zero();
        cov.block<3, 3>(state::velocity_index, state::velocity_index).diagonal().fill(static_cast<state::value_type>(parameters->velocity_cov));
        cov.block<3, 3>(state::omg_index, state::omg_index).diagonal().fill(static_cast<state::value_type>(parameters->omg_cov));
        cov.block<3, 3>(state::acceleration_index, state::acceleration_index).diagonal().fill(static_cast<state::value_type>(parameters->acceleration_cov));
        cov.block<3, 3>(state::bg_index, state::bg_index).diagonal().fill(static_cast<state::value_type>(parameters->bg_cov));
        cov.block<3, 3>(state::ba_index, state::ba_index).diagonal().fill(static_cast<state::value_type>(parameters->ba_cov));
        return cov;
    }

    void Estimator::h_point(const state &s, point_measurement_result &measurement_result) {
        measurement_result.valid = false;
        // get closest point
        Eigen::Matrix<state::value_type, 3, 1> point_imu_frame;
        if (parameters->extrinsic_est_en) {
            point_imu_frame = s.offset_R_L_I * point_lidar_frame.cast<state::value_type>() + s.offset_T_L_I;
        } else {
            point_imu_frame = Lidar_R_wrt_IMU * point_lidar_frame.cast<state::value_type>() + Lidar_T_wrt_IMU;
        }
        point_odom_frame = (s.rotation * point_imu_frame + s.position).cast<float>();
        ivox->get_closest_point(point_odom_frame, nearest_points, NUM_MATCH_POINTS);
        if (nearest_points.size() != NUM_MATCH_POINTS) {
            return;
        }
        // estimate plane
#if 0
        Eigen::Matrix<float, NUM_MATCH_POINTS, 3> A;
        for (int j = 0; j < NUM_MATCH_POINTS; j++) {
            A.row(j) = nearest_points[j];
        }
        Eigen::Matrix<float, NUM_MATCH_POINTS, 1> b;
        b.setConstant(-1);
        Eigen::Vector3f normal = A.colPivHouseholderQr().solve(b);
        float d = 1.0f / normal.norm();
#else
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto &p: nearest_points) {
            centroid.noalias() += p;
        }
        centroid /= static_cast<float>(nearest_points.size());
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (const auto &p: nearest_points) {
            Eigen::Vector3f centered = p - centroid;
            covariance.noalias() += centered * centered.transpose();
        }
        covariance /= static_cast<float>(nearest_points.size() - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        Eigen::Vector3f normal = solver.eigenvectors().col(0);
        float d = -normal.dot(centroid);
#endif
        for (int j = 0; j < NUM_MATCH_POINTS; j++) {
            float point_distanace = std::abs(normal.dot(nearest_points[j]) + d);
            if (point_distanace > parameters->plane_threshold) {
                return;
            }
        }
        float point_distanace = normal.dot(point_odom_frame) + d;
        if (point_lidar_frame.norm() <= parameters->match_sqaured * point_distanace * point_distanace) {
            return;
        }
        // calculate residual and jacobian matrix
        measurement_result.laser_point_cov = static_cast<state::value_type>(parameters->laser_point_cov);
        if (parameters->extrinsic_est_en) {
            Eigen::Matrix<state::value_type, 3, 1> normal0 = normal.cast<state::value_type>();
            Eigen::Matrix<state::value_type, 3, 1> C = s.rotation.transpose() * normal0;
            Eigen::Matrix<state::value_type, 3, 1> A, B;
            A.noalias() = point_imu_frame.cross(C);
            B.noalias() = point_lidar_frame.cast<state::value_type>().cross(s.offset_R_L_I.transpose() * C);
            measurement_result.H << normal0.transpose(), A.transpose(), B.transpose(), C.transpose();
        } else {
            Eigen::Matrix<state::value_type, 3, 1> normal0 = normal.cast<state::value_type>();
            Eigen::Matrix<state::value_type, 3, 1> A;
            A.noalias() = point_imu_frame.cross(s.rotation.transpose() * normal0);
            measurement_result.H << normal0.transpose(), A.transpose(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        measurement_result.z = -point_distanace;
        measurement_result.valid = true;
    }

    void Estimator::h_imu(const state &s, imu_measurement_result &measurement_result) {
        std::memset(measurement_result.satu_check, false, 6);
        measurement_result.z.segment<3>(0) = angular_velocity - s.omg - s.bg;
        measurement_result.z.segment<3>(3) = linear_acceleration * imu_acceleration_scale - s.acceleration - s.ba;
        measurement_result.imu_meas_omg_cov = static_cast<state::value_type>(parameters->imu_meas_omg_cov);
        measurement_result.imu_meas_acc_cov = static_cast<state::value_type>(parameters->imu_meas_acc_cov);
        if (parameters->check_satu) {
            for (int i = 0; i < 3; i++) {
                if (std::abs(angular_velocity(i)) >= parameters->satu_gyro) {
                    measurement_result.satu_check[i] = true;
                    measurement_result.z(i) = 0.0;
                }
                if (std::abs(linear_acceleration(i)) >= parameters->satu_acc) {
                    measurement_result.satu_check[i + 3] = true;
                    measurement_result.z(i + 3) = 0.0;
                }
            }
        }
    }

    // 点云去畸变实现（参考BATCH_LIWO公式3.44-3.47）
    Eigen::Vector3f Estimator::undistort_point(const Eigen::Vector3f &point, 
                                                 double point_time, 
                                                 double last_time,
                                                 const Eigen::Vector3f &velocity, 
                                                 const Eigen::Vector3f &angular_velocity) {
        // 计算时间差 Δt_j = t_j - t_Last（公式3.44）
        float dt = static_cast<float>(point_time - last_time);
        
        // 计算旋转角度 θ_j = ||ω|| * Δt_j 和旋转轴 a = ω/||ω||（公式3.45）
        float omega_norm = angular_velocity.norm();
        Eigen::Vector3f axis;
        Eigen::Matrix3f R_j;
        
        if (omega_norm > 1e-6f) {
            axis = angular_velocity / omega_norm;
            float theta = omega_norm * dt;
            
            // 使用Rodrigues公式计算旋转矩阵 R_j = exp(θ_j * [a]^)（公式3.46）
            Eigen::Matrix3f axis_hat;
            axis_hat << 0.0f, -axis(2), axis(1),
                        axis(2), 0.0f, -axis(0),
                       -axis(1), axis(0), 0.0f;
            
            float sin_theta = std::sin(theta);
            float cos_theta = std::cos(theta);
            R_j = Eigen::Matrix3f::Identity() + sin_theta * axis_hat + (1.0f - cos_theta) * axis_hat * axis_hat;
        } else {
            R_j = Eigen::Matrix3f::Identity();
        }
        
        // 计算平移 T_j = v * Δt_j（公式3.46）
        Eigen::Vector3f T_j = velocity * dt;
        
        // 去畸变变换 p'_j = R_j * p_j + T_j（公式3.47）
        Eigen::Vector3f point_undistorted = R_j * point + T_j;
        
        return point_undistorted;
    }

    // Batch点云观测模型（参考BATCH_LIWO公式3.48）
    void Estimator::h_point_batch(const state &s, batch_point_measurement_result &measurement_result) {
        measurement_result.valid = false;
        measurement_result.num_valid_points = 0;
        
        int num_points = static_cast<int>(batch_points_lidar_frame.size());
        if (num_points == 0) {
            return;
        }
        
        // 获取最后一个点的时间作为参考时刻
        double last_time = batch_points_timestamps.back();
        
        // 获取当前速度和角速度（在IMU坐标系下）
        Eigen::Vector3f velocity_imu = (s.rotation.transpose() * s.velocity).cast<float>();
        Eigen::Vector3f angular_velocity_imu = s.omg.cast<float>();
        
        // 预分配存储空间
        batch_points_undistorted.clear();
        batch_points_undistorted.reserve(num_points);
        batch_points_odom_frame.clear();
        batch_points_odom_frame.reserve(num_points);

        measurement_result.ensure_capacity(num_points);
        int num_valid = 0;
        
        // 遍历Batch中的每个点
        for (int i = 0; i < num_points; ++i) {
            const Eigen::Vector3f &point_lidar_frame = batch_points_lidar_frame[i];

            // 1. 先变换到IMU坐标系，再在IMU坐标系中去畸变
            Eigen::Matrix<state::value_type, 3, 1> point_imu_raw;
            if (parameters->extrinsic_est_en) {
                point_imu_raw = s.offset_R_L_I * point_lidar_frame.cast<state::value_type>() + s.offset_T_L_I;
            } else {
                point_imu_raw = Lidar_R_wrt_IMU * point_lidar_frame.cast<state::value_type>() + Lidar_T_wrt_IMU;
            }

            Eigen::Vector3f point_undistorted_imu = undistort_point(
                point_imu_raw.cast<float>(),
                batch_points_timestamps[i],
                last_time,
                velocity_imu,
                angular_velocity_imu
            );
            batch_points_undistorted.push_back(point_undistorted_imu);

            Eigen::Matrix<state::value_type, 3, 1> point_imu_frame = point_undistorted_imu.cast<state::value_type>();
            
            // 2. 变换到odom坐标系
            Eigen::Vector3f point_odom = (s.rotation * point_imu_frame + s.position).cast<float>();
            batch_points_odom_frame.push_back(point_odom);
            
            // 3. 在地图中搜索最近邻点
            ivox->get_closest_point(point_odom, nearest_points, NUM_MATCH_POINTS);
            if (nearest_points.size() != NUM_MATCH_POINTS) {
                continue;  // 跳过无法匹配的点
            }
            
            // 4. 拟合平面
            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            for (const auto &p: nearest_points) {
                centroid.noalias() += p;
            }
            centroid /= static_cast<float>(nearest_points.size());
            
            Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
            for (const auto &p: nearest_points) {
                Eigen::Vector3f centered = p - centroid;
                covariance.noalias() += centered * centered.transpose();
            }
            covariance /= static_cast<float>(nearest_points.size() - 1);
            
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
            Eigen::Vector3f normal = solver.eigenvectors().col(0);
            float d = -normal.dot(centroid);
            
            // 5. 检查平面拟合质量
            bool plane_valid = true;
            for (int j = 0; j < NUM_MATCH_POINTS; j++) {
                float point_distance = std::abs(normal.dot(nearest_points[j]) + d);
                if (point_distance > parameters->plane_threshold) {
                    plane_valid = false;
                    break;
                }
            }
            if (!plane_valid) {
                continue;
            }
            
            float point_distance = normal.dot(point_odom) + d;
            if (point_lidar_frame.norm() <= parameters->match_sqaured * point_distance * point_distance) {
                continue;
            }
            
            // 6. 计算残差和雅可比矩阵
            Eigen::Matrix<state::value_type, 1, 12> H;
            if (parameters->extrinsic_est_en) {
                Eigen::Matrix<state::value_type, 3, 1> normal0 = normal.cast<state::value_type>();
                Eigen::Matrix<state::value_type, 3, 1> C = s.rotation.transpose() * normal0;
                Eigen::Matrix<state::value_type, 3, 1> A, B;
                A.noalias() = point_imu_frame.cross(C);
                B.noalias() = point_lidar_frame.cast<state::value_type>().cross(s.offset_R_L_I.transpose() * C);
                H << normal0.transpose(), A.transpose(), B.transpose(), C.transpose();
            } else {
                Eigen::Matrix<state::value_type, 3, 1> normal0 = normal.cast<state::value_type>();
                Eigen::Matrix<state::value_type, 3, 1> A;
                A.noalias() = point_imu_frame.cross(s.rotation.transpose() * normal0);
                H << normal0.transpose(), A.transpose(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }
            
            measurement_result.z(num_valid) = -point_distance;
            measurement_result.H.row(num_valid) = H;
            measurement_result.R_diag(num_valid) = static_cast<state::value_type>(parameters->laser_point_cov);
            ++num_valid;
        }

        // 8. 组装Batch观测结果（公式3.48）
        if (num_valid == 0) {
            return;
        }

        measurement_result.num_valid_points = num_valid;
        
        measurement_result.valid = true;
    }

}// namespace small_point_lio
