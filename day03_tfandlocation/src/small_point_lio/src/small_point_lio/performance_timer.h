/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace small_point_lio {

/**
 * @brief 轻量级RAII风格的计时器（使用累积统计）
 * 使用方法：
 *   static AccumulativeTimer acc_timer("function_name", 10000); // 每10秒报告一次
 *   if (parameters->enable_performance_debug) {
 *       ScopedTimer timer(acc_timer);
 *       // ... 你的代码 ...
 *   } // 析构时自动累积耗时
 */
class ScopedTimer {
private:
    class AccumulativeTimer* accumulator_;
    std::chrono::steady_clock::time_point start_;
    bool enabled_;
    
public:
    explicit ScopedTimer(class AccumulativeTimer& accumulator, bool enabled = true);
    ~ScopedTimer();
    
    // 禁止拷贝
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
};

/**
 * @brief 累积计时器 - 用于统计多次调用的总耗时和平均值
 * 按时间间隔（默认10秒）报告统计信息
 */
class AccumulativeTimer {
private:
    std::string name_;
    double total_time_ms_ = 0.0;
    int count_ = 0;
    int report_interval_ms_;  // 报告间隔（毫秒）
    bool enabled_;
    std::chrono::steady_clock::time_point last_report_time_;
    
public:
    explicit AccumulativeTimer(const std::string &name, int report_interval_ms = 10000, bool enabled = true)
        : name_(name), report_interval_ms_(report_interval_ms), enabled_(enabled) {
        last_report_time_ = std::chrono::steady_clock::now();
    }
    
    void add_sample(double time_ms) {
        if (!enabled_) return;
        
        total_time_ms_ += time_ms;
        count_++;
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report_time_).count();
        
        if (elapsed >= report_interval_ms_) {
            if (count_ > 0) {
                double avg_time = total_time_ms_ / count_;
                double frequency = count_ * 1000.0 / elapsed;
                RCLCPP_INFO(rclcpp::get_logger("perf"), 
                            "[📊 %s] Avg: %.3f ms, Freq: %.1f Hz, Samples: %d, Period: %.1f s", 
                            name_.c_str(), avg_time, frequency, count_, elapsed / 1000.0);
            }
            // 重置统计
            total_time_ms_ = 0.0;
            count_ = 0;
            last_report_time_ = now;
        }
    }
    
    friend class ScopedTimer;
};

// ScopedTimer实现
inline ScopedTimer::ScopedTimer(AccumulativeTimer& accumulator, bool enabled)
    : accumulator_(&accumulator), enabled_(enabled) {
    if (enabled_) {
        start_ = std::chrono::steady_clock::now();
    }
}

inline ScopedTimer::~ScopedTimer() {
    if (enabled_) {
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count();
        accumulator_->add_sample(duration / 1000.0);
    }
}

} // namespace small_point_lio
