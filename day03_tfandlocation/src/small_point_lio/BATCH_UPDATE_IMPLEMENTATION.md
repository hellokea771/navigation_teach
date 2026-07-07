# Small-PointLIO Batch更新功能实现文档

## 概述
基于BATCH_LIWO论文的思路，为small-pointlio添加了Batch点云观测并进行共同更新的功能。实现了点云去畸变、批量观测模型和批量卡尔曼更新。

## 实现日期
2026年1月27日

## 核心改动

### 1. 数据结构设计 (`eskf.h`)

#### `batch_point_measurement_result`
```cpp
    struct batch_point_measurement_result {
        bool valid = false;
        int num_valid_points = 0;  // 有效点数量
        int capacity_points = 0;   // 预分配容量
        Eigen::Matrix<state::value_type, Eigen::Dynamic, 1> z;  // 残差向量 [z0, z1, ..., zN]
        Eigen::Matrix<state::value_type, Eigen::Dynamic, 12> H;  // 雅可比矩阵 [H0; H1; ...; HN]
        Eigen::Matrix<state::value_type, Eigen::Dynamic, 1> R_diag;  // 测量噪声协方差对角线
    };
```

- 参考论文公式3.48，将多个点的H矩阵和残差按行堆叠
- 使用 `ensure_capacity()/reserve_capacity()` 做容量复用，避免每帧重复分配
- 测量噪声使用对角向量 `R_diag`，在更新时加到 `HPHT` 对角线

### 2. 点云去畸变 (`estimator.cpp`)

#### `undistort_point()`函数
```cpp
Eigen::Vector3f undistort_point(const Eigen::Vector3f &point, 
                                 double point_time, 
                                 double last_time,
                                 const Eigen::Vector3f &velocity, 
                                 const Eigen::Vector3f &angular_velocity);
```

**实现细节（参考论文公式3.44-3.47）：**
- 计算时间差：`Δt_j = t_j - t_Last`（公式3.44）
- 计算旋转角度和轴：`θ_j = ||ω|| * Δt_j`，`a = ω/||ω||`（公式3.45）
- 使用Rodrigues公式计算旋转矩阵：`R_j = exp(θ_j * [a]^)`（公式3.46）
- 计算平移：`T_j = v * Δt_j`
- 去畸变变换：`p'_j = R_j * p_j + T_j`（公式3.47）

### 3. Batch观测模型 (`estimator.cpp`)

#### `h_point_batch()`函数
```cpp
void h_point_batch(const state &s, batch_point_measurement_result &measurement_result);
```

**处理流程：**
1. 获取Batch中最后一个点的时间作为参考时刻
2. 获取当前速度和角速度（IMU坐标系）
3. 对每个点进行去畸变（投影到参考时刻）
4. 变换到IMU坐标系，再到odom坐标系
5. 在地图中搜索最近邻点（5个）
6. 拟合平面（使用协方差矩阵特征分解）
7. 检查平面拟合质量
8. 计算残差和雅可比矩阵
9. 将所有有效点写入预分配的 `H`、`z`、`R_diag` 缓冲，并设置 `num_valid_points`

### 4. Batch卡尔曼更新 (`eskf.h`)

#### `update_point_batch()`函数
```cpp
bool update_point_batch();
```

**实现细节（参考论文公式3.62-3.66）：**
```cpp
// 计算 P*H^T （DIM x N）
PHT = P.block<DIM, 12>(0, 0) * H.transpose()

// 计算 H*P*H^T + R （N x N）
HPHT = H * PHT.topRows(12)
HPHT.diagonal() += R_diag

// 使用LDLT分解求解卡尔曼增益
K = ldlt.solve(PHT.transpose()).transpose()

// 状态更新：x = x ⊞ K*z
dx = K * z
x.plus(dx)

// 协方差更新：P = P - K*H*P
HP = H * P.block<12, DIM>(0, 0)
P = P - K * HP
```

**当前实现的性能优化点：**
- `eskf` 内部持有 `batch_measurement_result` 与工作区矩阵（`batch_PHT/batch_HPHT/batch_K/batch_HP`），跨帧复用
- 提供 `reserve_batch_buffers(max_points)`，在初始化阶段一次性按 `batch_max_points` 预分配
- `update_point_batch()` 仅在 `N > batch_workspace_capacity` 时触发兜底扩容（`[[unlikely]]` 分支）

### 5. 主循环改造 (`small_point_lio.cpp`)

#### 双模式架构
```cpp
void handle_once() {
    if (parameters.use_batch_update) {
        handle_once_batch();  // Batch模式
    } else {
        handle_once_original();  // 原始逐点模式
    }
}
```

**设计原则：**
- ✅ **严格保证原代码不变性**：当`use_batch_update=false`时，完全使用原始逐点更新逻辑
- ✅ **独立实现Batch逻辑**：`handle_once_batch()`与`handle_once_original()`完全独立
- ✅ **保持IMU更新不变**：两种模式下IMU更新逻辑完全相同
- ✅ **兼容所有原有功能**：初始化、重力对齐、外参估计等均保持不变

#### Batch模式处理流程
```cpp
// 1. 收集点到Batch缓存
batch_points_buffer.push_back(point_lidar_frame);
batch_point_count++;

// 2. 满足触发条件时进行更新
if (batch_point_count >= parameters.batch_point_size ||
    batch_point_count >= parameters.batch_max_points ||
    preprocess.point_deque.empty()) {
    // 2.1 状态预测到最后一个点的时刻
    estimator.kf.predict_state(time_current);
    
    // 2.2 准备Batch数据（点位置和时间戳）
    for (const auto &pt : batch_points_buffer) {
        estimator.batch_points_lidar_frame.push_back(pt.position);
        estimator.batch_points_timestamps.push_back(pt.timestamp);
    }
    
    // 2.3 Batch更新
    bool success = estimator.kf.update_point_batch();
    
    // 2.4 将去畸变后的点添加到地图
    if (success) {
        for (const auto &pt_odom : estimator.batch_points_odom_frame) {
            estimator.ivox->add_point(pt_odom);
        }
    }
    
    // 2.5 清空Batch缓存
    batch_points_buffer.clear();
    batch_point_count = 0;
}
```

#### 预分配策略（`Estimator::reset()`）
```cpp
if (parameters->use_batch_update && parameters->batch_max_points > 0) {
    kf.reserve_batch_buffers(parameters->batch_max_points);
    batch_points_lidar_frame.reserve(parameters->batch_max_points);
    batch_points_timestamps.reserve(parameters->batch_max_points);
    batch_points_undistorted.reserve(parameters->batch_max_points);
    batch_points_odom_frame.reserve(parameters->batch_max_points);
}
```

### 6. 参数配置

#### `parameters.h`
```cpp
bool use_batch_update = true;   // 是否启用Batch更新模式
int batch_point_size = 50;      // Batch中的点数（一次性更新的点数）
int batch_max_points = 500;     // Batch最大累积点数（防止内存过大）
```

#### `config/mid360.yaml`
```yaml
# Batch更新参数
use_batch_update: true          # 是否启用Batch点云更新模式
batch_point_size: 100           # Batch中一次性更新的点数
batch_max_points: 500           # Batch最大累积点数（防止内存过大）
```

## 关键特性

### ✅ 优势
1. **批量更新效率高**：减少卡尔曼滤波器更新次数，降低计算开销
2. **点云去畸变**：消除运动畸变，提高点云质量
3. **联合优化**：多个点联合更新状态，理论上精度更高
4. **完全向后兼容**：通过开关控制，不影响原有代码

### 🔒 安全性保证
1. **原代码完全不变**：`use_batch_update=false`时使用原始逻辑
2. **独立实现**：Batch模式与原始模式完全隔离
3. **编译通过**：无警告、无错误
4. **参数可配置**：通过YAML文件灵活调整

### 📊 性能调优建议
- `batch_point_size`：推荐30-100，过小效率低，过大可能降低精度
- `batch_max_points`：防止Batch过大导致内存问题
- 对于高速运动场景，适当减小`batch_point_size`以提高更新频率

## 测试方法

### 1. 验证原始模式不变
```bash
# 修改config/mid360.yaml
use_batch_update: false

# 运行并验证轨迹与修改前完全一致
ros2 launch small_point_lio small_point_lio.launch.py
```

### 2. 测试Batch模式
```bash
# 修改config/mid360.yaml
use_batch_update: true
batch_point_size: 50

# 运行并对比轨迹精度、地图质量、运行时间
ros2 launch small_point_lio small_point_lio.launch.py
```

### 3. 对比指标
- 轨迹精度（与Ground Truth比较）
- 地图质量（点云清晰度、重影程度）
- 运行时间（CPU占用率）
- 协方差一致性（P矩阵是否正定）

## 理论依据

本实现严格遵循BATCH_LIWO论文的数学推导：

- **公式3.44-3.47**：点云去畸变（运动补偿）
- **公式3.48**：Batch观测模型（H和z矩阵堆叠）
- **公式3.62-3.66**：Batch卡尔曼更新（批量状态更新）

## 后续改进方向

1. **自适应Batch大小**：根据运动速度动态调整batch_point_size
2. **鲁棒性增强**：添加离群点检测（RANSAC）
3. **性能优化**：使用稀疏矩阵加速大Batch计算
4. **融合轮速里程计**：实现完整的BATCH_LIWO（目前仅LIO部分）

## 文件修改清单

### 核心修改
- `src/small_point_lio/eskf.h` - 添加Batch数据结构和更新函数
- `src/small_point_lio/estimator.h` - 添加Batch相关成员和函数声明
- `src/small_point_lio/estimator.cpp` - 实现去畸变和Batch观测模型
- `src/small_point_lio/small_point_lio.h` - 添加Batch缓存和双模式函数
- `src/small_point_lio/small_point_lio.cpp` - 实现双模式路由和Batch主循环

### 参数配置
- `src/small_point_lio/parameters.h` - 添加Batch参数声明
- `src/small_point_lio/parameters.cpp` - 添加Batch参数读取
- `config/mid360.yaml` - 添加Batch参数配置

## 编译状态
✅ **编译成功** - 无错误、无警告

## 性能
- [small_point_lio_node-1] [INFO] [1769603643.007134815] [perf]: [📊 0.frame_total_batch] Avg: 0.621 ms, Freq: 250.0 Hz, Samples: 2500, Period: 10.0 s


## 作者
AI Assistant with Human Guidance

## 参考文献
[BATCH_LIWO] Point-LIWO: Batch LiDAR-Inertial-Wheel Odometry
