# 重力对齐与坐标变换说明

## 问题背景

LIO算法以第一帧IMU位姿定义 `lidar_odom` 坐标系。当机器人底盘倾斜或外参标定有偏差时，这个坐标系的Z轴不严格指向天空，导致建图结果倾斜。

## 坐标系定义

```
┌─────────────────────────────────────────────────────────────┐
│  lidar_odom (LIO内部)    第一帧IMU位姿定义，Z轴可能倾斜      │
│       ↓                                                     │
│  q_gravity_align_        重力对齐旋转（纯旋转，无平移）       │
│       ↓                                                     │
│  odom                    Z轴严格竖直向上的世界坐标系         │
│       ↓                                                     │
│  base_link               车体中心坐标系                      │
└─────────────────────────────────────────────────────────────┘
```

## 两个变换的作用

| 变换 | 作用 | 来源 |
|------|------|------|
| `q_gravity_align_` | 将倾斜的 `lidar_odom` 旋转成水平的 `odom` | IMU静止测量计算 |
| `tf_base_link_to_lidar_` | lidar/IMU 与车体中心的相对位姿 | TF树（launch发布） |

## 重力对齐原理

### IMU测量的物理意义

- IMU静止时测量的加速度 **永远指向真实的天空**（不管IMU怎么安装）
- 这个测量值在IMU自身坐标系下的表示，取决于IMU的安装姿态

### 场景示例

**场景1：IMU水平安装**
```
IMU测到: [0, 0, 9.81]  ← 在IMU坐标系下，Z轴向上
说明：IMU的Z轴 = 真实向上
```

**场景2：IMU倾斜45度安装**
```
IMU测到: [0, 6.94, 6.94]  ← 在IMU坐标系下
说明：真实向上 = IMU的 Y+Z 方向（斜着）
```

### 计算方法

```cpp
// measured_gravity = IMU测到的加速度平均值（指向真实天空）
Eigen::Vector3d gravity_dir = measured_gravity.normalized();

// 目标：odom系的Z轴向上 [0, 0, 1]
Eigen::Vector3d target_up(0.0, 0.0, 1.0);

// 计算旋转：把 gravity_dir 对齐到 target_up
Eigen::Quaterniond q_align = Eigen::Quaterniond::FromTwoVectors(gravity_dir, target_up);
```

## 代码中的变换流程

### Odometry回调

```cpp
// LIO输出：T_lidar_odom→lidar (lidar在lidar_odom系下的位姿)
Eigen::Vector3f pos_lidar_odom = odometry.position;
Eigen::Quaternionf q_lidar_odom = odometry.orientation;

// Step 1: 应用重力对齐，得到 lidar 在水平 odom 系下的位姿
// T_odom→lidar = R_gravity_align * T_lidar_odom→lidar
Eigen::Vector3f pos_lidar_in_odom = q_gravity_align_ * pos_lidar_odom;
Eigen::Quaternionf q_lidar_in_odom = q_gravity_align_ * q_lidar_odom;

// Step 2: 应用外参旋转，得到 base_link 的姿态
// q_base_in_odom = q_lidar_in_odom * q_base_to_lidar
// 注意：q_base_to_lidar = q_lidar_to_base.inverse()
Eigen::Quaternionf q_lidar_to_base(T_lidar_to_base_.rotation());
Eigen::Quaternionf q_base_to_lidar = q_lidar_to_base.inverse();
Eigen::Quaternionf q_base_in_odom = q_lidar_in_odom * q_base_to_lidar;

// Step 3: 计算 base_link 的位置（考虑外参平移）
// pos_base = pos_lidar + q_lidar * translation_lidar_to_base
Eigen::Vector3f translation_lidar_to_base = T_lidar_to_base_.translation();
Eigen::Vector3f pos_base_in_odom = pos_lidar_in_odom + q_lidar_in_odom * translation_lidar_to_base;
```

### Pointcloud回调

```cpp
// 点云已经在 lidar_odom 系下（LIO内部已经拼接好）
for (const auto &point : pointcloud) {
    // 只需要应用重力对齐，把点云旋转到水平的 odom 系
    Eigen::Vector3f aligned_point = q_gravity_align_ * point;
}
// frame_id = "odom"
```

**注意**：点云不需要外参变换，因为点云是**全局地图**，表示在 `odom` 系下。

## 实现细节

### 初始化流程

1. **外参获取** (TF 轮询，100ms 周期)：
   ```cpp
   // 从 TF 树获取 lidar_frame ← base_link
   tf_buffer->lookupTransform(lidar_frame, "base_link", tf2::TimePointZero);
   // 求逆得到 base_link ← lidar_frame
   T_lidar_to_base_ = T_base_to_lidar.inverse();
   ```

2. **IMU 数据收集** (200 个样本)：
   ```cpp
   if (gravity_alignment_enabled_ && !gravity_alignment_done_) {
       imu_accel_buffer_.push_back(imu_msg.linear_acceleration);
       if (imu_accel_buffer_.size() >= GRAVITY_INIT_SAMPLES) {
           // 计算平均加速度
           Eigen::Vector3d avg_accel = 均值(imu_accel_buffer_);
           computeGravityAlignment(avg_accel);
       }
   }
   ```

3. **对齐四元数计算**：
   ```cpp
   Eigen::Vector3d gravity_dir = measured_gravity.normalized();
   Eigen::Vector3d target_up(0.0, 0.0, 1.0);
   q_gravity_align_ = Quaterniond::FromTwoVectors(gravity_dir, target_up);
   ```

### 运行时处理

- **Odometry 回调**：应用重力对齐 + 外参变换，发布 `odom → base_link` TF
- **Pointcloud 回调**：仅应用重力对齐，发布 `odom` 系下的点云
- **初始化完成前**：跳过所有发布，等待 `gravity_alignment_done_ == true`

### 性能优化

- 外参只在启动时查询一次，缓存后直接使用
- IMU 数据收集期间不发布 TF/Odometry（避免不稳定数据）
- 重力对齐计算仅执行一次，后续直接应用缓存的四元数

## 数学推导

设：

- $T_{lo}^{l}$ = LIO输出的 lidar_odom → lidar 变换
- $R_g$ = 重力对齐旋转（`q_gravity_align_`）
- $T_l^b$ = lidar → base_link 外参
- $q_b^l$ = base_link → lidar 的旋转（`q_base_to_lidar`）

目标：求 $T_o^b$（odom → base_link）

**推导**：

```math
T_o^l = R_g · T_{lo}^{l}                    // Step 1: 应用重力对齐
q_o^b = q_o^l · q_b^l                       // Step 2: 链式旋转
pos_o^b = pos_o^l + q_o^l · trans_l^b       // Step 3: 位置变换
```

### 关键理解：为什么是 q_base_to_lidar？

当 lidar 倾斜安装（如 roll=-45°）时：

```text
初始状态（机器人静止在水平面）:
  - q_lidar_odom ≈ identity (LIO初始化时刻)
  - q_gravity_align ≈ +45° roll (补偿lidar倾斜)
  - q_base_to_lidar ≈ -45° roll (外参，base→lidar的安装角度)
  
计算过程:
  q_lidar_in_odom = q_gravity_align * identity ≈ +45° roll
  q_base_in_odom = q_lidar_in_odom * q_base_to_lidar
                 ≈ (+45°) * (-45°) ≈ identity ✓
```

**结果**：base_link 水平（符合预期，因为机器人底盘在水平面上）

**错误方案**（直接链乘 T_lidar_to_base）：
```text
q_base_in_odom = q_lidar_in_odom * q_lidar_to_base
               ≈ (+45°) * (+45°) = +90° roll ❌
结果：base_link 的 Z 轴指向侧方（错误！）
```

## 参数配置

```yaml
# config/mid360.yaml
small_point_lio:
    ros__parameters:
        gravity_alignment: true   # 启用重力对齐（推荐）
                                  # false时，odom系 = lidar_odom系（可能倾斜）
```

## 常见问题 FAQ

### Q1: 为什么不直接使用 q_lidar_to_base？

**A**: 坐标变换的方向性至关重要：

- `T_lidar_to_base_`：描述"lidar 坐标系到 base_link 坐标系"的变换
- 位姿链乘：描述"物体在不同坐标系下的姿态"

当我们说 "base_link 的姿态 = lidar 的姿态 × 某个旋转" 时：
- 这个"某个旋转"应该是 **base_link 相对于 lidar 的旋转**
- 即 `q_base_to_lidar = q_lidar_to_base.inverse()`

**物理直觉**：
- lidar 倾斜 +45°，base_link 相对于它旋转 -45°，最终 base_link 水平
- 公式：`q_base_in_world = q_lidar_in_world * q_base_relative_to_lidar`

### Q2: 为什么机器人启动时必须静止？

**A**: 重力对齐依赖 IMU 测量重力方向：

- IMU 静止时：测量值 ≈ 重力加速度（指向天空）
- IMU 运动时：测量值 = 重力 + 运动加速度（方向不确定）

如果启动时移动机器人，会导致：
- 测量的"重力方向"偏离真实值
- `q_gravity_align_` 计算错误
- odom 系仍然倾斜

### Q3: 如何验证重力对齐是否正确？

**A**: 检查以下几点：

1. **日志检查**：
   ```log
   Aligned 'up' direction in odom: [0.0000, 0.0000, 1.0000]
   ```
   应该非常接近 `[0, 0, 1]`

2. **RViz 可视化**：
   - Fixed Frame 设为 `odom`
   - 查看点云，地面应该水平（Z=0 平面）
   - 查看 TF 轴，`odom` 的 Z 轴（蓝色）应该指向天空

3. **调试日志**：
   - `q_base_in_odom` 在机器人静止时应该接近 identity
   - 即：`w ≈ 1.0, x ≈ 0, y ≈ 0, z ≈ 0`

### Q4: gravity_alignment 参数何时设为 false？

**A**: 以下场景可以禁用：

- 仿真环境，IMU 数据完美无噪声
- 外参标定非常精确，lidar/IMU 本身就水平安装
- 需要与 lidar_odom 系直接对接的下游算法

**大部分实际场景建议保持 `true`**。

## 使用注意事项

⚠️ **重要**：机器人启动时必须**静止**几秒钟，让系统收集200帧IMU数据计算重力方向！

## 运行时日志

启动后会看到：

```log
[INFO] Collecting IMU data for gravity alignment: 50/200
[INFO] Collecting IMU data for gravity alignment: 100/200
[INFO] Collecting IMU data for gravity alignment: 150/200
[INFO] Collecting IMU data for gravity alignment: 200/200
[INFO] Gravity alignment computed:
[INFO]   Measured 'up' direction in lidar_odom: [0.0123, -0.0087, 0.9999]
[INFO]   Alignment quaternion (lidar_odom → odom): [w=0.9999, x=0.0043, y=0.0061, z=0.0000]
[INFO]   Aligned 'up' direction in odom: [0.0000, 0.0000, 1.0000]
```

运行过程中每100帧会输出调试信息：

```log
[INFO] DEBUG transforms:
[INFO]   q_gravity_align: [w=0.921, x=-0.388, y=-0.022, z=0.000]
[INFO]   q_lidar_odom (LIO output): [w=0.999, x=0.001, y=-0.002, z=0.003]
[INFO]   q_base_to_lidar (extrinsic): [w=0.924, x=-0.383, y=0.000, z=0.000]
[INFO]   q_lidar_in_odom: [w=0.920, x=-0.389, y=-0.021, z=0.002]
[INFO]   q_base_in_odom (final): [w=0.999, x=-0.003, y=-0.002, z=0.001]
```

解读：

- `q_gravity_align`：约 45° roll，补偿 lidar 倾斜安装
- `q_base_to_lidar`：约 -45° roll，外参安装角度
- `q_base_in_odom`：接近 identity，base_link 保持水平 ✓

## 时序图

```mermaid
┌──────────┐     ┌──────────┐     ┌──────────────┐     ┌──────────┐
│   IMU    │────→│   LIO    │────→│ ROS Wrapper  │────→│    TF    │
│ 数据     │     │ 算法     │     │ 变换处理      │     │ 发布     │
└──────────┘     └──────────┘     └──────────────┘     └──────────┘
     │                │                  │                  │
     │ 前200帧 ──────────────→ 计算q_gravity_align_         │
     │                │                  │                  │
     │                │ T_lidar_odom^lidar                  │
     │                ├─────────────────→│                  │
     │                │                  │                  │
     │                │      应用重力对齐 + 外参              │
     │                │                  ├─────────────────→│
     │                │                  │    odom→base_link│
```
