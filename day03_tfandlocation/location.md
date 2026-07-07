# 建图、定位与 rosbag 实验

这份文档是第三天课程的补充内容，重点不是深入推导 SLAM 算法，而是帮助大家理解：

- 建图在导航系统中解决什么问题；
- 定位算法大致经历了怎样的发展；
- rosbag 在教学和调试中的作用；
- 如何用录制好的传感器数据运行 Point-LIO，并在 RViz 中观察 TF 和定位结果。

本节最后的实践目标是：

> 使用老师提供的 Point-LIO 算法运行录制好的 rosbag，观察 RViz 中的点云、Odometry 和 TF，理解 `odom -> base_link` 是如何产生的。

---

## 1. 建图是什么

建图是指机器人利用传感器（一般是雷达）数据，构建对周围环境的表示。

简单来说，建图回答的是：

> 机器人周围的环境长什么样？

在导航系统中，地图不是为了“好看”，而是为了后续的定位、规划和避障服务。

常见地图类型包括：

| 地图类型 | 常见形式 | 主要用途 |
|---|---|---|
| 2D 栅格地图 | `OccupancyGrid`、`.pgm + .yaml` | 室内导航、AMCL、Nav2 |
| 3D 点云地图 | `.pcd` 点云地图 | 激光定位、NDT、ICP、LIO |
| 局部障碍物地图 | costmap、局部点云 | 避障、局部规划 |
| 拓扑地图 | 点和边组成的路径网络 | 任务决策、站点导航 |

常见建图技术：

- 2D SLAM：GMapping、Cartographer、Hector SLAM；
- 3D LiDAR SLAM：LOAM、LeGO-LOAM、FAST-LIO、Point-LIO；
- 视觉 SLAM：ORB-SLAM、VINS、RTAB-Map；
- 多传感器融合 SLAM：LiDAR + IMU、视觉 + IMU、LiDAR + 轮速。

我们机器人使用的建图技术主要是通过雷达收集点云，然后使用LIO算法导出得到的3D点云（pcd）。通过对pcd文件进行二维处理压成pgm文件，这个pgm文件就是我们用来导航的地图

本课程中只需要先记住：

```text
建图的作用是建立环境参考，定位和规划都依赖这个参考。
```

如果没有地图，机器人可以依靠里程计短时间运动；但如果要在一个已知环境中稳定执行任务，就需要地图作为全局参考。

---

## 2. 定位是什么

定位是指机器人估计自己在某个坐标系下的位置和姿态。

简单来说，定位回答的是：

> 机器人现在在哪里？

在导航中，经常会看到三个容易混淆的概念：

| 概念 | 含义 | 特点 |
|---|---|---|
| Odometry | 里程计，根据连续运动估计当前位置 | 连续、短期好用、长期会漂移 |
| Localization | 在地图中估计机器人位置 | 依赖地图或全局参考 |
| Relocalization | 定位丢失后重新找回位置 | 常用于初始化、机器人被搬动、定位失败恢复 |

## 3. 定位算法的发展

早期移动机器人定位主要依赖编码器和陀螺仪。机器人根据轮子转了多少、转向角变化多少，推算自己移动了多远。这种方法简单、实时性好，但容易受轮子打滑、地面不平、机械误差影响，长期运行会产生累计误差。

后来，机器人开始使用激光雷达和地图进行定位。当时还只有二维激光雷达，只能扫描一个平面，典型方法是 AMCL，它会用粒子滤波估计机器人在 2D 地图中的位置。AMCL 在室内 2D 导航中非常常见，适合平面环境和二维激光雷达。

随着 3D 激光雷达和 IMU 的使用，定位逐渐发展为三维、多传感器融合问题。例如：

- ICP：通过点云之间的几何匹配估计位姿；
- NDT：将点云地图划分成概率分布，再进行匹配；
- LOAM / FAST-LIO / Point-LIO：利用 LiDAR 和 IMU 估计连续运动；
- LiDAR + IMU + 轮速融合：提高复杂运动和短时遮挡下的鲁棒性。

需要注意：

```text
LIO 算法通常更接近 odometry，负责输出连续的 odom -> base_link。
```

它可以在运行过程中构建局部地图，并利用点云匹配减小漂移，但这不等于完整的全局重定位系统。

如果希望机器人长期保持在全局地图中的准确位置，通常还需要额外的全局定位或重定位模块。

---

## 4. 重定位是什么

重定位是指机器人在不知道自己初始位置，或者定位已经丢失的情况下，重新确定自己在地图中的位置。

常见场景：

- 机器人刚开机，不知道自己在地图哪里；
- 机器人被人为搬动；
- 里程计漂移过大；
- 定位算法跟丢；
- 系统重启后需要恢复当前位置。

重定位算法通常会做一件事：

```text
把当前传感器观测和已有地图进行匹配，估计机器人在 map 下的位置。
```

如果系统中同时存在：

```text
odom -> base_link
map -> base_link
```

那么可以计算：

```text
map -> odom = map -> base_link × inverse(odom -> base_link)
```

这样既能保留 `odom -> base_link` 的连续性，又能让机器人在 `map` 下的位置被全局定位修正。

本节实验使用 Point-LIO，重点观察的是：

```text
odom -> base_link
```

也就是说，本实验主要展示连续里程计和 TF，不重点展示 `map -> odom` 的全局修正。

---

## 5. rosbag 的作用

rosbag 可以理解为 ROS 系统里的“数据录像机”。

它可以把真实机器人运行时的话题数据录下来，例如：

- LiDAR 点云；
- IMU 数据；
- TF；
- Odometry；
- 图像；
- 机器人状态。

在教学和调试中，rosbag 很重要，原因是：

- 不需要每次都上实车；
- 所有学员使用同一份数据，实验结果更容易对比；
- 可以重复播放同一段数据；
- 可以慢速播放，方便观察细节；
- 算法修改后可以用同一份数据验证效果。

常用命令：

```bash
ros2 bag info <bag_path>
```

查看 bag 中包含哪些 topic。

```bash
ros2 bag play <bag_path> --clock
```

播放 bag，并发布 `/clock`。

```bash
ros2 bag play <bag_path> --clock -r 0.5
```

以 0.5 倍速播放，适合教学演示和调试。

录制 bag 的基本命令：

```bash
ros2 bag record -o point_lio_raw_bag /livox/lidar /imu /tf_static
```

具体 topic 名称需要根据实际传感器驱动和算法配置决定。Point-LIO 运行时需要的是原始传感器输入，而不是已经处理好的 `/Odometry` 或 `/cloud_registered`。

播放 bag 时要注意：

```text
如果节点使用仿真时间，必须设置 use_sim_time=true。
```

否则节点使用系统时间，而 bag 中的数据使用录制时间，容易导致 TF 查询失败或数据不同步。

---

## 6. Point-LIO 跑 rosbag 实验

### 6.1 实验目标

本实验目标是让大家观察：

- Point-LIO 如何根据 LiDAR 和 IMU 数据输出里程计；
- RViz 中点云如何随机器人运动变化；
- TF 树中 `odom -> base_link` 如何出现；
- `base_link -> livox_frame` 这类静态外参有什么作用。

在当前工程中，Point-LIO 节点主要发布：

```text
/Odometry
/cloud_registered
odom -> base_link
```

launch 文件中还会发布雷达外参：

```text
base_link -> livox_frame
```

所以 RViz 中建议重点观察：

```text
odom -> base_link -> livox_frame
```

### 6.2 实验前检查

进入工作空间，并 source 环境：

```bash
cd /home/li/navigation2026
source install/setup.bash
```

查看 bag 信息：

```bash
ros2 bag info <bag_path>
```

确认 bag 中至少包含 Point-LIO 所需的原始输入，例如：

```text
/livox/lidar
/imu
```

如果 topic 名称和配置文件不一致，需要修改 Point-LIO 配置文件，或者在 launch 中做 remap。

### 6.3 启动 Point-LIO

使用录制数据时，建议开启仿真时间：

```bash
ros2 launch small_point_lio small_point_lio.launch.py use_sim_time:=true
```

如果使用固定安装的雷达，默认参数即可：

```bash
ros2 launch small_point_lio small_point_lio.launch.py use_sim_time:=true lidar_mount_mode:=fixed
```

如果雷达安装在云台上，则需要根据实际系统使用：

```bash
ros2 launch small_point_lio small_point_lio.launch.py use_sim_time:=true lidar_mount_mode:=gimbal_yaw
```

本课程实验如果没有特别说明，默认使用 `fixed` 模式。

### 6.4 播放 rosbag

另开一个终端：

```bash
cd /home/li/navigation2026
source install/setup.bash
ros2 bag play <bag_path> --clock
```

如果数据播放太快，可以降速：

```bash
ros2 bag play <bag_path> --clock
```

### 6.5 打开 RViz

另开一个终端：

```bash
rviz2
```

建议设置：

- Fixed Frame：`odom`
- 添加 `TF`
- 添加 `PointCloud2`，topic 选择 `/cloud_registered`
- 添加 `Odometry`，topic 选择 `/Odometry`

如果点云不显示，优先检查：

- Fixed Frame 是否设置为 `odom`；
- `/cloud_registered` 是否有数据；
- `/tf` 是否存在 `odom -> base_link`；
- `/tf_static` 是否存在 `base_link -> livox_frame`；
- 是否设置了 `use_sim_time=true`；
- rosbag 播放时是否带了 `--clock`。

### 6.6 命令行观察

查看 topic：

```bash
ros2 topic list
```

查看 Point-LIO 输出的里程计：

```bash
ros2 topic echo /Odometry
```

查看点云发布频率：

```bash
ros2 topic hz /cloud_registered
```

查看 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

查看雷达外参：

```bash
ros2 run tf2_ros tf2_echo base_link livox_frame
```

生成 TF 树：

```bash
ros2 run tf2_tools view_frames
```

生成后可以查看输出的 PDF，观察当前 TF 树结构。

---

## 7. 实验中应该观察什么

### 7.1 观察 TF 树

理想情况下，应该能看到类似结构：

```text
odom
 └── base_link
      └── livox_frame
```

其中：

- `odom -> base_link`：Point-LIO 根据传感器数据估计出的机器人运动；
- `base_link -> livox_frame`：雷达安装在机器人上的位置和朝向；
- `/cloud_registered`：Point-LIO 输出的配准后点云。

### 7.2 观察点云

在 RViz 中观察 `/cloud_registered`：

- 点云是否稳定；
- 点云是否跟随机器人运动；
- 点云是否出现明显旋转错误；
- 点云是否整体偏移或倒置。

如果外参错误，点云可能会出现明显异常，例如：

- 前方环境显示到侧面；
- 点云朝向不对；
- 地面倾斜严重；
- 机器人运动时点云抖动明显。


---

## 8. 常见问题

### 8.1 RViz 中没有点云

优先检查：

```bash
ros2 topic list
ros2 topic hz /cloud_registered
ros2 run tf2_ros tf2_echo odom base_link
```

常见原因：

- Point-LIO 没有正常输出点云；
- Fixed Frame 设置错误；
- TF 断链；
- 没有开启 `use_sim_time`；
- bag 播放没有加 `--clock`。

### 8.2 TF 中没有 `odom -> base_link`

说明 Point-LIO 可能没有正常输出里程计。

检查：

- LiDAR topic 是否正确；
- IMU topic 是否正确；
- 配置文件中的话题名是否和 bag 中一致；
- Point-LIO 终端是否有报错；
- 时间戳是否正常。

### 8.3 TF 中没有 `base_link -> livox_frame`

说明雷达外参没有发布。

检查：

- launch 是否正常启动；
- `lidar_mount_mode` 是否设置正确；
- `static_transform_publisher` 是否运行；
- frame 名称是否和点云消息里的 `frame_id` 一致。

### 8.4 点云显示了，但姿态明显不对

可能原因：

- 雷达外参错误；
- IMU 坐标系方向错误；
- 雷达安装角度和 launch 中参数不一致；
- bag 数据与当前配置不匹配。

---

## 9. 作业

使用老师提供的 Point-LIO 算法和 rosbag(ros2 bag play livox_bag_20260523_183327/livox_bag_20260523_183327_0.mcap
)，完成一次定位实验。

要求提交：

1. RViz 截图，截图中需要显示：
   - TF；
   - `/cloud_registered`；
   - `/Odometry`。

2. TF 树截图或 `view_frames` 生成结果，说明至少以下两段 TF：
   - `odom -> base_link`；
   - `base_link -> livox_frame`。

3. 简短回答以下问题：
   - Point-LIO 发布的是 `map -> odom` 还是 `odom -> base_link`？
   - `odom -> base_link` 为什么通常是连续的？
   - `base_link -> livox_frame` 表示什么？
   - 如果 RViz 中点云不显示，你会优先检查哪些内容？

4. 如果实验过程中遇到问题，记录：
   - 现象；
   - 你检查了哪些 topic 或 TF；
   - 最后如何解决。

验收标准：

- 能成功运行 Point-LIO 和 rosbag；
- RViz 中能看到点云和 TF；
- 能解释 `odom -> base_link -> livox_frame` 的含义；
