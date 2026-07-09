#!/usr/bin/env bash
# 启动 URDF 可视化演示（完全自包含）

echo "=== 环境准备 ==="

# 1. Source ROS 基础环境
source /opt/ros/jazzy/setup.bash
echo "[OK] ROS Jazzy sourced"

# 2. Source 工作区 overlay
source ~/navigation_teach/install/setup.bash
echo "[OK] Workspace overlay sourced"
ros2 pkg list 2>/dev/null | grep my_tf
echo "[OK] Package my_tf found"

# 3. 修复 snap 库冲突
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
echo "[OK] LD_PRELOAD set: $LD_PRELOAD"

# 4. 关闭残留进程
echo ""
echo "=== 清理旧进程 ==="
pkill -f "joint_state_pub.py" 2>/dev/null && echo "  killed old joint_state_pub" || true
pkill -f "my_tf2_broadcaster" 2>/dev/null && echo "  killed old tf_broadcaster" || true
pkill -f "rviz2" 2>/dev/null && echo "  killed old rviz2" || true
pkill -f "robot_state_publisher" 2>/dev/null && echo "  killed old robot_state_publisher" || true
sleep 1

# 5. 启动节点
echo ""
echo "=== 启动节点 ==="

echo "1) joint_state_publisher..."
ros2 run my_tf joint_state_pub.py > /tmp/jsp.log 2>&1 &
PID1=$!
echo "   PID: $PID1"

sleep 0.8
echo "2) robot_state_publisher (loading URDF)..."
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat ~/navigation_teach/install/my_tf/share/my_tf/urdf/my_diffbot.urdf)" > /tmp/rsp.log 2>&1 &
PID2=$!
echo "   PID: $PID2"

sleep 0.8
echo "3) rviz2 (GUI)..."
rviz2 -d ~/navigation_teach/install/my_tf/share/my_tf/rviz/tf_demo.rviz > /tmp/rviz.log 2>&1 &
PID3=$!
echo "   PID: $PID3"

sleep 0.8
echo "4) my_tf2_broadcaster..."
ros2 run my_tf my_tf2_broadcaster > /tmp/tf.log 2>&1 &
PID4=$!
echo "   PID: $PID4"

echo ""
echo "=== 启动完成 ==="
echo "   joint_state_pub:     PID $PID1"
echo "   robot_state_pub:     PID $PID2"
echo "   rviz2:               PID $PID3"
echo "   my_tf2_broadcaster:  PID $PID4"
echo ""
echo "正在检查进程状态..."
sleep 2

# 检查进程
for proc in rviz2 joint_state_pub robot_state_publisher my_tf2_broadcaster; do
    COUNT=$(ps aux | grep "$proc" | grep -v grep | wc -l)
    if [ "$COUNT" -gt 0 ]; then
        echo "  [RUNNING] $proc"
    else
        echo "  [STOPPED] $proc"
    fi
done

echo ""
echo "日志文件:"
echo "  joint_state_pub:     cat /tmp/jsp.log"
echo "  robot_state_pub:    cat /tmp/rsp.log"
echo "  rviz2:              cat /tmp/rviz.log"
echo "  tf_broadcaster:     cat /tmp/tf.log"
echo ""
echo "关闭请运行:"
echo "  bash ~/navigation_teach/stop_urdf_viz.sh"
