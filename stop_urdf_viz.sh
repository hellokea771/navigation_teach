#!/usr/bin/env bash
# 关闭 URDF 可视化演示
echo "Stopping URDF visualization..."
pkill -f "joint_state_pub.py" 2>/dev/null || true
pkill -f "my_tf2_broadcaster" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
sleep 1
echo "Done."
# 检查是否还有残留
REMAINING=$(ps aux | grep -E 'rviz2|joint_state_pub|robot_state_pub|my_tf2_broadcaster' | grep -v grep | wc -l)
if [ "$REMAINING" -eq 0 ]; then
    echo "All processes stopped."
else
    echo "Warning: $REMAINING processes may still be running."
fi
