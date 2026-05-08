# 雷达位置改变后的问题诊断

## 问题 1：转不动（转不到规划的方向）

### 可能原因

1. **YAW_OFFSET 需要重新校准**
   - 雷达位置改变后，坐标系可能有偏差
   - 需要重新测量朝向偏差

2. **TF 坐标系问题**
   - 雷达 TF 已更新，但其他坐标系没有相应调整

3. **朝向目标无效**
   - RViz goal 的朝向四元数可能为 0（无效）

### 诊断步骤

#### 1. 查看参数是否加载正确
```bash
rosparam get /omnidirectional_pid_local_planner/YAW_OFFSET
# 应该显示一个数值，默认是 0.0
```

#### 2. 查看小车当前朝向和目标朝向
```bash
# 在 RViz 中查看：
# - 小车的朝向箭头（绿色）
# - 目标朝向箭头（红色）

# 或使用命令：
rosrun tf tf_echo map base_link | grep -A 5 "Rotation"
```

#### 3. 启用 DEBUG 日志查看朝向计算
```bash
# 修改 launch 文件，或在代码中改为：
# ROS_DEBUG → ROS_INFO

# 启动后观察日志：
# [robot_pid_local_planner] Goal Yaw: 1.234 rad, offset: -0.5, final: 0.734 rad
```

#### 4. 验证目标朝向是否有效
```bash
# 在 RViz 中点击 "2D Nav Goal"
# 确保：
# 1. 点击位置
# 2. 拖动鼠标设置朝向（看到红色箭头）
# 3. 松开鼠标

# 查看发送的目标：
rostopic echo /move_base_simple/goal -n 1
# 查看 orientation 是否不为 0
```

### 解决方案

如果朝向还是不对，可能需要重新校准 `YAW_OFFSET`：

1. **测量新的偏差**
   - 设置一个目标朝向
   - 观察小车停止后的实际朝向
   - 计算偏差（期望 - 实际）

2. **更新参数**
   ```yaml
   # config/omnidirectional_pid_param.yaml
   YAW_OFFSET: -0.52  # 根据新测量调整
   ```

3. **重新编译和测试**
   ```bash
   catkin_make --pkg robot_pid_local_planner -j4
   bash kill_all.sh
   bash sim_slam.sh  # 或实车启动脚本
   ```

---

## 问题 2：长按 2D Nav Goal 后无法导航

### 可能原因

1. **IDLE 状态锁定**
   - 前一个导航后进入 IDLE（锁定）
   - 新 goal 没有正确解除锁定

2. **状态没有重置**
   - PID 积分堆积
   - 超时计时器没有重置

### 诊断步骤

#### 1. 观察日志
```bash
# 启动节点后观察是否有这样的消息：
# [robot_pid_local_planner] reached goal -> IDLE (latched).
# [robot_pid_local_planner] Resuming from IDLE due to new goal.

# 如果没有看到第二条，说明新 goal 没有正确解除锁定
```

#### 2. 检查目标是否被接收
```bash
rostopic echo /move_base_simple/goal -n 1
# 应该显示新的位置和朝向
```

#### 3. 检查是否有路径
```bash
rostopic echo /opt_path -n 1 | head -20
# 应该显示路径信息，如果为空或报错，说明没有生成路径
```

### 解决方案

已修复：**IDLE 状态现在会检查新 goal 并自动恢复**

如果问题依然存在，尝试：

1. **重新编译**
   ```bash
   catkin_make --pkg robot_pid_local_planner -j4 --cmake-args -DCMAKE_BUILD_TYPE=Debug
   ```

2. **清除构建缓存**
   ```bash
   cd ~/RealSlamRos1
   rm -rf build devel
   catkin_make --pkg robot_pid_local_planner -j4
   ```

3. **重启所有节点**
   ```bash
   bash kill_all.sh
   bash sim_slam.sh  # 或实车脚本
   ```

---

## 综合诊断命令

### 一键诊断脚本
```bash
#!/bin/bash
echo "=== PID Local Planner 诊断 ==="
echo ""
echo "【1】参数检查："
rosparam get /omnidirectional_pid_local_planner/YAW_OFFSET
rosparam get /omnidirectional_pid_local_planner/ENABLE_FINAL_YAW
echo ""
echo "【2】TF 检查："
rosrun tf tf_echo map base_link | grep Rotation
echo ""
echo "【3】话题检查："
echo "Goal: "
rostopic echo /move_base_simple/goal -n 1 | grep -A 2 "orientation"
echo "Path: "
rostopic echo /opt_path -n 1 | head -5
echo ""
echo "【4】小车状态："
rostopic echo /carto_odom -n 1 | grep position
```

### 保存为文件运行
```bash
chmod +x ~/RealSlamRos1/diagnose.sh
~/RealSlamRos1/diagnose.sh
```

---

## 修复总结

### 已修改的代码

1. **IDLE 状态管理**
   - 即使在 IDLE 状态，也会检查新 goal
   - 检测到新 goal 时自动解除锁定
   - 重置所有 PID 和计时器

2. **状态重置**
   - goalCb：重置 yaw_adjust_start_time_
   - onTimer：添加 NaN 检查
   - 增加详细的日志

### 预期行为

✅ 新 goal 到达时立即响应
✅ IDLE 状态不会永久锁定
✅ 朝向计算有详细日志（DEBUG 级别）
✅ 朝向错误时能检测到 NaN

---

## 后续调整建议

如果仍有问题，可以调整：

| 参数 | 说明 | 调整 |
|------|------|------|
| `ENABLE_FINAL_YAW` | 启用终点朝向 | true / false |
| `YAW_TOL` | 朝向容差 | 增大容差（0.15） |
| `YAW_OFFSET` | 朝向偏移校正 | 根据测量调整 |
| `YAW_ADJUST_TIMEOUT` | 最长旋转时间 | 增加时间（10.0） |

重新编译后生效。
