# omnidirectional_dwa_aligner

这是一个按“全向底盘 DWA + 起始对正 + 终点姿态对正”思路整理出的 ROS1 catkin 功能包。

## 功能说明

这个功能包包含 3 个主要阶段：

1. **起始阶段先转正**
   - 当刚收到目标点时，如果当前车头方向与局部路径切线方向差得太多，先原地旋转。

2. **中间阶段使用 DWA 跟踪**
   - 在 `vx / vy / wz` 三维速度空间采样。
   - 代价函数包含：
     - 到局部目标点距离
     - 与期望速度差值
     - 障碍物代价
     - 路径偏差代价
     - 姿态代价

3. **结束阶段再对终点姿态**
   - 到达终点位置后，不立即结束。
   - 继续原地旋转，直到车身姿态与 RViz 中设置的目标箭头方向一致。

## 订阅话题

- `/move_base_simple/goal`：RViz 目标点
- `/opt_path`：全局路径
- `/odom`：里程计
- `/local_map`：局部占据栅格地图

## 发布话题

- `/cmd_vel_auto`：局部规划输出速度
- `candidate_trajectories`：候选轨迹可视化
- `selected_trajectory`：最优轨迹可视化
- `local_path`：最优局部路径
- `local_goal`：局部目标点可视化

## 编译方法

把该功能包放到 catkin 工作空间的 `src/` 目录下，然后：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 启动方法

```bash
roslaunch omnidirectional_dwa_aligner omnidirectional_dwa_aligner.launch
```

## 说明

这是一个便于阅读和二次开发的版本，代码注释较多，优先强调结构清晰和功能含义。
如果你要直接完全替换你现有工程，还需要按你的真实话题名、地图来源、底盘控制接口再对接一次。
