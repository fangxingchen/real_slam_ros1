混合控制说明

1. 无障碍时：使用 PID 风格路径跟随。
   - 车头先对齐路径方向，再向前走。
   - 小角度误差会减速但不中断，大角度误差会先原地转向。

2. 发现局部路径走廊内有障碍时：切换到 DWA。
   - 使用局部地图 / ESDF 判断当前局部目标方向是否被动态障碍阻塞。
   - 连续 HYBRID_SWITCH_TO_DWA_COUNT 次阻塞后切到 DWA，连续 HYBRID_SWITCH_TO_PID_COUNT 次畅通后切回 PID，避免反复横跳。

3. 到达终点位置后：继续原地旋转到终点姿态。

建议先调这几个参数：
- PID_TRACK_MAX_SPEED
- PID_YAW_KP
- PID_YAW_STOP_THRESHOLD
- HYBRID_OBSTACLE_DIST_THRESHOLD
- HYBRID_ESDF_BLOCK_THRESHOLD
