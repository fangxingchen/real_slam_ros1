修改内容：
1. 新增 src/main.cpp，补上节点入口，解决 undefined reference to main。
2. 新增 src/pid_follow.cpp 与 src/vel_transform.cpp，补上缺失实现，解决 pid_follow / vel_transform 链接错误。
3. 修正仿真模式下 /truthPose 的订阅类型，改为 nav_msgs/Odometry。
4. 修正 pathCallback 中 firststartPoint[1] 赋值错误。
5. 修正 config/omnidirectional_robot_param.yaml 中 MAX_YAWRATE 与 MAX_D_YAWRATE 为 0 的问题。
6. 同步补齐 CMakeLists.txt 与 package.xml 依赖。
