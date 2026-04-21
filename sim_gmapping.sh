### 仿真环境下创建地图

# 1) 启动 roscore
gnome-terminal --title="roscore" -- bash -c "roscore"
# 等待 roscore 就绪
until rostopic list >/dev/null 2>&1; do
  sleep 0.2
done
# 2) 启动 Gazebo + robot
gnome-terminal --title="gazebo_start" -- bash  -c "source ~/RealSlamRos1/devel/setup.bash; roslaunch tri_steer_gazebo carto_construct_map.launch"
# 等待 Gazebo 节点起来（/gazebo/model_states 出现再继续）
until rostopic list 2>/dev/null | grep -q "/gazebo/model_states"; do
  sleep 0.5
done
# 3) 启动 teleop + robot
gnome-terminal -t "teleop_start" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch tri_steer_keyboard keyboard_tri_steer.launch"
# 4) 雷达点云反转成水平车体下的point-cloud2
gnome-terminal -t "laser to cloud" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch read_laser_data laser_to_cloud.launch; exec bash"
# 5）终极修正版：手动强行合并两个工作空间的环境变量
gnome-terminal -t "carto gmapping" -- bash -c "source /home/ubuntu/Cartographer/Cartographer_Locatization/install_isolated/setup.bash; export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/home/ubuntu/RealSlamRos1/src; echo '检查合并后的路径：'; echo \$ROS_PACKAGE_PATH; roslaunch cartographer_ros my_robot_2d.launch; exec bash"