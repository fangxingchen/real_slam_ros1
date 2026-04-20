# 1) 保存地图 (引号修复 + 路径正确) mape是实际建图  mapf是仿真建图
gnome-terminal -t "carto gmapping save" -- bash -c "cd /home/ubuntu/Cartographer/Cartographer_Locatization && source install_isolated/setup.bash && rosservice call /write_state \"{filename: '/home/ubuntu/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mape.pbstream'}\"; exec bash"
sleep 5

# 2）转换 pbstream → 栅格地图 保存到 Cartographer 目录
gnome-terminal -t "pbstream to pgm" -- bash -c "cd /home/ubuntu/Cartographer/Cartographer_Locatization && source install_isolated/setup.bash && rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/ubuntu/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mape.pbstream -map_filestem=/home/ubuntu/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mape -resolution=0.1; exec bash"
sleep 5

# 3）转换 pbstream → 栅格地图 保存到 tri_steer_gazebo/map
gnome-terminal -t "pbstream to pgm2" -- bash -c "cd /home/ubuntu/Cartographer/Cartographer_Locatization && source install_isolated/setup.bash && rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/ubuntu/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mape.pbstream -map_filestem=/home/ubuntu/RealSlamRos1/src/tri_steer_gazebo/map/carto_mape -resolution=0.1; exec bash"

# 4）转换 pbstream → 栅格地图 保存到 robot_costmap/maps
gnome-terminal -t "pbstream to pgm3" -- bash -c "cd /home/ubuntu/Cartographer/Cartographer_Locatization && source install_isolated/setup.bash && rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/ubuntu/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mape.pbstream -map_filestem=/home/ubuntu/RealSlamRos1/src/robot_costmap/maps/carto_mape -resolution=0.1; exec bash"
