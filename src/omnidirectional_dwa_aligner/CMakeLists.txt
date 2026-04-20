cmake_minimum_required(VERSION 3.0.2)
project(omnidirectional_dwa_aligner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
  visualization_msgs
  robot_communication
  robot_costmap
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs tf tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs pcl_conversions pcl_ros robot_communication robot_costmap eigen_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(omnidirectional_dwa_aligner_node
  src/omnidirectional_dwa_aligner.cpp
  src/pid_follow.cpp
  src/vel_transform.cpp
  src/main.cpp
)

add_dependencies(omnidirectional_dwa_aligner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(omnidirectional_dwa_aligner_node
  ${catkin_LIBRARIES}
)

install(TARGETS omnidirectional_dwa_aligner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
