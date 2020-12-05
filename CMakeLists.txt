cmake_minimum_required(VERSION 3.5)
project(gbot_core)

find_package(ament_cmake REQUIRED)
find_package(cartographer_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY configuration_files launch rviz urdf
  DESTINATION share/${PROJECT_NAME}/
 )
 
ament_package()
