cmake_minimum_required(VERSION 2.8.3)
project(second_project)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_to_laserscan src/pointcloud_to_laserscan.cpp)
add_executable(ugv_odom_to_tf src/ugv_odom_to_tf.cpp)

target_link_libraries( pointcloud_to_laserscan ${catkin_LIBRARIES} )
target_link_libraries( ugv_odom_to_tf ${catkin_LIBRARIES} )



