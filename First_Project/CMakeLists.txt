cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
# specifichiamo i pacchetti che servono all'interno del nostro package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf
  dynamic_reconfigure
)

#handles dyn conf of lidar
generate_dynamic_reconfigure_options( 
  cfg/dyn_conf_third_node.cfg
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
	roscpp 
	std_msgs 
	sensor_msgs 
	nav_msgs 
	geometry_msgs 
	tf2
  tf
  dynamic_reconfigure
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

# creiamo l'eseguibile dandogli pub come nome
add_executable(gps_to_odom src/gps_to_odom.cpp)
add_executable(odom_to_tf src/odom_to_tf.cpp)
add_executable(lidar_remap src/lidar_remap.cpp)

add_dependencies(lidar_remap ${PROJECT_NAME}_gencfg)

target_link_libraries(gps_to_odom ${catkin_LIBRARIES})
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})
target_link_libraries(lidar_remap ${catkin_LIBRARIES})


