cmake_minimum_required(VERSION 2.8.3)
project(husky_isa4_nav)

#find_package(catkin REQUIRED roslaunch)
find_package(catkin REQUIRED COMPONENTS
  roslaunch
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  roscpp
  rospy

cv_bridge

)

catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



## For extra emergency stop function. Acts on the move base node
# add_executable(extra_estop src/estop.cpp)
# target_link_libraries(extra_estop ${catkin_LIBRARIES})

## For extra emergency stop function. Acts on the twist_mux node
add_executable(mux_estop src/estop_mux.cpp)
target_link_libraries(mux_estop ${catkin_LIBRARIES})




roslaunch_add_file_check(launch)

install(
  DIRECTORY launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#for the python code of Marker visualization
install(PROGRAMS
  src/publish_lidar_marker.py

  src/img_conv.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/husky_isa4_nav
)
