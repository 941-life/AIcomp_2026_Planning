cmake_minimum_required(VERSION 3.0.2)
project(global_path_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  katri_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  visualization_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    katri_msgs
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    tf2
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(odom_path_publisher src/odom_path_publisher.cpp)
add_dependencies(odom_path_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_path_publisher ${catkin_LIBRARIES})

add_executable(local_path_selector src/local_path_selector.cpp)
add_dependencies(local_path_selector ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_path_selector ${catkin_LIBRARIES})

add_executable(region_state_publisher src/region_state_publisher.cpp)
add_dependencies(region_state_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(region_state_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})

catkin_install_python(PROGRAMS
  scripts/global_path_publisher.py
  scripts/ll2utm.py
  scripts/path_maker_sm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  odom_path_publisher
  local_path_selector
  region_state_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  map_data
  path_data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
