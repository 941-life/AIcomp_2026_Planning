cmake_minimum_required(VERSION 3.0.2)
project(highway_adas_ros)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  gadis_perception_msgs
  geometry_msgs
  highway_adas
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES highway_adas_ros_adapter
  CATKIN_DEPENDS
    gadis_perception_msgs
    geometry_msgs
    highway_adas
    nav_msgs
    roscpp
    rospy
    std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(highway_adas_ros_adapter src/ros_adapter.cpp)
add_dependencies(highway_adas_ros_adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(highway_adas_ros_adapter ${catkin_LIBRARIES})

add_executable(highway_adas_node src/highway_adas_node.cpp)
add_dependencies(highway_adas_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(highway_adas_node highway_adas_ros_adapter ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/highway_region_manager.py
  scripts/synthetic_highway_scene.py
  scripts/verify_synthetic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/run_synthetic_verification.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS highway_adas_ros_adapter highway_adas_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/highway_adas_ros/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
