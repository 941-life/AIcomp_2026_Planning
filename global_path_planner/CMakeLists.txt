cmake_minimum_required(VERSION 3.0.2)
project(global_path_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  katri_msgs
  nav_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  visualization_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES following_planner highway_planner_core
  CATKIN_DEPENDS
    geometry_msgs
    katri_msgs
    nav_msgs
    roscpp
    roslib
    rospy
    sensor_msgs
    std_msgs
    tf
    tf2
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(highway_planner_core src/highway_planner_core.cpp)
add_dependencies(highway_planner_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(highway_planner_core ${catkin_LIBRARIES})

add_library(following_planner src/following_planner.cpp)
add_dependencies(following_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(following_planner ${catkin_LIBRARIES})

add_executable(odom_path_publisher src/odom_path_publisher.cpp)
add_dependencies(odom_path_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_path_publisher ${catkin_LIBRARIES})

add_executable(local_path_selector src/local_path_selector.cpp)
add_dependencies(local_path_selector ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_path_selector ${catkin_LIBRARIES})

add_executable(region_state_publisher src/region_state_publisher.cpp)
add_dependencies(region_state_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(region_state_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(highway_planner_node src/highway_planner_node.cpp)
add_dependencies(highway_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  highway_planner_node
  following_planner
  highway_planner_core
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(highway_planner_core_test test/highway_planner_core_test.cpp)
  if(TARGET highway_planner_core_test)
    target_link_libraries(highway_planner_core_test highway_planner_core ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(following_planner_test test/following_planner_test.cpp)
  if(TARGET following_planner_test)
    target_link_libraries(following_planner_test following_planner ${catkin_LIBRARIES})
  endif()
endif()

catkin_install_python(PROGRAMS
  scripts/control_status_adapter.py
  scripts/gps_imu_fusion.py
  scripts/global_path_publisher.py
  scripts/ll2utm.py
  scripts/path_maker_sm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  following_planner
  highway_planner_core
  highway_planner_node
  odom_path_publisher
  local_path_selector
  region_state_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  map_data
  path_data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
