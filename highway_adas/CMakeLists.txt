cmake_minimum_required(VERSION 3.10)
project(highway_adas LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin QUIET)
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES highway_adas
  )
endif()

add_library(highway_adas
  src/geometry.cpp
  src/object_tracker.cpp
  src/following_controller.cpp
  src/lane_change_planner.cpp
  src/highway_region_policy.cpp
  src/adas_planner.cpp
  src/highway_decision_module.cpp
)

target_include_directories(highway_adas PUBLIC include)

if(MSVC)
  target_compile_options(highway_adas PRIVATE /W4)
else()
  target_compile_options(highway_adas PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_executable(highway_adas_tests test/highway_adas_tests.cpp)
target_link_libraries(highway_adas_tests PRIVATE highway_adas)
add_test(NAME highway_adas_tests COMMAND highway_adas_tests)

# Keep the core test framework-independent while exposing the conventional
# package test target expected by catkin_make.
if(catkin_FOUND)
  add_custom_target(run_tests_${PROJECT_NAME}
    COMMAND $<TARGET_FILE:highway_adas_tests>
    DEPENDS highway_adas_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    USES_TERMINAL
  )
endif()

if(catkin_FOUND)
  install(TARGETS highway_adas
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY include/highway_adas/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
endif()
