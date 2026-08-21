cmake_minimum_required(VERSION 3.10)
project(highway_adas LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
