cmake_minimum_required(VERSION 2.8.3)
project(tangentbug)

set(CMAKE_VERBOSE_MAKEFILE ON)

# catkin dependencies
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    sensor_msgs
    nav_msgs
    roscpp
    tf
)

# boost
find_package(Boost)

catkin_package()

# Build
include_directories(${catkin_INCLUDE_DIRS})

add_executable(setpoint src/setpoint_fixed.cpp)

set_target_properties(setpoint PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")

target_link_libraries(setpoint ${catkin_LIBRARIES})

add_executable(tangentbug src/tangentbug.cpp)

set_target_properties(tangentbug PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")

target_link_libraries(tangentbug ${catkin_LIBRARIES})

