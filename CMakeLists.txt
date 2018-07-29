cmake_minimum_required(VERSION 2.8.3)
project(pathfinder_ros)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# Include Pathfinder headers
include_directories(include ${CMAKE_SOURCE_DIR}/pathfinder_ros/lib/pathfinder/include)

add_executable(pathfinder_node src/node.cpp)
target_link_libraries(pathfinder_node ${catkin_LIBRARIES})

# Include Pathfinder lib
IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm") 
    target_link_libraries(pathfinder_node ${CMAKE_SOURCE_DIR}/pathfinder_ros/lib/pathfinder/lib/libpathfinder_arm.a)
ELSE(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    target_link_libraries(pathfinder_node ${CMAKE_SOURCE_DIR}/pathfinder_ros/lib/pathfinder/lib/libpathfinder_x86.a)
ENDIF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")

