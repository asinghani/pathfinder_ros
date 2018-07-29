cmake_minimum_required(VERSION 2.8.3)
project(pathfinder_ros)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs message_generation)

include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
    FILES
    Path.msg
    PathSegment.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

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

install(PROGRAMS
    scripts/path_to_waypoints.py
    scripts/simple_path_to_nav_goal.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
