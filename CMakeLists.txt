cmake_minimum_required(VERSION 2.8.3)
project(pathfinder_ros)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pathfinder_node src/node.cpp)
target_link_libraries(pathfinder_node ${catkin_LIBRARIES})
