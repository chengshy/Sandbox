cmake_minimum_required(VERSION 2.8.3)
project(pl_test)

find_package(catkin REQUIRED COMPONENTS roscpp tf moveit_ros_planning moveit_ros_move_group)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(pl_test nodes/pl_test.cpp)
target_link_libraries(pl_test ${catkin_LIBRARIES})

