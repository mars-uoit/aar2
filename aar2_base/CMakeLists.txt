cmake_minimum_required(VERSION 2.8.3)
project(aar2_base)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  roboteq_msgs
  roscpp
  std_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(aar2_base src/aar2_base.cpp)

target_link_libraries(aar2_base
   ${catkin_LIBRARIES}
)

