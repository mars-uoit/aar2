cmake_minimum_required(VERSION 2.8.3)
project(aar2_base)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  roboteq_msgs
  roscpp
  std_msgs
  tf
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(aar2_base src/aar2_base.cpp)
add_executable(aar2_baseFront src/aar2_baseFront.cpp)
add_executable(aar2_baseMid src/aar2_baseMid.cpp)
add_executable(aar2_baseBack src/aar2_baseBack.cpp)

target_link_libraries(aar2_base
   ${catkin_LIBRARIES}
)
target_link_libraries(aar2_baseFront
   ${catkin_LIBRARIES}
)
target_link_libraries(aar2_baseMid
   ${catkin_LIBRARIES}
)
target_link_libraries(aar2_baseBack
   ${catkin_LIBRARIES}
)

