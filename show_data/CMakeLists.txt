cmake_minimum_required(VERSION 3.0.2)
project(show_data)

SET(CMAKE_BUILD_TYPE Debug) #debug模式
# SET(CMAKE_BUILD_TYPE Release) #debug模式

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  gazebo_msgs 
  visualization_msgs
  srvbg
  )
find_package(Eigen3)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpc_node
#  CATKIN_DEPENDS roscpp std_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}  
  ${catkin_INCLUDE_DIRS}
)


add_executable(show_data_node src/show_data.cpp)
target_link_libraries(show_data_node ${catkin_LIBRARIES})

