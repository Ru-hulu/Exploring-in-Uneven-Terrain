cmake_minimum_required(VERSION 3.0.2)
project(mpc_solver)

# SET(CMAKE_BUILD_TYPE Debug) #debug模式
SET(CMAKE_BUILD_TYPE Release) #debug模式
set(CMAKE_CXX_FLAGS_RELEASE"-O3")

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

set(qppath "/usr/local/include/qpOASES") 
include_directories(
  include
  qppath
  ${EIGEN3_INCLUDE_DIR}  
  ${catkin_INCLUDE_DIRS}
)


add_executable(mpc_control_node src/mpc_control_node.cpp)
target_link_libraries(mpc_control_node ${catkin_LIBRARIES} qpOASES)

