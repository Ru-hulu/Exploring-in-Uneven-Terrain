cmake_minimum_required(VERSION 2.8.3)
project(bspline)
# SET(CMAKE_BUILD_TYPE Debug) #debug模式
SET(CMAKE_BUILD_TYPE Release) #Releas
set(CMAKE_CXX_FLAGS_RELEASE"-O3")


set(NLOPT_INC_DIRS /usr/local/include)
set(nlopt_lib /usr/local/lib/libnlopt.so)

# find_package(NLopt REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  nav_msgs
  geometry_msgs
  )

file( GLOB SRCS src/bucketedqueue.cpp src/dynamicvoronoi.cpp src/non_uniform_bspline.cpp)
add_library( bspline 
  ${SRCS}
  # ${nlopt_lib}
)

catkin_package(
 INCLUDE_DIRS include ${NLOPT_INC_DIRS}
 LIBRARIES bspline    ${nlopt_lib}
 CATKIN_DEPENDS roscpp
)
# 试着删掉  ${nlopt_lib}， planner_server的编译会报错。

set(Eigen3_INCLUDE_DIRS "/usr/include/eigen3")
include_directories( 
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${NLOPT_INC_DIRS}    
)
