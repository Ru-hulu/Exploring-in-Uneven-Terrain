cmake_minimum_required(VERSION 2.8.3)
project(hybrida)
# SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    ompl
    )

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collisiondetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/smoother.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamicvoronoi.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bucketedqueue.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bucketedqueue.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/HybridA_interface.cpp
    )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/algorithm.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node3d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/collisiondetection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/path.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/smoother.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vector2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gradient.h #Andrew Noske
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dubins.h #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamicvoronoi.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bucketedqueue.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/point.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/HybridA_interface.hpp
    )

# add_library(hybridA ${SOURCES} ${HEADERS})
add_library(hybrida ${SOURCES})

## Declare a catkin package
catkin_package(
    INCLUDE_DIRS include ${OMPL_INCLUDE_DIRS}
    LIBRARIES hybrida ${OMPL_LIBRARIES}
    CATKIN_DEPENDS roscpp tf
)

# set(OMPL_DIR /home/r/Robot_LIB/omplapp-1.5.1-Source/ompl/)
# message("-----------${OMPL_INCLUDE_DIRS}-------------")
# message("-----------${OMPL_LIBRARIES}-------------")
## OPEN MOTION PLANNING LIBRARY
# find_package(OMPL REQUIRED)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS})

# add_executable(hybrid_astar src/HybridA_interface.cpp ${HEADERS} ${SOURCES})
# target_link_libraries(hybrid_astar ${catkin_LIBRARIES})
# target_link_libraries(hybrid_astar ${OMPL_LIBRARIES})