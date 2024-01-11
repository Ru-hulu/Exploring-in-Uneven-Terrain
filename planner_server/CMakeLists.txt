cmake_minimum_required(VERSION 2.8.3)
project(planner_server)
# SET(CMAKE_BUILD_TYPE Debug) #debug模式
SET(CMAKE_BUILD_TYPE Release) #Releas
set(CMAKE_CXX_FLAGS_RELEASE"-O3")

set(
  PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  srvbg
  dynamic_reconfigure
  lkh
  hybrida
  octomap_ros
  octomap_msgs
  bspline
  cv_bridge
  )
# # set(NLOPT_INC_DIRS /usr/local/include)
# # set(nlopt_lib /usr/local/lib/libnlopt.so)  
# find_package(NLopt REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)
find_package(OpenCV 4.2.0 REQUIRED)

# message("-!!!----------${NLOPT_INCLUDE_DIRS}-------------")
# message("-!!!----------${NLOPT_LIBRARIES}-------------")

  # ${NLOPT_INCLUDE_DIRS}    
  # ${NLOPT_INC_DIRS}    

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${OCTOMAP_INCLUDE_DIRS}   
  ${OpenCV_INCLUDE_DIRS}  
  )
link_directories(${OpenCV_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)


# set(LINK_LIBS ${NLopt_LIBRARIES} ${nlopt_lib} ${catkin_LIBRARIES}  ${OCTOMAP_LIBRARIES})
set(LINK_LIBS ${catkin_LIBRARIES}  ${OCTOMAP_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(planner_server src/planner.cpp src/spectral_clustering.cpp src/astar_planner.cpp)
target_link_libraries(planner_server ${LINK_LIBS})