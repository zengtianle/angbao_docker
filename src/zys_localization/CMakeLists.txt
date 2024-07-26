cmake_minimum_required(VERSION 3.0.2)
project(zys_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  message_generation
)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
# set(glog_DIR "/usr/include/glog/")
# find_package(Glog REQUIRED)
include(./glog.cmake)
## Generate services in the 'srv' folder
add_service_files(FILES
  localization_srv.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_DEFINITIONS} )


add_library(ndt_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)




add_executable(zys_localization_node 
  src/zys_localization_node.cpp 
  src/zys_localization.cpp
  src/map.c
  src/map_cspace.cpp
  src/field_map.cc)

add_dependencies(zys_localization_node 
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS}
)

target_link_libraries (zys_localization_node  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}
)

add_executable(processingMaps src/processingMaps.cpp
  src/map.c
  src/map_cspace.cpp
  src/field_map.cc )

add_dependencies(processingMaps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries (processingMaps  ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)


# add_executable(get_initialpose src/get_initialpose.cpp )

# add_dependencies(get_initialpose 
# ${${PROJECT_NAME}_EXPORTED_TARGETS} 
# ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries (get_initialpose  ${catkin_LIBRARIES} ${OpenCV_LIBS} glog

# )




add_executable(test_node
  src/test.cpp )

add_dependencies(test_node 
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS}
)

target_link_libraries (test_node  ${catkin_LIBRARIES} ${OpenCV_LIBS}

)
