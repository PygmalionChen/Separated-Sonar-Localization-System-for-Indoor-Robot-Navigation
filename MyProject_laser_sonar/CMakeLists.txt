CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set( CMAKE_BUILD_TYPE "Debug" )
PROJECT(MyProject_laser_sonar)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -fopenmp")

#set( CMAKE_BUILD_TYPE "Release" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#set(CMAKE_PREFIX_PATH "/usr/local/opencv3.1/share/OpenCV")
#set(CMAKE_PREFIX_PATH "/home/xjh/library/opencv-3.1_OpenMP/build")

#FIND_PACKAGE(OpenCV 3 REQUIRED)# HINTS "/usr/local/opencv3.1/shared/OpenCV")

#message("include_dir = " ${OpenCV_INCLUDE_DIRS})
# Eigen
include_directories( "/usr/include/eigen3" )

find_package(Boost REQUIRED COMPONENTS system thread)
set(CMAKE_PREFIX_PATH "/usr/local/pcl1.8/share/pcl-1.8")
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization segmentation)

find_package(CSparse REQUIRED)
find_package(Ceres REQUIRED)
message("PCL include_dir = " ${PCL_INCLUDE_DIRS})

include_directories( 
    ${CSPARSE_INCLUDE_DIR}
    ${CERES_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
link_directories(${PCL_LIBRARY_DIRS})
set(THIRD_PARTY_LIBS 
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Boost_LIBRARIES}
        #boost_system
	)
#获得pose
add_executable( test test.cpp )
target_link_libraries( test ${THIRD_PARTY_LIBS} )

add_executable( sonar_process sonar_process.cpp )
target_link_libraries( sonar_process ${THIRD_PARTY_LIBS} )

