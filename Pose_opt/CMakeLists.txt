CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set( CMAKE_BUILD_TYPE "Release" )
PROJECT(Pose_opt)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -fopenmp")

#list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Eigen
include_directories( "/usr/include/eigen3" )

find_package(Boost REQUIRED COMPONENTS system thread)
set(CMAKE_PREFIX_PATH "/usr/local/pcl1.8/share/pcl-1.8")
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization segmentation)

#find_package(CSparse REQUIRED)
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
	)
# Optimize
add_executable( PoseOptimize PoseOptimize.cpp )
target_link_libraries( PoseOptimize ${THIRD_PARTY_LIBS} )

