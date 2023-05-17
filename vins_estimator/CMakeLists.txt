cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3")
add_definitions(-DCOMPILEDWITHC11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_model
    )

find_package(OpenCV 3.4 REQUIRED)

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  utility/nanoflann
)

catkin_package()

LIST(APPEND SOURCEFILES
        src/parameters.cpp
        src/feature_manager.cpp
        src/factor/pose_local_parameterization.cpp
        src/factor/projection_factor.cpp
        src/factor/marginalization_factor.cpp
        src/utility/utility.cpp
        src/utility/visualization.cpp
        src/utility/CameraPoseVisualization.cpp
        src/initial/solve_5pts.cpp
        src/initial/initial_aligment.cpp
        src/initial/initial_sfm.cpp
        src/initial/initial_ex_rotation.cpp
        src/sophus/common.hpp
        src/sophus/types.hpp
        src/sophus/so3.hpp
        src/utility/line_geometry.h
        src/utility/line_geometry.cpp
        src/factor/line_parameterization.h
        src/factor/line_parameterization.cpp
        src/factor/line_projection_factor.h
        src/factor/line_projection_factor.cpp
        # select
        src/feature_selector.cpp
        src/utility/selector/horizon_generator.cpp
        src/factor/time_factor.cpp


)
ADD_LIBRARY(vinsEstimatorLib SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(vinsEstimatorLib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(vins_estimator
        src/estimator_node.cpp
        src/estimator.cpp
        )
target_link_libraries(vins_estimator vinsEstimatorLib)

