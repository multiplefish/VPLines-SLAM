cmake_minimum_required(VERSION 3.0)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3")
add_definitions(-DCOMPILEDWITHC11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    image_transport
    message_filters
    )

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(cvmodified
    src/cvmodified/cvmodified.cpp
    )


add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    )

target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS} cvmodified)

# set(ELSED_PATH ${PROJECT_SOURCE_DIR}/../ELSED)
# find_library(ELSED_LIB elsed ${ELSED_PATH}/build)
# include_directories(${ELSED_PATH}/src)


set(LINE_MATCHING_PATH ${PROJECT_SOURCE_DIR}/../line_matching)
find_library(LINE_MATCHING_LIB line_matching ${LINE_MATCHING_PATH}/build)
include_directories(
    ${LINE_MATCHING_PATH}/src
    ${LINE_MATCHING_PATH}/thirdparty/opencv-3.4
    # /home/fish/vin_ws/src/water-slam/line_matching/thirdparty/opencv-3.4
    )

# add_library(lines_match
#     src/line_match/src/edline_detector.cpp
#     src/line_match/src/klt.cpp
#     src/line_match/src/line_matching.cpp
#     src/line_match/src/lk_tracker_invoker_1d.cpp
#     src/line_match/src/lk_tracker_invoker_2d_single.cpp
#     src/line_match/src/lk_tracker_invoker_2d.cpp
#     src/line_match/src/lk_tracker_invoker_ori.cpp
#     )


add_executable(linefeature_tracker
    src/line_feature_tracker_node.cpp
    src/parameters.cpp
    src/line_feature_tracker.cpp
    src/vanishing_point_detection.cpp
    )


target_link_libraries(linefeature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS}
     cvmodified 
    #  lines_match
    #  ${ELSED_LIB} 
     ${LINE_MATCHING_LIB}
     )

# 可视化
add_executable(image_show
    src/image_show_node.cpp
    src/parameters.cpp
    )
target_link_libraries(image_show ${catkin_LIBRARIES} ${OpenCV_LIBS} 
    ${LINE_MATCHING_LIB}
    )