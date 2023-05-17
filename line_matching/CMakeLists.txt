cmake_minimum_required(VERSION 3.0)
project(LINE_MATCHING)


# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# # set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
# # set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3")
# add_definitions(-DCOMPILEDWITHC11)


# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Release)
# endif()

# # opencv
# find_package(OpenCV 3 REQUIRED)

# include_directories(
#   ${OpenCV_INCLUDE_DIRS}
#   ${PROJECT_SOURCE_DIR}/thirdparty/opencv-3.4
#   )

# file(GLOB_RECURSE LIB_SOURCES "src/*.cpp" "src/*.h")
# add_library(line_matching ${LIB_SOURCES})
# target_link_libraries(line_matching ${OpenCV_LIBS})

# add_executable(line_matching_main src/test_line_matching.cpp)
# target_link_libraries(line_matching_main line_matching ${OpenCV_LIBS})

set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV 3 REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/thirdparty/opencv-3.4
  )
file(GLOB_RECURSE LIB_SOURCES "src/*.cpp" "src/*.h")
# include_directories(src/include)
add_library(line_matching ${LIB_SOURCES})
target_link_libraries(line_matching ${OpenCV_LIBS})

# add_executable(elsed_main src/main.cpp)
# target_link_libraries(elsed_main elsed ${OpenCV_LIBS})




