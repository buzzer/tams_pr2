cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

project(race_tray_monitor)

rosbuild_init()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (PCL_FOUND)
else (PCL_FOUND)
  MESSAGE("PCL 1.7 not found. You must execute the following
    command\nsudo apt-get install libpcl-1.7-all-dev\n")
endif (PCL_FOUND)
              
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/race_tray_monitor.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_link_boost(${PROJECT_NAME} signals)
rosbuild_add_executable(race_tray_monitor src/race_tray_monitor.cpp)
#target_link_libraries(example ${PROJECT_NAME})
SET_TARGET_PROPERTIES(race_tray_monitor PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(race_tray_monitor ${PCL_LIBRARIES})

