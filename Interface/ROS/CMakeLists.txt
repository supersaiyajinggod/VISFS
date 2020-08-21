cmake_minimum_required(VERSION 2.8.0)
# project(VISFSInterfaceROS)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

if(NOT ROS_BUILD_TYPE)
    set(ROS_BUILD_TYPE Release) 
endif(NOT ROS_BUILD_TYPE)
message("Build type: " ${ROS_BUILD_TYPE})

# Check C++17, C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    dynamic_reconfigure
    std_msgs
    sensor_msgs
    tf
    tf_conversions
    image_transport
    image_geometry
    cv_bridge
    pcl_conversions
    message_filters
)

find_package(Boost 1.65 REQUIRED COMPONENTS
    system
    # thread
    # unit_test_framework
)

find_package(OpenCV 3.2 REQUIRED)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 REQUIRED)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif(NOT OpenCV_FOUND)
endif()

set(HEAD_FILE ${PROJECT_SOURCE_DIR}/include)

include_directories(
    ${HEAD_FILE}
    ${catkin_INCLUDE_DIRS}
)

add_library(VISFSInterfaceROS src/InterfaceROS.cpp)
target_link_libraries(VISFSInterfaceROS
    # ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    # ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

rosbuild_add_executable(VISFSInterfaceROSNode src/InterfaceROSNode.cpp)
target_link_libraries(VISFSInterfaceROSNode
    VISFSInterfaceROS
    ${Boost_LIBRARIES}
)