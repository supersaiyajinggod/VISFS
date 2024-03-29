cmake_minimum_required(VERSION 2.8.0)
# project(VISFSInterfaceROS)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

if(NOT ROS_BUILD_TYPE)
    set(ROS_BUILD_TYPE Release) 
endif(NOT ROS_BUILD_TYPE)
message("Build type: " ${ROS_BUILD_TYPE})


set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -fpermissive")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -w -fpermissive")
set(CMAKE_CXX_FLAGS "-fPIC")

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
    rtabmap_ros
    nav_msgs
)

find_package(Boost 1.65 REQUIRED COMPONENTS
    system
    log
    # thread
    # unit_test_framework
)

find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 REQUIRED)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif(NOT OpenCV_FOUND)
endif()

# find_package(RTABMap 0.19.5 REQUIRED)

set(HEAD_FILE ${PROJECT_SOURCE_DIR}/include)
set(VISFS_HEAD ${PROJECT_SOURCE_DIR}/../../corelib/include)
set(UTILITE_HEAD ${PROJECT_SOURCE_DIR}/../../utilite/include)

include_directories(
    ${HEAD_FILE}
    ${catkin_INCLUDE_DIRS}
    ${VISFS_HEAD}
    ${UTILITE_HEAD}
)

set(LIB_DIR ${PROJECT_SOURCE_DIR}/../../lib)
link_directories(
    ${LIB_DIR}
)

add_library(VISFSInterfaceROS 
    src/InterfaceROS.cpp
    src/MsgConversion.cpp
)
target_link_libraries(VISFSInterfaceROS
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    VISFS_utilite
    VISFS_corelib
)

rosbuild_add_executable(VISFSInterfaceROSNode src/InterfaceROSNode.cpp)
target_link_libraries(VISFSInterfaceROSNode
    VISFSInterfaceROS
    ${Boost_LIBRARIES}
)