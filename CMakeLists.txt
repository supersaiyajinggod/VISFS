cmake_minimum_required(VERSION 2.8.5)
project(VISFS)

set(VISFS_MAJOR_VERSION 0)
set(VISFS_MINOR_VERSION 0)
set(VISFS_PATCH_VERSION 1)
set(VISFS_VERSION ${VISFS_MAJOR_VERSION}.${VISFS_MINOR_VERSION}.${VISFS_PATCH_VERSION})

set(PROJECT_VERSION "${VISFS_VERSION}")
set(PROJECT_MAJOR_VERSION ${VISFS_MAJOR_VERSION})
set(PROJECT_MINOR_VERSION ${VISFS_MINOR_VERSION})
set(PROJECT_PATCH_VERSION ${VISFS_PATCH_VERSION})

option(BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON)

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

####### OUTPUT DIR #######
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

####### local cmake modules #######
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED REQUIRED)
find_package(G2O REQUIRED)

message(STATUS "--------------------------------------------")
message(STATUS "Info :")
message(STATUS "  Version : ${RTABMAP_VERSION}")
message(STATUS "  CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  CMAKE_BUILD_TYPE =     ${CMAKE_BUILD_TYPE}")
message(STATUS "  CMAKE_INSTALL_LIBDIR = ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "  CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")
message(STATUS "  FLANN_KDTREE_MEM_OPT = ${FLANN_KDTREE_MEM_OPT}")
message(STATUS "  PCL_DEFINITIONS = ${PCL_DEFINITIONS}")
message(STATUS "  PCL_VERSION = ${PCL_VERSION}")

message(STATUS "")
message(STATUS "Optional dependencies ('*' affects some default parameters) :")

if(OpenCV_FOUND)
	message(STATUS " *With OpenCV               = YES (License: BSD)")
endif(OpenCV_FOUND)

if(G2O_FOUND)
	message(STATUS " *With g2o                  = YES (License: BSD)")
endif(G2O_FOUND)

add_subdirectory(utilite)
add_subdirectory(corelib)
# add_subdirectory(Interface)


