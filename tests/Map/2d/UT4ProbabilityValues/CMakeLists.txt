cmake_minimum_required(VERSION 3.10)

project(UT4ProbabilityValues)

set(SRC1_DIR ${PROJECT_SOURCE_DIR}/../../../../corelib/src/Map)
set(SRC2_DIR ${PROJECT_SOURCE_DIR})
set(SRC3_DIR ${PROJECT_SOURCE_DIR}/../../../../utilite/src)
set(HEAD_DIR ${PROJECT_SOURCE_DIR}/../../../../corelib/include)
set(UTILITIES_HEAD ${PROJECT_SOURCE_DIR}/../../../../utilite/include)

find_package(Eigen3 3.2.0 REQUIRED)

set(Boost_COMPONENTS unit_test_framework) 
find_package(Boost 1.65 REQUIRED COMPONENTS ${Boost_COMPONENTS})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) 
endif(NOT CMAKE_BUILD_TYPE)
message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -fpermissive")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -w -fpermissive")
set(CMAKE_CXX_FLAGS "-fPIC")

# Check C++20 C++17, C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++20" COMPILER_SUPPORTS_CXX20)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX20)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
   add_definitions(-DCOMPILEDWITHC20)
   message(STATUS "Using flag -std=c++20.")
elseif(COMPILER_SUPPORTS_CXX17)
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

include_directories(
    ${HEAD_DIR}
    ${UTILITIES_HEAD}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
    ${SRC1_DIR}/ProbabilityValues.cpp
    ${SRC2_DIR}/UT4ProbabilityValues.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
)