set(SRC_FILES
    src/Conversion.cpp
    src/Math.cpp
    src/ProcessInfo.cpp
    src/Timer.cpp
)

set(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

include_directories(
    ${INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(VISFS_utilite ${SRC_FILES})