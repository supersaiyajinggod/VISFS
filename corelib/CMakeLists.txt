set(SRC_FILES
    src/CameraModels/PinholeModel.cpp
    src/Map/ProbabilityValues.cpp
    src/Map/ValueConversionTables.cpp
    src/Map/2d/Grid2d.cpp
    src/Map/2d/ProbabilityGrid.cpp
    src/Map/2d/ProbabilityGridRangeDataInserter2D.cpp
    src/Map/2d/RayToPixelMask.cpp
    src/Map/2d/Submap2D.cpp
    src/Optimizer/Optimizer.cpp
    src/Optimizer/OptimizeTypeDefine.cpp
    src/Estimator.cpp
    src/Extrapolator.cpp
    src/LocalMap.cpp
    src/Monitor.cpp
    src/MultiviewGeometry.cpp
    src/Parameters.cpp
    src/Signature.cpp
    src/System.cpp
    src/Tracker.cpp
)

set(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/utilite/include
)

include_directories(
    ${INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
)

add_library(VISFS_corelib ${SRC_FILES})
target_link_libraries(VISFS_corelib
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${G2O_LIBRARIES}
    VISFS_utilite
)