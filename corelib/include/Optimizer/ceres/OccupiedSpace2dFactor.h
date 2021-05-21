#ifndef VISFS_OPTIMIZER_CERES_OCCUPIEDSPACEFACTOR_H_
#define VISFS_OPTIMIZER_CERES_OCCUPIEDSPACEFACTOR_H_

#include <ceres/ceres.h>

#include "Sensor/PointCloud.h"
#include "Map/2d/Grid2d.h"


namespace VISFS {
namespace Optimizer {

ceres::CostFunction * createOccupiedSpaceCostFunction2d(const double _info, const Sensor::PointCloud & _pointCloud,
        const Map::Grid2D & _grid, const Eigen::Isometry3d _transformImageToRobot);

}   //  Optimizer 
}   // VISFS

#endif  // VISFS_OPTIMIZER_CERES_OCCUPIEDSPACEFACTOR_H_