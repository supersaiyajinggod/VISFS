#ifndef VISFS_OPTIMIZER_H
#define VISFS_OPTIMIZER_H

#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Parameters.h"
#include "CameraModels/GeometricCamera.h"
#include "Sensor/PointCloud.h"
#include "Map/2d/Submap2D.h"

namespace VISFS {
namespace Optimizer {

struct FeatureBA {
    cv::KeyPoint kpt;
    float depth;
    FeatureBA(const cv::KeyPoint & _kpt, const float _depth) {
        kpt = _kpt;
        depth = _depth;
    }
};

class Optimizer {
public:
    Optimizer(const ParametersMap & _parameters = ParametersMap());
    ~Optimizer(){}

    /** \brief Optimize the local maps. Used in visual, odometry and laser local fusion.
     * \param[in] rootId Fixed pose.
     * \param[in] poses Poses to optimize.
     * \param[in] links The link infomation bwtween poses.
     * \param[in] cameraModels The camera models.
     * \param[in&out] points3D The points in space.
     * \param[in] wordReferences The points3D in images.
     * \param[in] pointClouds The range points from laser.
     * \param[in] submap The submap constructed by several point clouds.
     * \param[out] outliers The outliers of the pair of local point and signature.
     * \author eddy
     */
    std::map<std::size_t, Eigen::Isometry3d> localOptimize(
        std::size_t _rootId,     // fixed pose
        const std::map<std::size_t, Eigen::Isometry3d> & _poses,    // map<pose index, transform>
        const std::map<std::size_t,std::tuple<std::size_t, std::size_t, Eigen::Isometry3d>> & _links,  // map<link index, tuple<the from pose index, the to pose index, transform, infomation matrix>>
        const std::vector<std::shared_ptr<GeometricCamera>> & _cameraModels, // vector camera model left and right
        std::map<std::size_t, std::tuple<Eigen::Vector3d, bool>> & _points3D,   // map<feature id, <feature global pose, fixed?>>
        const std::map<std::size_t, std::map<std::size_t, FeatureBA>> & _wordReferences,
        const std::vector<Sensor::PointCloud> & _pointClouds,
        const std::shared_ptr<const Map::Submap2D> & _submap,
        std::vector<std::tuple<std::size_t, std::size_t>> & _outliers   //  tuple<feature id, signature id>
    );  

private:
    enum Framework {
        G2O     =   0,
        CERES   =   1
    }; 

    int framework_;
    int solver_;
    int trustRegion_;
    int iterations_;
    double pixelVariance_;
    double odometryCovariance_;
    double laserCovariance_;
    double robustKernelDelta_;

};

}   // Optimizer
}   // VISFS

#endif  // VISFS_OPTIMIZER_H