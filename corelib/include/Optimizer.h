#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "CameraModels/GeometricCamera.h"

namespace VISFS {

struct FeatureBA {
    cv::KeyPoint kpt;
    double depth;
};

class Optimizer {
public:
    Optimizer(){}
    ~Optimizer(){}

    /** \brief Optimize the poses and points with bundle adjustment.
     * \param[in] rootId Fixed pose.
     * \param[in] poses Poses to optimize.
     * \param[in] links The link infomation bwtween poses.
     * \param[in] cameraModels The camera models.
     * \param[in&out] points3D The points in space.
     * \param[in] wordReferences The points3D in images.
     * \param[out] outliers The outliers of points3D.
     * \author eddy
     */
    std::map<std::size_t, Eigen::Isometry3d> poseOptimize(
        std::size_t rootId,     // fixed pose
        const std::map<std::size_t, Eigen::Isometry3d> & _poses,    // map<pose index, transform>
        const std::map<std::size_t,std::tuple<std::size_t, std::size_t, Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> & _links,  // map<link index, tuple<the from pose index, the to pose index, transform, infomation matrix>>
        const std::vector<GeometricCamera> & _cameraModels, // vector camera model left and right
        std::map<std::size_t, Eigen::Vector3d> & _points3D,
        const std::map<std::size_t, std::map<std::size_t, FeatureBA>> & _wordReferences,
        std::set<std::size_t> & _outliers
    );

private:
    int iterations_;
    int optimizer_;
    double pixelVariance_;
    double robustKernelDelta_;

};

}   // namespace

#endif  // OPTIMIZER_H