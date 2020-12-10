#ifndef VISFS_OPTIMIZER_H
#define VISFS_OPTIMIZER_H

#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Parameters.h"
#include "CameraModels/GeometricCamera.h"

namespace VISFS {

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
        const std::vector<boost::shared_ptr<GeometricCamera>> & _cameraModels, // vector camera model left and right
        std::map<std::size_t, Eigen::Vector3d> & _points3D,
        const std::map<std::size_t, std::map<std::size_t, FeatureBA>> & _wordReferences,
        std::set<std::size_t> & _outliers
    );

    /** \brief Optimize the local maps.
     * \param[in] rootId Fixed pose.
     * \param[in] poses Poses to optimize.
     * \param[in] links The link infomation bwtween poses.
     * \param[in] cameraModels The camera models.
     * \param[in&out] points3D The points in space.
     * \param[in] wordReferences The points3D in images.
     * \param[out] outliers The outliers of points3D.
     * \author eddy
     */
    std::map<std::size_t, Eigen::Isometry3d> localOptimize(
        std::size_t _rootId,     // fixed pose
        const std::map<std::size_t, Eigen::Isometry3d> & _poses,    // map<pose index, transform>
        const std::map<std::size_t,std::tuple<std::size_t, std::size_t, Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> & _links,  // map<link index, tuple<the from pose index, the to pose index, transform, infomation matrix>>
        const std::vector<boost::shared_ptr<GeometricCamera>> & _cameraModels, // vector camera model left and right
        std::map<std::size_t, std::tuple<Eigen::Vector3d, bool>> & _points3D,   // map<feature id, <feature global pose, fixed?>>
        const std::map<std::size_t, std::map<std::size_t, FeatureBA>> & _wordReferences,
        std::set<std::size_t> & _outliers
    );

private:
    int iterations_;
    int solver_;
    int optimizer_;
    double pixelVariance_;
    double robustKernelDelta_;

};

}   // namespace

#endif  // VISFS_OPTIMIZER_H