#ifndef VISFS_OPTIMIZER_POSECONSTRAINTFACTOR_H_
#define VISFS_OPTIMIZER_POSECONSTRAINTFACTOR_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

namespace VISFS {
namespace Optimizer {

class PoseConstraintFactor : public ceres::SizedCostFunction<6, 7, 7> {
public:
    PoseConstraintFactor(const Eigen::Matrix<double, 6, 6> & _info, const Eigen::Vector3d & _obsTranslation, const Eigen::Quaterniond & _obsRotation);

    virtual bool Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const;
    
private:
    Eigen::Matrix<double, 6, 6> info_;
    //  Transform from 2 to 1.
    Eigen::Vector3d obsTranslation_;    // T12
    Eigen::Quaterniond obsRotation_;    // R12
};

}   // Optimizer
}   // VISFS

#endif  // VISFS_OPTIMIZER_POSECONSTRAINTFACTOR_H_