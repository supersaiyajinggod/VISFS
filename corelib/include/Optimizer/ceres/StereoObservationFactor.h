#ifndef VISFS_OPTIMIZER_STEREOOBSERVATIONFACTOR_H_
#define VISFS_OPTIMIZER_STEREOOBSERVATIONFACTOR_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

namespace VISFS {
namespace Optimizer {

class StereoObservationFactor : public ceres::SizedCostFunction<3, 3, 7> {
public:
    StereoObservationFactor(const double _fx, const double _fy, const double _cx, const double _cy,
            const double _bf, const Eigen::Matrix3d & _info, const Eigen::Vector3d & _obs);

    virtual bool Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const;

    // void check(double ** parameters);

private:
	Eigen::Vector3d project(const Eigen::Vector3d & pc) const {
		const double invZ = 1.0 / pc[2];
		Eigen::Vector3d res;
		res[0] = pc[0] * invZ * fx_ + cx_;
		res[1] = pc[1] * invZ * fy_ + cy_;
		res[2] = res[0] - bf_ * invZ;
		return res;
	}

    double fx_, fy_, cx_, cy_, bf_;
    Eigen::Matrix3d info_;
    Eigen::Vector3d obs_;
};

Eigen::Vector3d reProjectError(const Eigen::Vector3d & _tcw, const Eigen::Quaterniond & _qcw, const Eigen::Vector3d _pw,
        const Eigen::Matrix3d & _K, const double & _baseline, const Eigen::Vector3d & _obs);

}   // Optimizer
}   // VISFS

#endif  // VISFS_OPTIMIZER_STEREOOBSERVATIONFACTOR_H_