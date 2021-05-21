#include "Optimizer/ceres/PoseConstraintFactor.h"
#include "Math.h"

namespace VISFS {
namespace Optimizer {

PoseConstraintFactor::PoseConstraintFactor(const Eigen::Matrix<double, 6, 6> & _info, const Eigen::Vector3d & _obsTranslation, const Eigen::Quaterniond & _obsRotation) :
    info_(_info), obsTranslation_(_obsTranslation), obsRotation_(_obsRotation) {
}

bool PoseConstraintFactor::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const {
    const Eigen::Vector3d sP1(parameters[0][0], parameters[0][1], parameters[0][2]);
    const Eigen::Quaterniond sQ1(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    const Eigen::Vector3d sP2(parameters[1][0], parameters[1][1], parameters[1][2]);
    const Eigen::Quaterniond sQ2(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    const Eigen::Vector3d & mP12 = obsTranslation_;
    const Eigen::Quaterniond & mQ12 = obsRotation_;

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual.block<3, 1>(0, 0) = sQ1*sQ2.inverse()*(-sP2) + sP1 - mP12;
    residual.block<3, 1>(3, 0) = 2 * (mQ12.inverse()*sQ1*sQ2.inverse()).vec();
    residual = info_ * residual;

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobianPose1(jacobians[0]);
            jacobianPose1.setZero();

            jacobianPose1.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            jacobianPose1.block<3, 3>(0, 3) = -skewSymmetric(sQ1*(sQ2.inverse()*(-sP2)));
            jacobianPose1.block<3, 3>(3, 3) = (QuaternionLeft(sQ2*sQ1.inverse())*QuaternionRight(mQ12)).bottomRightCorner<3, 3>();    
            jacobianPose1 = info_ * jacobianPose1;         
        }

        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobianPose2(jacobians[1]);
            jacobianPose2.setZero();
            
            jacobianPose2.block<3, 3>(0, 0) = -(sQ1*sQ2.inverse()).toRotationMatrix();
            jacobianPose2.block<3, 3>(0, 3) = sQ1.toRotationMatrix()*sQ2.inverse().toRotationMatrix()*skewSymmetric(-sP2);
            jacobianPose2.block<3, 3>(3, 3) = -(QuaternionLeft(mQ12.inverse()*sQ1*sQ2.inverse())).bottomRightCorner<3, 3>();
            jacobianPose2 = info_ * jacobianPose2;
        }
    }

    return true;
}

}	// Optimizer
}	// VISFS