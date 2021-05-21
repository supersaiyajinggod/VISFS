#include "Optimizer/ceres/StereoObservationFactor.h"
#include "Math.h"

namespace VISFS {
namespace Optimizer {

StereoObservationFactor::StereoObservationFactor(const double _fx, const double _fy, const double _cx, const double _cy,
            const double _bf, const Eigen::Matrix3d & _info, const Eigen::Vector3d & _obs) :
            fx_(_fx), fy_(_fy), cx_(_cx), cy_(_cy), bf_(_bf), info_(_info), obs_(_obs) {
}

bool StereoObservationFactor::Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const {
    const Eigen::Vector3d Pw(parameters[0][0], parameters[0][1], parameters[0][2]);
    const Eigen::Vector3d tcw(parameters[1][0], parameters[1][1], parameters[1][2]);
    const Eigen::Quaterniond qcw(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    const Eigen::Matrix3d R = qcw.toRotationMatrix();
    const Eigen::Vector3d Pc = R * Pw + tcw;
    const double & x = Pc[0];
    const double & y = Pc[1];
    const double & z = Pc[2];
    const double z_2 = z * z;

    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = obs_ - project(Pc);
    residual = info_ * residual;

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobianPw(jacobians[0]);

            jacobianPw(0, 0) = -fx_ * R(0, 0) / z + fx_ * x * R(2, 0) / z_2;
            jacobianPw(0, 1) = -fx_ * R(0, 1) / z + fx_ * x * R(2, 1) / z_2;
            jacobianPw(0, 2) = -fx_ * R(0, 2) / z + fx_ * x * R(2, 2) / z_2;

            jacobianPw(1, 0) = -fy_ * R(1, 0) / z + fy_ * y * R(2, 0) / z_2;
            jacobianPw(1, 1) = -fy_ * R(1, 1) / z + fy_ * y * R(2, 1) / z_2;
            jacobianPw(1, 2) = -fy_ * R(1, 2) / z + fy_ * y * R(2, 2) / z_2;

            jacobianPw(2, 0) = jacobianPw(0, 0) - bf_ * R(2, 0) / z_2;
            jacobianPw(2, 1) = jacobianPw(0 ,1) - bf_ * R(2, 1) / z_2;
            jacobianPw(2, 2) = jacobianPw(0, 2) - bf_ * R(2, 2) / z_2;

            jacobianPw = info_ * jacobianPw;
        }

        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobianTcw(jacobians[1]);

            jacobianTcw(0, 0) = -1. / z * fx_;
            jacobianTcw(0, 1) = 0.;
            jacobianTcw(0, 2) = x / z_2 * fx_;
            jacobianTcw(0, 3) = x * y / z_2 * fx_;
            jacobianTcw(0, 4) = -(1. + (x * x / z_2)) * fx_;
            jacobianTcw(0, 5) = y / z * fx_;
            jacobianTcw(0, 6) = 0.;

            jacobianTcw(1, 0) = 0.;
            jacobianTcw(1, 1) = -1. / z * fy_;
            jacobianTcw(1, 2) = y / z_2 * fy_;
            jacobianTcw(1, 3) = (1. + y * y / z_2) * fy_;
            jacobianTcw(1, 4) = -x * y / z_2 * fy_;
            jacobianTcw(1, 5) = -x / z * fy_;
            jacobianTcw(1, 6) = 0.;

            jacobianTcw(2, 0) = jacobianTcw(0, 0);
            jacobianTcw(2, 1) = 0.;
            jacobianTcw(2, 2) = jacobianTcw(0, 2) - bf_ / z_2;
            jacobianTcw(2, 3) = jacobianTcw(0, 3) - bf_ * y /z_2;
            jacobianTcw(2, 4) = jacobianTcw(0, 4) + bf_ * x / z_2;
            jacobianTcw(2, 5) = jacobianTcw(0, 5);
            jacobianTcw(2, 6) = 0.;

            jacobianTcw.leftCols<6>() = info_ * jacobianTcw.leftCols<6>();
        }
    }

    return true;
}

Eigen::Vector3d reProjectError(const Eigen::Vector3d & _tcw, const Eigen::Quaterniond & _qcw, const Eigen::Vector3d _pw,
        const Eigen::Matrix3d & _K, const double & _baseline, const Eigen::Vector3d & _obs) {
    Eigen::Vector3d error;
    const Eigen::Vector3d pc = _qcw * _pw + _tcw;
    const double invZ = 1. / pc[2];
    error[0] = pc[0] * invZ * _K(0, 0) + _K(0, 2);
    error[1] = pc[1] * invZ * _K(1, 1) + _K(1, 2);
    error[2] = error[0] - _baseline * _K(0, 0) * invZ;
    error = _obs - error;
    return error;
}

}	// Optimizer
}	// VISFS