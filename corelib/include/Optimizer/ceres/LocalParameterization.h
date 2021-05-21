#ifndef VISFS_OPTIMIZER_LOCALPARAMETERIZAYION_H_
#define VISFS_OPTIMIZER_LOCALPARAMETERIZAYION_H_

#include <ceres/ceres.h>

namespace VISFS {
namespace Optimizer {

class PoseLocalParameterization : public ceres::LocalParameterization {
    virtual bool Plus(const double * x, const double * delta, double * x_plus_delta) const;
    virtual bool ComputeJacobian(const double * x, double * jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};

class PointLocalParameterization : public ceres::LocalParameterization {
    virtual bool Plus(const double * x, const double * delta, double * x_plus_delta) const;
    virtual bool ComputeJacobian(const double * x, double * jacobian) const;
    virtual int GlobalSize() const { return 3; };
    virtual int LocalSize() const { return 3; };    
};

}   // Optimizer
}   // VISFS

#endif  // VISFS_OPTIMIZER_LOCALPARAMETERIZAYION_H_