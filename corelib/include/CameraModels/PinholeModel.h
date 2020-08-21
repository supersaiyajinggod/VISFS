#ifndef CAMERAMODELS_PINHOLE_MODEL
#define CAMERAMODELS_PINHOLE_MODEL

#include "CameraModels/GeometricCamera.h"

namespace VISFS {

class PinholeModel : public GeometricCamera {
public:
    PinholeModel(const std::vector<double> & _parameters);
    PinholeModel(const Eigen::Matrix3d & _K, const double & _baseline = 0.05);
    ~PinholeModel();

    cv::Mat cvKfloat() const;
    cv::Mat cvKdouble() const;
    Eigen::Matrix3f eigenKfloat() const;
    Eigen::Matrix3d eigenKdouble() const;
    cv::Mat cvDfloat() const;
    cv::Mat cvDdouble() const;
    float getBaseLine() const;

};

}   // VISFS

#endif  // CAMERAMODELS_PINHOLE_MODEL