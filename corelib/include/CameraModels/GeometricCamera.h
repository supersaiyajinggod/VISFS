#ifndef CAMERAMODELS_GEOMETRIC_CAMERA
#define CAMERAMODELS_GEOMETRIC_CAMERA

#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace VISFS {

class GeometricCamera {
public:
    GeometricCamera() : tansformFromImageToRobot_(Eigen::Isometry3d::Identity()) {
        Eigen::Matrix3d R;
        R << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
        tansformFromImageToRobot_.prerotate(R);
    }

    GeometricCamera(const std::vector<double> & _parameters) : parameters_(_parameters), tansformFromImageToRobot_(Eigen::Isometry3d::Identity()) {
        Eigen::Matrix3d R;
        R << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
        tansformFromImageToRobot_.prerotate(R);
    }

    ~GeometricCamera() {}

    virtual cv::Mat cvKfloat() const { return cv::Mat(); }
    virtual cv::Mat cvKdouble() const { return cv::Mat(); }
    virtual Eigen::Matrix3f eigenKfloat() const { return Eigen::Matrix3f(); }
    virtual Eigen::Matrix3d eigenKdouble() const { return Eigen::Matrix3d(); }
    virtual cv::Mat cvDfloat() const { return cv::Mat(); }
    virtual cv::Mat cvDdouble() const { return cv::Mat(); }
    virtual float getBaseLine() const { return 0.f; }

    unsigned int getId() { return id_; }
    unsigned int getType() { return type_; }
    Eigen::Isometry3d getTansformImageToRobot() const { return tansformFromImageToRobot_; }


    const unsigned int CAM_PINHOLE = 0;
    const unsigned int CAM_FISHEYE = 1;

    static unsigned int nNextId;

protected:
    std::vector<double> parameters_;    // fx, fy, cx, cy, baseline
    unsigned int id_;
    unsigned int type_;
    Eigen::Isometry3d tansformFromImageToRobot_; // Transform image coordinate frame (x->right, y->down, z->forward) to robot coordinate frame (x->forward, y->left, z->up).
};

}   // namespace

#endif  // CAMERAMODELS_GEOMETRIC_CAMERA