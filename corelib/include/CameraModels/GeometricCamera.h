#ifndef CAMERAMODELS_GEOMETRIC_CAMERA
#define CAMERAMODELS_GEOMETRIC_CAMERA

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace VISFS {

class GeometricCamera {
public:
    GeometricCamera() {
        Eigen::Matrix3d R;
        R << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
        tansformFromImageToRobot_.prerotate(R);
    }

    GeometricCamera(const std::vector<double> & _parameters) : parameters_(_parameters) {
        Eigen::Matrix3d R;
        R << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
        tansformFromImageToRobot_.prerotate(R);
    }

    ~GeometricCamera() {}

    virtual cv::Mat cvKfloat() const = 0;
    virtual cv::Mat cvKdouble() const = 0;
    virtual Eigen::Matrix3f eigenKfloat() const = 0;
    virtual Eigen::Matrix3d eigenKdouble() const = 0;
    virtual cv::Mat cvDfloat() const = 0;
    virtual cv::Mat cvDdouble() const = 0;
    virtual float getBaseLine() const = 0;

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