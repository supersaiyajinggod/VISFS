#include "Signature.h"

namespace VISFS {

std::size_t Signature::nextId_ = 1;

Signature::Signature() : 
    id_(0),
    timestamp_(0.0),
    cameraLeft_(nullptr),
    cameraRight_(nullptr),
    pose_(Eigen::Isometry3d(Eigen::Matrix4d::Zero())),
    guess_(Eigen::Isometry3d(Eigen::Matrix4d::Zero())),
    wheelOdom_(Eigen::Isometry3d(Eigen::Matrix4d::Zero())) {}

Signature::Signature(const double & _timestamp, const cv::Mat & _imageLeft, const cv::Mat & _imageRight, const boost::shared_ptr<GeometricCamera> & _cameraLeft, const boost::shared_ptr<GeometricCamera> & _cameraRight) :
    timestamp_(_timestamp), imageLeft_(_imageLeft), imageRight_(_imageRight), cameraLeft_(_cameraLeft), cameraRight_(_cameraRight) {
    // Set signature id
    id_ = nextId_++;
    pose_ = Eigen::Isometry3d::Identity();

    if (imageLeft_.channels() > 1) {
        cv::Mat temp;
        cv::cvtColor(imageLeft_, temp, cv::COLOR_BGR2GRAY);
        imageLeft_ = temp;
    }
    if (imageRight_.channels() > 1) {
        cv::Mat temp;
        cv::cvtColor(imageRight_, temp, cv::COLOR_BGR2GRAY);
        imageRight_ = temp;
    }
}

Signature::Signature(const double & _timestamp, const cv::Mat & _imageLeft, const cv::Mat & _imageRight, const boost::shared_ptr<GeometricCamera> & _cameraLeft, const boost::shared_ptr<GeometricCamera> & _cameraRight, const Eigen::Isometry3d & _guessPose, const Eigen::Isometry3d & _wheelOdom) :
    timestamp_(_timestamp), imageLeft_(_imageLeft), imageRight_(_imageRight), cameraLeft_(_cameraLeft), cameraRight_(_cameraRight),
    guess_(_guessPose), wheelOdom_(_wheelOdom) {
    // Set signature id
    id_ = nextId_++;

    if (imageLeft_.channels() > 1) {
        cv::Mat temp;
        cv::cvtColor(imageLeft_, temp, cv::COLOR_BGR2GRAY);
        imageLeft_ = temp;
    }
    if (imageRight_.channels() > 1) {
        cv::Mat temp;
        cv::cvtColor(imageRight_, temp, cv::COLOR_BGR2GRAY);
        imageRight_ = temp;
    }
}

void Signature::setPose(const Eigen::Matrix3d & _R, const Eigen::Vector3d & _t) {
    pose_.setIdentity();
    pose_.prerotate(_R);
    pose_.pretranslate(_t);
}

bool Signature::empty() const {
    return imageLeft_.empty() || id_ < 1;
}

}   // namespace