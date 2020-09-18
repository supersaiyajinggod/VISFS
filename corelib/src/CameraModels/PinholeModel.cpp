#include "CameraModels/PinholeModel.h"

namespace VISFS {

unsigned int GeometricCamera::nNextId = 0;

PinholeModel::PinholeModel(const std::vector<double> & _parameters) : GeometricCamera(_parameters) {
    id_ = nNextId++;
    type_ = CAM_PINHOLE;
}

PinholeModel::PinholeModel(const Eigen::Matrix3d & _K, const double & _baseline) : GeometricCamera() {
    id_ = nNextId++;
    type_ = CAM_PINHOLE;

    parameters_.emplace_back(_K(0, 0));
    parameters_.emplace_back(_K(1, 1));
    parameters_.emplace_back(_K(0, 2));
    parameters_.emplace_back(_K(1, 2));
    parameters_.emplace_back(_baseline);
}

cv::Mat PinholeModel::cvKfloat() const {
    cv::Mat K = (cv::Mat_<float>(3, 3)
                <<  static_cast<float>(parameters_[0]), 0.f, static_cast<float>(parameters_[2]),
                    0.f, static_cast<float>(parameters_[1]), static_cast<float>(parameters_[3]),
                    0.f, 0.f, 1.f);
    return K;
}

cv::Mat PinholeModel::cvKdouble() const {
    cv::Mat K = (cv::Mat_<double>(3, 3)
                <<  parameters_[0], 0.0, parameters_[2],
                    0.0, parameters_[1], parameters_[3],
                    0.0, 0.0, 1.0);
    return K;
}

Eigen::Matrix3f PinholeModel::eigenKfloat() const {
    Eigen::Matrix3f K;
    K   <<  static_cast<float>(parameters_[0]), 0.f, static_cast<float>(parameters_[2]),
            0.f, static_cast<float>(parameters_[1]), static_cast<float>(parameters_[3]),
            0.f, 0.f, 1.f;
    return K;
}

Eigen::Matrix3d PinholeModel::eigenKdouble() const {
    Eigen::Matrix3d K;
    K   <<  parameters_[0], 0.0, parameters_[2],
            0.0, parameters_[1], parameters_[3],
            0.0, 0.0, 1.0;
    return K;
}

cv::Mat PinholeModel::cvDfloat() const {
    if (parameters_.size() > 5) {
        cv::Mat D = (cv::Mat_<float>(1, 5)
                    << parameters_[5], parameters_[6], parameters_[7], parameters_[8], parameters_[9]);
        return D;
    } else {
        return cv::Mat::zeros(1, 5, CV_32FC1);
    }
}

cv::Mat PinholeModel::cvDdouble() const {
    if (parameters_.size() > 5) {
        cv::Mat D = (cv::Mat_<double>(1, 5)
                    << parameters_[5], parameters_[6], parameters_[7], parameters_[8], parameters_[9]);
        return D;
    } else {
        return cv::Mat::zeros(1, 5, CV_64FC1);
    }
}

float PinholeModel::getBaseLine() const {
    return static_cast<float>(parameters_[4]);
}
    

}   // namespace