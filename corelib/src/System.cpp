#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include "System.h"

namespace VISFS {

System::System(const ParametersMap & _parameters) {
    estimator_ = new Estimator(_parameters);
    tracker_ = new Tracker(estimator_, _parameters);
    threadTracker_ = new boost::thread(boost::bind(&Tracker::threadProcess, tracker_));
    threadEstimator_ = new boost::thread(boost::bind(&Estimator::threadProcess, estimator_));
}

System::~System() {
    threadTracker_->interrupt();
    threadEstimator_->interrupt();
    threadTracker_->join();
    threadEstimator_->join();
    delete tracker_;
    delete estimator_;
}

void System::init(const boost::shared_ptr<GeometricCamera> & _cameraLeft, const boost::shared_ptr<GeometricCamera> & _cameraRight) {
    cameraLeft_ = _cameraLeft;
    cameraRight_ = _cameraRight;
}

void System::init(const double fxl, const double fyl, const double cxl, const double cyl, const double fxr, const double fyr,
            const double cxr, const double cyr, const double baseline) {
    Eigen::Matrix3d Kl, Kr;
    Kl << fxl, 0.0, cxl, 0.0, fyl, cyl, 0, 0, 1;
    Kr << fxr, 0.0, cxr, 0.0, fyr, cyr, 0, 0, 1;
    GeometricCamera * cameraLeft = new PinholeModel(Kl, baseline);
    boost::shared_ptr<GeometricCamera> spcameraLeft(cameraLeft);
    GeometricCamera * cameraRight = new PinholeModel(Kr, baseline);
    boost::shared_ptr<GeometricCamera> spcameraRight(cameraRight);
    cameraLeft_ = spcameraLeft;
    cameraRight_ = cameraRight_;
}

Eigen::Isometry3d System::getGuessPose(const Eigen::Isometry3d & _guessVelocity, const double _dt) {
    if (!_guessVelocity.isApprox(Eigen::Isometry3d::Identity()) && ! _dt > 0) {
        return Eigen::Isometry3d::Identity();
    } else {
        double vx, vy, vz, vroll, vpitch, vyaw;
        pcl::getTranslationAndEulerAngles(_guessVelocity, vx, vy, vz, vroll, vpitch, vyaw);
        vx *= _dt;
        vy *= _dt;
        vz *= _dt;
        vroll *= _dt;
        vpitch *= _dt;
        vyaw *= _dt;
        Eigen::Affine3d pose;
        pcl::getTransformation(vx, vy, vz, vroll, vpitch, vyaw, pose);
        return Eigen::Isometry3d(pose.matrix());
    }
}

void System::inputStereoImage(const double time_, const cv::Mat & imageLeft_, const cv::Mat & imageRight_) {
    // Construct signature.
    Signature signature(time_, imageLeft_, imageRight_, cameraLeft_, cameraRight_);
    Eigen::Isometry3d guessPose = getGuessPose(velocityGuess_, time_ - previousTimeStamp_);
    tracker_->inputSignature(signature, guessPose);
    previousTimeStamp_ = time_;
}

bool System::outputOdometryInfo(Eigen::Isometry3d & _pose, TrackInfo & _trackInfo, EstimateInfo & _estimateInfo) {
    Signature signature;
    signature = estimator_->getEstimatedSignature();
    if (!signature.empty()) {
        _pose = signature.getPose();
        _trackInfo = signature.getTrackInfo();
        _estimateInfo = signature.getEstimateInfo();
        return true;
    }
    return false;

}

}   // namespace