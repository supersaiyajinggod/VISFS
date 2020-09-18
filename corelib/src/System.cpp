#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include "System.h"

namespace VISFS {

System::System(const ParametersMap & _parameters) :
    tracker_(nullptr),
    threadTracker_(nullptr),
    estimator_(nullptr),
    threadEstimator_(nullptr),
    monitor_(nullptr),
    threadMonitor_(nullptr),
    cameraLeft_(nullptr),
    cameraRight_(nullptr),
    velocityGuess_(Eigen::Isometry3d(Eigen::Matrix4d::Zero())),
    previousTimeStamp_(0.0),
    monitorSwitch_(Parameters::defaultSystemMonitor()) {
    
    Parameters::parse(_parameters, Parameters::kSystemMonitor(), monitorSwitch_);

    estimator_ = new Estimator(_parameters);
    tracker_ = new Tracker(estimator_, _parameters);

    if (monitorSwitch_) {
        monitor_ = new Monitor(_parameters);
        tracker_->setMonitor(monitor_);
        threadMonitor_ = new boost::thread(boost::bind(&Monitor::threadProcess, monitor_));
    }

    threadTracker_ = new boost::thread(boost::bind(&Tracker::threadProcess, tracker_));
    threadEstimator_ = new boost::thread(boost::bind(&Estimator::threadProcess, estimator_));
}

System::~System() {
    if (monitorSwitch_) {
        threadMonitor_->interrupt();
        threadMonitor_->join();
        delete monitor_;
        monitor_ = nullptr;
    }

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
    cameraRight_ = spcameraRight;
}

Eigen::Isometry3d System::getGuessPose(const Eigen::Isometry3d & _guessVelocity, const double _dt) {
    if (!_guessVelocity.isApprox(Eigen::Isometry3d::Identity()) && ! (_dt > 0.0)) {
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
    Eigen::Isometry3d guessPose;
    if (!velocityGuess_.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero())) && !(previousTimeStamp_ == 0.0)) {
        guessPose = getGuessPose(velocityGuess_, time_ - previousTimeStamp_);
    } else {
        guessPose = Eigen::Isometry3d::Identity();
    }
    tracker_->inputSignature(signature, guessPose);
    previousTimeStamp_ = time_;
}

bool System::outputOdometryInfo(double & _stamp, Eigen::Isometry3d & _pose, TrackInfo & _trackInfo, EstimateInfo & _estimateInfo) {
    Signature signature;
    signature = estimator_->getEstimatedSignature();
    if (!signature.empty()) {
        _stamp = signature.getTimeStamp();
        _pose = signature.getPose();
        _trackInfo = signature.getTrackInfo();
        _estimateInfo = signature.getEstimateInfo();
        return true;
    }
    return false;

}

}   // namespace