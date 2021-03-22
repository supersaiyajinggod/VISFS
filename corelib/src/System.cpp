#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include "System.h"
#include "Conversion.h"
#include "Timer.h"

namespace VISFS {

System::System(const ParametersMap & _parameters) :
    tracker_(nullptr),
    threadTracker_(nullptr),
    estimator_(nullptr),
    threadEstimator_(nullptr),
    monitor_(nullptr),
    threadMonitor_(nullptr),
    logger_(nullptr),
    logLevel_(Parameters::defaultSystemLogLevel()),
    console_(Parameters::defaultSystemLogOnConsole()),
    logFolder_(Parameters::defaultSystemLogFolder()),
    cameraLeft_(nullptr),
    cameraRight_(nullptr),
    monitorSwitch_(Parameters::defaultSystemMonitor()),
    sensorStrategy_(Parameters::defaultSystemSensorStrategy()),
	claheSwitch_(Parameters::defaultSystemCLAHE()) {
    
    Parameters::parse(_parameters, Parameters::kSystemMonitor(), monitorSwitch_);
    Parameters::parse(_parameters, Parameters::kSystemSensorStrategy(), sensorStrategy_);
    Parameters::parse(_parameters, Parameters::kSystemLogLevel(), logLevel_);
    Parameters::parse(_parameters, Parameters::kSystemLogOnConsole(), console_);
    Parameters::parse(_parameters, Parameters::kSystemLogFolder(), logFolder_);
	Parameters::parse(_parameters, Parameters::kSystemCLAHE(), claheSwitch_);

    logger_ = new Logger(static_cast<SeverityLevel>(logLevel_), console_, logFolder_);
    // Logger lg(static_cast<SeverityLevel>(logLevel_), console_, logFolder_);

    extrapolator_ = new Extrapolator(_parameters);

    estimator_ = new Estimator(_parameters);
    tracker_ = new Tracker(_parameters);

    estimator_->setTracker(tracker_);
    tracker_->setEstimator(estimator_);

    if (monitorSwitch_) {
        monitor_ = new Monitor(_parameters);
        estimator_->setMonitor(monitor_);
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
    delete extrapolator_;
    delete logger_;
}

void System::init(const boost::shared_ptr<GeometricCamera> & _cameraLeft, const boost::shared_ptr<GeometricCamera> & _cameraRight,
                const Eigen::Isometry3d & _transformCamera2Robot, const Eigen::Isometry3d & _transformLaser2Robot) {
    cameraLeft_ = _cameraLeft;
    cameraRight_ = _cameraRight;
    transformCamera2Robot_ = _transformCamera2Robot;
    transformLaser2Robot_ = _transformLaser2Robot;

    LOG_INFO << "System initialization over!";
}

void System::init(const double fxl, const double fyl, const double cxl, const double cyl, const double fxr, const double fyr,
            const double cxr, const double cyr, const double baseline,
            const Eigen::Isometry3d & _transformCamera2Robot, const Eigen::Isometry3d & _transformLaser2Robot) {
    Eigen::Matrix3d Kl, Kr;
    Kl << fxl, 0.0, cxl, 0.0, fyl, cyl, 0, 0, 1;
    Kr << fxr, 0.0, cxr, 0.0, fyr, cyr, 0, 0, 1;
    GeometricCamera * cameraLeft = new PinholeModel(Kl, baseline);
    boost::shared_ptr<GeometricCamera> spcameraLeft(cameraLeft);
    GeometricCamera * cameraRight = new PinholeModel(Kr, baseline);
    boost::shared_ptr<GeometricCamera> spcameraRight(cameraRight);
    cameraLeft_ = spcameraLeft;
    cameraRight_ = spcameraRight;
    transformCamera2Robot_ = _transformCamera2Robot;
    transformLaser2Robot_ = _transformLaser2Robot;

	LOG_INFO << "System initialization over!";
}

void System::inputPrimarySensorData(const double _time, const cv::Mat & _imageLeft, const cv::Mat & _imageRight, const Sensor::TimedPointCloudWithIntensities & _timedPointCloud) {
    Eigen::Isometry3d guessPose;
    Eigen::Isometry3d globalWheelPose(Eigen::Isometry3d(Eigen::Matrix4d::Zero()));
    Signature signature;

    // UTimer timer;
	if (claheSwitch_) {
		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
		clahe->apply(_imageLeft, _imageLeft);
		clahe->apply(_imageRight, _imageRight);
	}
	// timer.elapsed("CLAHE");

    extrapolator_->extrapolatorPose(_time, globalWheelPose, guessPose);

    if (sensorStrategy_ == 0 || sensorStrategy_ == 1) {     // stereo or rgbd
        signature = Signature(_time, _imageLeft, _imageRight, cameraLeft_, cameraRight_, transformCamera2Robot_, transformLaser2Robot_, guessPose, Eigen::Isometry3d(Eigen::Matrix4d::Zero()), _timedPointCloud);
    } else if (sensorStrategy_ == 2) {      // stereo + wheel
        signature = Signature(_time, _imageLeft, _imageRight, cameraLeft_, cameraRight_, transformCamera2Robot_, transformLaser2Robot_, guessPose, globalWheelPose, _timedPointCloud);
    } else if (sensorStrategy_ == 3) {      // stereo + laser + wheel
        signature = Signature(_time, _imageLeft, _imageRight, cameraLeft_, cameraRight_, transformCamera2Robot_, transformLaser2Robot_, guessPose, globalWheelPose, _timedPointCloud);
    }

    tracker_->inputSignature(signature);
}

void System::inputWheelOdometry(const double _time, const Eigen::Isometry3d & _pose, const Eigen::Isometry3d & _velocity) {
    if (sensorStrategy_ == 2 || sensorStrategy_ == 3) {
        extrapolator_->addOdometry(_time, _pose, _velocity);
    } else {
        LOG_WARN << "System no need for wheel Odometry, please check prameters.";
    }
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