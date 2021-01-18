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
    velocityGuess_(Eigen::Isometry3d(Eigen::Matrix4d::Zero())),
    previousWheelOdom_(Eigen::Isometry3d(Eigen::Matrix4d::Zero())),
    previousTimeStamp_(0.0),
    monitorSwitch_(Parameters::defaultSystemMonitor()),
    sensorStrategy_(Parameters::defaultSystemSensorStrategy()),
    wheelFreq_(Parameters::defaultSystemWheelOdometryFreq()),
	claheSwitch_(Parameters::defaultSystemCLAHE()) {
    
    Parameters::parse(_parameters, Parameters::kSystemMonitor(), monitorSwitch_);
    Parameters::parse(_parameters, Parameters::kSystemSensorStrategy(), sensorStrategy_);
    Parameters::parse(_parameters, Parameters::kSystemWheelOdometryFreq(), wheelFreq_);
    Parameters::parse(_parameters, Parameters::kSystemLogLevel(), logLevel_);
    Parameters::parse(_parameters, Parameters::kSystemLogOnConsole(), console_);
    Parameters::parse(_parameters, Parameters::kSystemLogFolder(), logFolder_);
	Parameters::parse(_parameters, Parameters::kSystemCLAHE(), claheSwitch_);

    logger_ = new Logger(static_cast<SeverityLevel>(logLevel_), console_, logFolder_);
    // Logger lg(static_cast<SeverityLevel>(logLevel_), console_, logFolder_);

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
}

void System::init(const boost::shared_ptr<GeometricCamera> & _cameraLeft, const boost::shared_ptr<GeometricCamera> & _cameraRight) {
    cameraLeft_ = _cameraLeft;
    cameraRight_ = _cameraRight;

    LOG_INFO << "System initialization over!";
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

	LOG_INFO << "System initialization over!";
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

Eigen::Isometry3d System::accMotionModel(const double _deltaTime, const bool _dirction, const Eigen::Isometry3d & _pose, const Eigen::Isometry3d & _v1, const Eigen::Isometry3d & _v2) const {
    // direction : true  secondlast -> last,  false : last -> secondlast
    double vx1, vy1, vz1, vroll1, vpitch1, vyaw1, vx2, vy2, vz2, vroll2, vpitch2, vyaw2, ax, ay, az, aroll, apitch, ayaw;
    pcl::getTranslationAndEulerAngles(_v1, vx1, vy1, vz1, vroll1, vpitch1, vyaw1);
    pcl::getTranslationAndEulerAngles(_v2, vx2, vy2, vz2, vroll2, vpitch2, vyaw2);
    ax = vx2 - vx1;
    ay = vy2 - vy1;
    az = vz2 - vz1;
    aroll = vroll2 - vroll1;
    apitch = vpitch2 - vpitch1;
    ayaw = vyaw2 - vyaw1;
    const double halfDeltaT = 0.5 * _deltaTime;
    double x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(_pose, x, y, z, roll, pitch, yaw);
    if (_dirction) {
        x = x + vx1 * _deltaTime + ax * halfDeltaT;
        y = y + vy1 * _deltaTime + ay * halfDeltaT;
        z = z + vz1 * _deltaTime + az * halfDeltaT;
        roll = roll + vroll1 * _deltaTime + aroll * halfDeltaT;
        pitch = pitch + vpitch1 * _deltaTime + apitch * halfDeltaT;
        yaw = yaw + vyaw1 * _deltaTime + ayaw * halfDeltaT;
    } else {
        ax *= -1.0;
        ay *= -1.0;
        az *= -1.0;
        aroll *= -1.0;
        apitch *= -1.0;
        ayaw *= -1.0;
        vx2 *= -1.0;
        vy2 *= -1.0;
        vz2 *= -1.0;
        vroll2 *= -1.0;
        vpitch2 *= -1.0;
        vyaw2 *= -1.0;

        x = x + vx2 * _deltaTime + ax * halfDeltaT;
        y = y + vy2 * _deltaTime + ay * halfDeltaT;
        z = z + vz2 * _deltaTime + az * halfDeltaT;
        roll = roll + vroll2 * _deltaTime + aroll * halfDeltaT;
        pitch = pitch + vpitch2 * _deltaTime + apitch * halfDeltaT;
        yaw = yaw + vyaw2 * _deltaTime + ayaw * halfDeltaT;
    }

    Eigen::Affine3d pose;
    pcl::getTransformation(x, y, z, roll, pitch, yaw, pose);
    return Eigen::Isometry3d(pose.matrix());
}

Eigen::Isometry3d System::velMotionModel(const double _deltaTime, const Eigen::Isometry3d & _basePose, const double & _time1, const double & _time2, Eigen::Isometry3d & _pose1, const Eigen::Isometry3d & _pose2) const {
    if (_deltaTime <= 0.0) {
        LOG_ERROR << "System::velMotionModel: _deltaTime <= 0.0.";
        return Eigen::Isometry3d(Eigen::Matrix4d::Zero());
    }

    double x1, y1, z1, roll1, pitch1, yaw1, x2, y2, z2, roll2, pitch2, yaw2, bx, by, bz, broll, bpitch, byaw;
    pcl::getTranslationAndEulerAngles(_pose1, x1, y1, z1, roll1, pitch1, yaw1);
    pcl::getTranslationAndEulerAngles(_pose2, x2, y2, z2, roll2, pitch2, yaw2);
    pcl::getTranslationAndEulerAngles(_basePose, bx, by, bz, broll, bpitch, byaw);
    const double interval = _time2 - _time1;

    bx = bx + (x2 - x1) * _deltaTime / interval;
    by = by + (y2 - y1) * _deltaTime / interval;
    bz = bz + (z2 - z1) * _deltaTime / interval;
    broll = broll + (roll2 - roll1) * _deltaTime / interval;
    bpitch = bpitch + (pitch2 - pitch1) * _deltaTime / interval;
    byaw = byaw + (yaw2 - yaw1) * _deltaTime / interval;

    Eigen::Affine3d pose;
    pcl::getTransformation(bx, by, bz, broll, bpitch, byaw, pose);
    return Eigen::Isometry3d(pose.matrix());
}

Eigen::Isometry3d System::predictAlignPose(const double _time, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _lastOdom, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _secondLastOdom) const {
    Eigen::Isometry3d alignPose(Eigen::Isometry3d::Identity());
    auto [lastTime, lastPose, lastVelocity] = _lastOdom;
    auto [secondLastTime, secondLastPose, secondLastVelocity] = _secondLastOdom;
    const int timeInterval = 1000 / wheelFreq_;   // millisecond
    if (secondLastTime <= _time &&  _time <= lastTime) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(lastTime) - uTimeDouble2Boost(secondLastTime);
        if (timeElapse.total_milliseconds() > 2*timeInterval) {
            LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime << ", time second last wheel: " << secondLastTime << ", the image:" << _time;         
        } else {
            const double deltaTime = _time - secondLastTime;
            alignPose = velMotionModel(deltaTime, secondLastPose, secondLastTime, lastTime, secondLastPose, lastPose);
        }          
    } else if (lastTime < _time) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(_time) - uTimeDouble2Boost(lastTime);
        if (timeElapse.total_milliseconds() > timeInterval) {
            LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime << ", time second last wheel: " << secondLastTime << ", the image:" << _time;      
        } else {
            const double deltaTime = _time - lastTime;
            // alignPose = accMotionModel(deltaTime, true, lastPose, secondLastVelocity, lastVelocity);
            alignPose = velMotionModel(deltaTime, secondLastPose, secondLastTime, lastTime, secondLastPose, lastPose);
        }
    } else {
        LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime << ", time second last wheel: " << secondLastTime << ", the image:" << _time;     
    }

    return alignPose;
}

Eigen::Isometry3d System::predictAlignPose(const double _time, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _lastOdom, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _secondLastOdom,
                                                        const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _thridLastOdom) const {
    Eigen::Isometry3d alignPose(Eigen::Isometry3d::Identity());
    auto [lastTime, lastPose, lastVelocity] = _lastOdom;
    auto [secondLastTime, secondLastPose, secondLastVelocity] = _secondLastOdom;
    auto [thirdLastTime, thirdLastPose, thirdLastVelocity] = _thridLastOdom;
    const int timeInterval = 1000 / wheelFreq_;   // millisecond
    if (secondLastTime <= _time &&  _time <= lastTime) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(lastTime) - uTimeDouble2Boost(secondLastTime);
        if (timeElapse.total_milliseconds() > 2*timeInterval) {
            LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime << ", time second last wheel: " << secondLastTime << ", the image:" << _time;         
        } else {
            const double deltaTime = _time - secondLastTime;
            alignPose = velMotionModel(deltaTime, secondLastPose, secondLastTime, lastTime, secondLastPose, lastPose);
        }          
    } else if (lastTime < _time) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(_time) - uTimeDouble2Boost(lastTime);
        if (timeElapse.total_milliseconds() > timeInterval) {
            LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime << ", time second last wheel: " << secondLastTime << ", the image:" << _time;      
        } else {
            const double deltaTime = _time - lastTime;
            // alignPose = accMotionModel(deltaTime, true, lastPose, secondLastVelocity, lastVelocity);
            alignPose = velMotionModel(deltaTime, lastPose, secondLastTime, lastTime, secondLastPose, lastPose);
        }
    } else if (thirdLastTime <= _time && _time < secondLastTime) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(secondLastTime) - uTimeDouble2Boost(thirdLastTime);
        if (timeElapse.total_milliseconds() > 2*timeInterval) {
            LOG_ERROR << "Time stamps error, Time second last wheel: " << secondLastTime << ", time third last wheel: " << thirdLastTime << ", the image:" << _time;         
        } else {
            const double deltaTime = _time - thirdLastTime;
            alignPose = velMotionModel(deltaTime, thirdLastPose, thirdLastTime, secondLastTime, thirdLastPose, secondLastPose);
        }
    } else if (_time < thirdLastTime) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(thirdLastTime) - uTimeDouble2Boost(_time);
        if (timeElapse.total_milliseconds() > 2*timeInterval) {
            LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime << ", time second last wheel: " << secondLastTime << ", time third last wheel: " << thirdLastTime << ", the image:" << _time; 
        } else {
            const double deltaTime = thirdLastTime - _time;
            alignPose = velMotionModel(deltaTime, thirdLastPose, thirdLastTime, secondLastTime, secondLastPose, thirdLastPose);
        }
    } else {
    	LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime << ", time second last wheel: " << secondLastTime << ", time third last wheel: " << thirdLastTime << ", the image:" << _time;     
    }

    return alignPose;
}


Eigen::Isometry3d System::predictAlignPose(const int _n, const double _time, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _lastOdom, ...) {
    assert(_n >= 1);
    Eigen::Isometry3d alignPose(Eigen::Matrix4d::Zero());
    
    std::va_list vl;
	va_start(vl, _n);

    if (_n == 1) {
        auto [lastTime, lastPose, lastVelocity] = _lastOdom;
        if (std::abs(lastTime - _time) > 2.0/static_cast<double>(wheelFreq_)) {
            alignPose = lastPose;
        } else {
            LOG_WARN << "Time gap is too large when only use single measure of wheel odometry.";
        }
    }

    std::vector<std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d>> wheelOdomBuf;
    for (auto i = 0; i < _n - 1; ++i) {
        typedef std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> odomType;
        wheelOdomBuf.emplace_back(va_arg(vl, odomType));
    }
    va_end(vl);


    if (_n == 2) {
        auto _secondLastOdom = *wheelOdomBuf.rbegin();
        auto [lastTime, lastPose, lastVelocity] = _lastOdom;
        auto [secondLastTime, secondLastPose, secondLastVelocity] = _secondLastOdom;
        const int timeInterval = 1000 / wheelFreq_;   // millisecond
        if (secondLastTime <= _time &&  _time <= lastTime) {
            boost::posix_time::time_duration timeElapse;
            timeElapse = uTimeDouble2Boost(lastTime) - uTimeDouble2Boost(secondLastTime);
            if (timeElapse.total_milliseconds() > 2*timeInterval) {
                LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime << ", time second last wheel: " << secondLastTime << ", the image:" << _time;         
            } else {
                timeElapse = uTimeDouble2Boost(lastTime) - uTimeDouble2Boost(_time);
                bool direction = timeElapse.total_milliseconds() > wheelFreq_/2 ? true : false; //true: second->last, false: last->second
                // if (direction) {
                //     const double deltaTime = _time - secondLastTime;
                //     alignPose = accMotionModel(deltaTime, direction, secondLastPose, secondLastVelocity, lastVelocity);
                // } else {
                //     const double deltaTime = lastTime - _time;
                //     alignPose = accMotionModel(deltaTime, direction, lastPose, secondLastVelocity, lastVelocity);
                // }
                const double deltaTime = _time - secondLastTime;
                alignPose = velMotionModel(deltaTime, secondLastPose, secondLastTime, lastTime, secondLastPose, lastPose);
            }          
        } else if (lastTime < _time) {
            boost::posix_time::time_duration timeElapse;
            timeElapse = uTimeDouble2Boost(_time) - uTimeDouble2Boost(lastTime);
            if (timeElapse.total_milliseconds() > timeInterval) {
                LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime << ", time second last wheel: " << secondLastTime << ", the image:" << _time;      
            } else {
                const double deltaTime = _time - lastTime;
                // alignPose = accMotionModel(deltaTime, true, lastPose, secondLastVelocity, lastVelocity);
                alignPose = velMotionModel(deltaTime, secondLastPose, secondLastTime, lastTime, secondLastPose, lastPose);
            }
        } else if (_time < secondLastTime) {

        } else {
            LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime << ", time second last wheel: " << secondLastTime << ", the image:" << _time;     
        }   

    }

    return alignPose;
}

void System::inputStereoImage(const double _time, const cv::Mat & _imageLeft, const cv::Mat & _imageRight) {
    Eigen::Isometry3d guessPose;
    Eigen::Isometry3d globalWheelPose;
    Eigen::Isometry3d deltaOdomPose;
    Signature signature;

    UTimer timer;
	if (claheSwitch_) {
		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
		clahe->apply(_imageLeft, _imageLeft);
		clahe->apply(_imageRight, _imageRight);
	}
	timer.elapsed("CLAHE");

    if (sensorStrategy_ == 0 || sensorStrategy_ == 1) {     // stereo or rgbd
        if (!velocityGuess_.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero())) && !(previousTimeStamp_ == 0.0)) {
            guessPose = getGuessPose(velocityGuess_, _time - previousTimeStamp_);
        } else {
            guessPose = Eigen::Isometry3d::Identity();
        }
        signature = Signature(_time, _imageLeft, _imageRight, cameraLeft_, cameraRight_, guessPose);
    } else if (sensorStrategy_ == 2) {      // stereo + wheel
        if (!wheelOdometryBuf_.empty() && wheelOdometryBuf_.size() > 2) {
            // get wheel odometry
            std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> lastOdom, secondLastOdom, thirdLastOdom;
            const int odomCount = wheelOdometryBuf_.size();
            {   
                boost::lock_guard<boost::mutex> lock(mutexWheelOdometryBuf_);
                auto rit = wheelOdometryBuf_.rbegin();
                lastOdom = * rit;
                secondLastOdom = * ++rit;
                if (odomCount >= 3)
                    thirdLastOdom = * ++rit;
                wheelOdometryBuf_.clear();
            }
            // process wheel odometry, calculate guess pose.
            if (odomCount == 2) {
                globalWheelPose = predictAlignPose(_time, lastOdom, secondLastOdom);
                // globalWheelPose = predictAlignPose(2, _time, lastOdom, secondLastOdom);
            } else if (odomCount >= 3) {
                globalWheelPose = predictAlignPose(_time, lastOdom, secondLastOdom, thirdLastOdom);
            }
            
            if (!globalWheelPose.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
                auto lastTime = std::get<0>(lastOdom);
                auto secondLastTime = std::get<0>(secondLastOdom);

                if (previousWheelOdom_.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
                    guessPose = Eigen::Isometry3d::Identity();
                    deltaOdomPose = Eigen::Isometry3d(Eigen::Matrix4d::Zero());
                } else {
                    deltaOdomPose = previousWheelOdom_.inverse() * globalWheelPose;
                    guessPose = deltaOdomPose;
                }
            } else {
                guessPose = Eigen::Isometry3d::Identity();
                deltaOdomPose = Eigen::Isometry3d(Eigen::Matrix4d::Zero());
            }
            previousWheelOdom_ = globalWheelPose;

            signature = Signature(_time, _imageLeft, _imageRight, cameraLeft_, cameraRight_, guessPose, globalWheelPose);
        } else {
            // Error no odom, use pure stereo.
            LOG_ERROR << "Error: Strategy is stereo + wheel, but there is no wheel odom recieved.";  
            if (!velocityGuess_.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero())) && !(previousTimeStamp_ == 0.0)) {
                guessPose = getGuessPose(velocityGuess_, _time - previousTimeStamp_);
            } else {
                guessPose = Eigen::Isometry3d::Identity();
            }
            signature = Signature(_time, _imageLeft, _imageRight, cameraLeft_, cameraRight_, guessPose);
        }
    }
    previousTimeStamp_ = _time;
    tracker_->inputSignature(signature);
}

void System::inputWheelOdometry(const double _time, const Eigen::Isometry3d & _pose, const Eigen::Isometry3d & _velocity) {
    if (sensorStrategy_ == 2) {
        boost::lock_guard<boost::mutex> lock(mutexWheelOdometryBuf_);
        wheelOdometryBuf_.emplace_back(_time, _pose, _velocity);
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