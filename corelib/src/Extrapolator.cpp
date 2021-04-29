#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include "Extrapolator.h"
#include "Conversion.h"
#include "Timer.h"
#include "Log.h"

namespace VISFS {

Extrapolator::Extrapolator(const ParametersMap & _parameters) :
    velocityGuess_(Eigen::Isometry3d(Eigen::Matrix4d::Zero())),
    previousWheelOdom_(Eigen::Isometry3d(Eigen::Matrix4d::Zero())),
    previousTimeStamp_(0.0),
    sensorStrategy_(Parameters::defaultSystemSensorStrategy()),
    wheelFreq_(Parameters::defaultSystemWheelOdometryFreq()) {

    Parameters::parse(_parameters, Parameters::kSystemSensorStrategy(), sensorStrategy_);
    Parameters::parse(_parameters, Parameters::kSystemWheelOdometryFreq(), wheelFreq_);
}

Extrapolator::~Extrapolator() {}

void Extrapolator::addOdometry(const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _odom) {
    std::lock_guard<std::mutex> lock(mutexWheelOdometryBuf_);
    wheelOdometryBuf_.emplace_back(_odom);
}

void Extrapolator::addOdometry(const double _time, const Eigen::Isometry3d & _pose, const Eigen::Isometry3d & _velocity) {
    std::lock_guard<std::mutex> lock(mutexWheelOdometryBuf_);
    wheelOdometryBuf_.emplace_back(std::make_tuple(_time, _pose, _velocity));
}

std::vector<std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d>> Extrapolator::getApproximateOdometry(const double _time) {
    std::vector<std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d>> vOdom;
    std::lock_guard<std::mutex> lock(mutexWheelOdometryBuf_);

    double bestScore = 1.0;
    std::list<std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d>>::iterator bestCandidate = wheelOdometryBuf_.begin();
    std::list<std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d>>::iterator secondBestCandidate = wheelOdometryBuf_.begin();
    for (auto it = wheelOdometryBuf_.begin(); it != wheelOdometryBuf_.end(); ++it) {
        auto [time, pose, velocity] = *it;
        double score = std::abs(_time - time);
        if (std::abs(_time - time) < bestScore) {
            bestScore = score;
            secondBestCandidate = bestCandidate;
            bestCandidate = it;
        }
    }

    if (std::get<0>(*bestCandidate) > std::get<0>(*secondBestCandidate)) {
        vOdom.emplace_back(*bestCandidate);
        vOdom.emplace_back(*secondBestCandidate);
    } else {
        vOdom.emplace_back(*secondBestCandidate);
        vOdom.emplace_back(*bestCandidate);
    }

    while (wheelOdometryBuf_.size() > wheelFreq_ / 10) {
        wheelOdometryBuf_.pop_front();
    }
    

    return vOdom;

}

void Extrapolator::extrapolatorPose(const double _time, Eigen::Isometry3d & _globalPose, Eigen::Isometry3d & _deltaPose) {
    if (sensorStrategy_ == 0 || sensorStrategy_ == 1) {     // stereo | rgbd
        if (!velocityGuess_.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero())) && !(previousTimeStamp_ == 0.0)) {
            _deltaPose = extrapolateFromVelocity(velocityGuess_, _time - previousTimeStamp_);
        } else {
            _deltaPose = Eigen::Isometry3d::Identity();
        }
    } else if (sensorStrategy_ >= 2) {  // stereo + wheel
        if (hasAvaliableOdometry()) {
            // get wheel odometry
            std::vector<std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d>> vOdom;
            vOdom = getApproximateOdometry(_time);
            // process wheel odometry, calculate guess pose.
            _globalPose = predictAlignPose(_time, vOdom[0], vOdom[1]);

            if (!_globalPose.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
                if (previousWheelOdom_.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
                    _deltaPose = Eigen::Isometry3d::Identity();
                } else {
                    _deltaPose = previousWheelOdom_.inverse() * _globalPose;
                }
            } else {
                _deltaPose = Eigen::Isometry3d::Identity();
            }
            previousWheelOdom_ = _globalPose;
        } else {
            // Error no odom, use pure stereo.
            LOG_ERROR << "Error: Strategy is stereo + wheel, but there is no wheel odom recieved.";
            if (!velocityGuess_.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero())) && !(previousTimeStamp_ == 0.0)) {
                _deltaPose = extrapolateFromVelocity(velocityGuess_, _time - previousTimeStamp_);
            } else {
                _deltaPose = Eigen::Isometry3d::Identity();
            }
        }
    }
    previousTimeStamp_ = _time;
}

Eigen::Isometry3d Extrapolator::extrapolateFromVelocity(const Eigen::Isometry3d & _guessVelocity, const double _dt) {
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

Eigen::Isometry3d Extrapolator::accMotionModel(const double _deltaTime, const bool _dirction, const Eigen::Isometry3d & _pose, const Eigen::Isometry3d & _v1, const Eigen::Isometry3d & _v2) const {
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

Eigen::Isometry3d Extrapolator::velMotionModel(const double _deltaTime, const Eigen::Isometry3d & _basePose, const double & _time1, const double & _time2, Eigen::Isometry3d & _pose1, const Eigen::Isometry3d & _pose2) const {
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

Eigen::Isometry3d Extrapolator::predictAlignPose(const double _time, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _lastOdom, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _secondLastOdom) const {
    Eigen::Isometry3d alignPose(Eigen::Isometry3d(Eigen::Matrix4d::Zero()));
    auto [lastTime, lastPose, lastVelocity] = _lastOdom;
    auto [secondLastTime, secondLastPose, secondLastVelocity] = _secondLastOdom;
    const int timeInterval = 1000 / wheelFreq_;   // millisecond
    if (secondLastTime <= _time &&  _time <= lastTime) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(lastTime) - uTimeDouble2Boost(secondLastTime);
        if (timeElapse.total_milliseconds() > 2*timeInterval) {
            LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime - floor(lastTime) << ", time second last wheel: " << secondLastTime - floor(secondLastTime) << ", the image:" << _time - floor(_time);         
        } else {
            const double deltaTime = _time - secondLastTime;
            alignPose = velMotionModel(deltaTime, secondLastPose, secondLastTime, lastTime, secondLastPose, lastPose);
        }          
    } else if (lastTime < _time) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(_time) - uTimeDouble2Boost(lastTime);
        if (timeElapse.total_milliseconds() > timeInterval) {
            LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime - floor(lastTime) << ", time second last wheel: " << secondLastTime - floor(secondLastTime) << ", the image:" << _time - floor(_time);     
        } else {
            const double deltaTime = _time - lastTime;
            // alignPose = accMotionModel(deltaTime, true, lastPose, secondLastVelocity, lastVelocity);
            alignPose = velMotionModel(deltaTime, secondLastPose, secondLastTime, lastTime, secondLastPose, lastPose);
        }
    } else {
        LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime - floor(lastTime) << ", time second last wheel: " << secondLastTime - floor(secondLastTime) << ", the image:" << _time - floor(_time);    
    }

    return alignPose;
}

Eigen::Isometry3d Extrapolator::predictAlignPose(const double _time, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _lastOdom, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _secondLastOdom,
                                                        const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _thridLastOdom) const {
    Eigen::Isometry3d alignPose(Eigen::Isometry3d(Eigen::Matrix4d::Zero()));
    auto [lastTime, lastPose, lastVelocity] = _lastOdom;
    auto [secondLastTime, secondLastPose, secondLastVelocity] = _secondLastOdom;
    auto [thirdLastTime, thirdLastPose, thirdLastVelocity] = _thridLastOdom;
    const int timeInterval = 1000 / wheelFreq_;   // millisecond
    if (secondLastTime <= _time &&  _time <= lastTime) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(lastTime) - uTimeDouble2Boost(secondLastTime);
        if (timeElapse.total_milliseconds() > 2*timeInterval) {
            LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime - floor(lastTime) << ", time second last wheel: " << secondLastTime - floor(secondLastTime) << ", the image:" << _time - floor(_time);        
        } else {
            const double deltaTime = _time - secondLastTime;
            alignPose = velMotionModel(deltaTime, secondLastPose, secondLastTime, lastTime, secondLastPose, lastPose);
        }          
    } else if (lastTime < _time) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(_time) - uTimeDouble2Boost(lastTime);
        if (timeElapse.total_milliseconds() > timeInterval) {
            LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime - floor(lastTime) << ", time second last wheel: " << secondLastTime - floor(secondLastTime) << ", the image:" << _time - floor(_time);     
        } else {
            const double deltaTime = _time - lastTime;
            // alignPose = accMotionModel(deltaTime, true, lastPose, secondLastVelocity, lastVelocity);
            alignPose = velMotionModel(deltaTime, lastPose, secondLastTime, lastTime, secondLastPose, lastPose);
        }
    } else if (thirdLastTime <= _time && _time < secondLastTime) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(secondLastTime) - uTimeDouble2Boost(thirdLastTime);
        if (timeElapse.total_milliseconds() > 2*timeInterval) {
            LOG_ERROR << "Time stamps error, Time second last wheel: " << secondLastTime - floor(secondLastTime) << ", time third last wheel: " << thirdLastTime - floor(thirdLastTime) << ", the image:" << _time - floor(_time);       
        } else {
            const double deltaTime = _time - thirdLastTime;
            alignPose = velMotionModel(deltaTime, thirdLastPose, thirdLastTime, secondLastTime, thirdLastPose, secondLastPose);
        }
    } else if (_time < thirdLastTime) {
        boost::posix_time::time_duration timeElapse;
        timeElapse = uTimeDouble2Boost(thirdLastTime) - uTimeDouble2Boost(_time);
        if (timeElapse.total_milliseconds() > 2*timeInterval) {
            LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime - floor(lastTime) << ", time second last wheel: " << secondLastTime - floor(secondLastTime) << ", time third last wheel: " << thirdLastTime - floor(thirdLastTime) << ", the image:" << _time - floor(_time); 
        } else {
            const double deltaTime = thirdLastTime - _time;
            alignPose = velMotionModel(deltaTime, thirdLastPose, thirdLastTime, secondLastTime, secondLastPose, thirdLastPose);
        }
    } else {
    	LOG_ERROR << "Time stamps error, Time last wheel: " << lastTime - floor(lastTime) << ", time second last wheel: " << secondLastTime - floor(secondLastTime) << ", time third last wheel: " << thirdLastTime - floor(thirdLastTime) << ", the image:" << _time - floor(_time); 
    }

    return alignPose;
}

}   // namespace