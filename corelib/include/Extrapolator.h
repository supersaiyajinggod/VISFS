#ifndef VISFS_EXTRAPOLATOR
#define VISFS_EXTRAPOLATOR

#include <tuple>
#include <vector>
#include <list>
#include <Eigen/Geometry>
#include <boost/thread.hpp>

#include "Parameters.h"

namespace VISFS {

class Extrapolator {
public:
    Extrapolator(const ParametersMap & _parameters);
    ~Extrapolator();

    void setVelocityGuess(const Eigen::Isometry3d & _velocity) { velocityGuess_ = _velocity; }

    void addOdometry(const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _odom);
    void addOdometry(const double _time, const Eigen::Isometry3d & _pose, const Eigen::Isometry3d & _velocity);

    void extrapolatorPose(const double _time, Eigen::Isometry3d & globalPose, Eigen::Isometry3d & deltaPose);

private:
    Eigen::Isometry3d extrapolateFromVelocity(const Eigen::Isometry3d & _guessVelocity, const double _dt);

	/** \brief Pridict the pose align with at the given timestamp.
      * \param[in] time The given timestamp wanted to be align.  
      * \param[in] lastOdom The latest Odometry recieved.
      * \param[in] secondLastOdom The seccond last Odometry recieved
      * \return The predicted pose.
	  * \author eddy
      */    
    Eigen::Isometry3d predictAlignPose(const double _time, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _lastOdom, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _secondLastOdom) const;

	/** \brief Pridict the pose align with at the given timestamp.
      * \param[in] time The given timestamp wanted to be align.  
      * \param[in] lastOdom The latest Odometry recieved.
      * \param[in] secondLastOdom The seccond last Odometry recieved
      * \param[in] thridLastOdom The thrid last Odometry recieved
      * \return The predicted pose.
	  * \author eddy
      */    
    Eigen::Isometry3d predictAlignPose(const double _time, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _lastOdom, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _secondLastOdom,
                                                           const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _thridLastOdom) const;

    inline bool hasAvaliableOdometry() const { return !wheelOdometryBuf_.empty() && wheelOdometryBuf_.size() > 2; }

    std::vector<std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d>> getApproximateOdometry(const double _time);

    Eigen::Isometry3d accMotionModel(const double _deltaTime, const bool _dirction, const Eigen::Isometry3d & _pose, const Eigen::Isometry3d & _v1, const Eigen::Isometry3d & _v2) const;

    Eigen::Isometry3d velMotionModel(const double _deltaTime, const Eigen::Isometry3d & _basePose, const double & _time1, const double & _time2, Eigen::Isometry3d & _pose1, const Eigen::Isometry3d & _pose2) const;

    // state
    Eigen::Isometry3d velocityGuess_;
    Eigen::Isometry3d previousWheelOdom_;
    double previousTimeStamp_;

    // data buf
    boost::mutex mutexWheelOdometryBuf_;
    std::list<std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d>> wheelOdometryBuf_;  // timestamp, pose, velocity

    int sensorStrategy_;    // 0 Stereo, 1 rgbd, 2 stereo + wheel.
    int wheelFreq_;
};

}   // namespace

#endif  // VISFS_EXTRAPOLATOR