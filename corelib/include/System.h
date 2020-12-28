#ifndef VISFS_SYSTEM
#define VISFS_SYSTEM

#include <queue>
#include <tuple>
#include <cstdarg>

#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>

#include "Parameters.h"
#include "Tracker.h"
#include "Estimator.h"
#include "Monitor.h"
#include "CameraModels/PinholeModel.h"
#include "Log.h"

class Logger;

namespace VISFS {

class System {

public:
    System(const ParametersMap & _parameters = ParametersMap());
    ~System();

    void init(const boost::shared_ptr<GeometricCamera> & _cameraLeft, const boost::shared_ptr<GeometricCamera> & _cameraRight);
    void init(const double fxl, const double fyl, const double cxl, const double cyl, const double fxr, const double fyr,
                const double cxr, const double cyr, const double baseline);

	/** \brief Interface to put the stereo image to our system.
      * \param[in] The time stamps of the images.  
      * \param[in] Left image. 
      * \param[in] Right iamge.
	  * \author eddy
      */    
    void inputStereoImage(const double _time, const cv::Mat & _imageLeft, const cv::Mat & _imageRight);

	/** \brief Interface to put the wheel odometry to our system.
      * \param[in] The time stamps of the wheel odometry.  
      * \param[in] The wheel odometry pose.
      * \param[in] The wheel odometry velocity.
	  * \author eddy
      */    
    void inputWheelOdometry(const double _time, const Eigen::Isometry3d & _pose, const Eigen::Isometry3d & _velocity);

    bool outputOdometryInfo(double & _stamp, Eigen::Isometry3d & _pose, TrackInfo & _trackInfo, EstimateInfo & _estimateInfo);

private:
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

	/** \brief Pridict the pose align with at the given timestamp.
      * \param[in] n The total number of wheel odometry which were inputed.
      * \param[in] time The given timestamp wanted to be align.  
      * \param[in] lastOdom The latest Odometry recieved.
      * \param[in] ... The extended parameters.
      * \return The predicted pose.
	  * \author eddy
      */ 
    Eigen::Isometry3d predictAlignPose(const int _n, const double _time, const std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d> & _lastOdom, ...);

    Eigen::Isometry3d accMotionModel(const double _deltaTime, const bool _dirction, const Eigen::Isometry3d & _pose, const Eigen::Isometry3d & _v1, const Eigen::Isometry3d & _v2) const;

    Eigen::Isometry3d velMotionModel(const double _deltaTime, const Eigen::Isometry3d & _basePose, const double & _time1, const double & _time2, Eigen::Isometry3d & _pose1, const Eigen::Isometry3d & _pose2) const;

    Eigen::Isometry3d getGuessPose(const Eigen::Isometry3d & _guessVelocity, const double _dt);

    Tracker * tracker_;
    boost::thread * threadTracker_;
    Estimator * estimator_;
    boost::thread * threadEstimator_;
    Monitor * monitor_;
    boost::thread * threadMonitor_;

    Logger * logger_;
    int logLevel_;
	bool console_;
	std::string logFolder_;

    boost::shared_ptr<GeometricCamera> cameraLeft_;
    boost::shared_ptr<GeometricCamera> cameraRight_;

    // state
    Eigen::Isometry3d velocityGuess_;
    Eigen::Isometry3d previousWheelOdom_;
    double previousTimeStamp_;

    // data buf
    boost::mutex mutexWheelOdometryBuf_;
    std::vector<std::tuple<double, Eigen::Isometry3d, Eigen::Isometry3d>> wheelOdometryBuf_;  // timestamp, pose, velocity

    bool monitorSwitch_;
    int sensorStrategy_;    // 0 Stereo, 1 rgbd, 2 stereo + wheel.
    int wheelFreq_;
	bool claheSwitch_;		// Contrast Limited Adaptive Histogram Equalization

};

}   // namespace VISFS

#endif  // VISFS_SYSTEM