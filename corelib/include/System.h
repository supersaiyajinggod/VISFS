#ifndef VISFS_SYSTEM
#define VISFS_SYSTEM

#include <queue>
#include <tuple>
#include <cstdarg>

#include <opencv2/core/core.hpp>
#include <thread>
#include <mutex>

#include "Parameters.h"
#include "Tracker.h"
#include "Estimator.h"
#include "Monitor.h"
#include "CameraModels/PinholeModel.h"
#include "Log.h"
#include "Extrapolator.h"

class Logger;

namespace VISFS {

class System {

public:
    System(const ParametersMap & _parameters = ParametersMap());
    ~System();

    void init(const std::shared_ptr<GeometricCamera> & _cameraLeft, const std::shared_ptr<GeometricCamera> & _cameraRight,
                    const Eigen::Isometry3d & _transformCamera2Robot, const Eigen::Isometry3d & _transformLaser2Robot);
    void init(const double fxl, const double fyl, const double cxl, const double cyl, const double fxr, const double fyr,
                const double cxr, const double cyr, const double baseline,
                const Eigen::Isometry3d & _transformCamera2Robot, const Eigen::Isometry3d & _transformLaser2Robot);

	/** \brief Interface to put the stereo image to our system.
      * \param[in] time The time stamps of the images.  
      * \param[in] imageLeft Left image. 
      * \param[in] imageRight Right image.
      * \param[in] timedPointCloud Timed point Cloud.
	  * \author eddy
      */    
    void inputPrimarySensorData(const double _time, const cv::Mat & _imageLeft, const cv::Mat & _imageRight, const Sensor::TimedPointCloudWithIntensities & _timedPointCloud);

	/** \brief Interface to put the wheel odometry to our system.
      * \param[in] The time stamps of the wheel odometry.  
      * \param[in] The wheel odometry pose.
      * \param[in] The wheel odometry velocity.
	  * \author eddy
      */    
    void inputWheelOdometry(const double _time, const Eigen::Isometry3d & _pose, const Eigen::Isometry3d & _velocity);

    bool outputOdometryInfo(double & _stamp, Eigen::Isometry3d & _pose, TrackInfo & _trackInfo, EstimateInfo & _estimateInfo);

private:
    Tracker * tracker_;
    std::thread * threadTracker_;
    Estimator * estimator_;
    std::thread * threadEstimator_;
    Monitor * monitor_;
    std::thread * threadMonitor_;

    Logger * logger_;
    int logLevel_;
	bool console_;
	std::string logFolder_;

    std::shared_ptr<GeometricCamera> cameraLeft_;
    std::shared_ptr<GeometricCamera> cameraRight_;
    Eigen::Isometry3d transformCamera2Robot_;
    Eigen::Isometry3d transformLaser2Robot_;

	Extrapolator * extrapolator_;

    bool monitorSwitch_;
    int sensorStrategy_;    // 0 Stereo, 1 rgbd, 2 stereo + wheel, 3 stereo + laser + wheel.
	bool claheSwitch_;		// Contrast Limited Adaptive Histogram Equalization

};

}   // namespace VISFS

#endif  // VISFS_SYSTEM