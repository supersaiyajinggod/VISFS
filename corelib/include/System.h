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
#include "Extrapolator.h"

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

	Extrapolator * extrapolator_;

    bool monitorSwitch_;
    int sensorStrategy_;    // 0 Stereo, 1 rgbd, 2 stereo + wheel.
	bool claheSwitch_;		// Contrast Limited Adaptive Histogram Equalization

};

}   // namespace VISFS

#endif  // VISFS_SYSTEM