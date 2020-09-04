#ifndef VISFS_SYSTEM
#define VISFS_SYSTEM

#include <queue>
#include <tuple>

#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>

#include "Parameters.h"
#include "Tracker.h"
#include "Estimator.h"
#include "CameraModels/PinholeModel.h"

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

    bool outputOdometryInfo(Eigen::Isometry3d & _pose, TrackInfo & _trackInfo, EstimateInfo & _estimateInfo);


private:
    Eigen::Isometry3d getGuessPose(const Eigen::Isometry3d & _guessVelocity, const double _dt);

    Tracker * tracker_;
    boost::thread * threadTracker_;
    Estimator * estimator_;
    boost::thread * threadEstimator_;

    boost::shared_ptr<GeometricCamera> cameraLeft_;
    boost::shared_ptr<GeometricCamera> cameraRight_;

    // state
    Eigen::Isometry3d velocityGuess_;
    double previousTimeStamp_;

};

}   // namespace VISFS

#endif  // VISFS_SYSTEM