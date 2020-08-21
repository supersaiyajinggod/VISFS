#ifndef VISFS_SYSTEM
#define VISFS_SYSTEM

#include <queue>
#include <tuple>

#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>

namespace VISFS {

class System {

public:
    System();

	/** \brief Interface to put the stereo image to our system.
      * \param[in] The time stamps of the images.  
      * \param[in] Left image. 
      * \param[in] Right iamge.
	  * \author eddy
      */    
    void inputStereoImage(const double time_, const cv::Mat & imageLeft_, const cv::Mat & imageRight_);


private:
	boost::mutex mutexDataBuf_;

	std::queue<std::tuple<double, cv::Mat, cv::Mat>> stereoImageBuf_;

};

}   // namespace VISFS

#endif  // VISFS_SYSTEM