#ifndef VISFS_TRACKER
#define VISFS_TRACKER

#include <boost/thread.hpp>

#include "Parameters.h"
#include "Signature.h"
#include "Estimator.h"
#include "Monitor.h"

namespace VISFS {

class Tracker {
public:
    Tracker(Estimator * _estimator, const ParametersMap & _parameters = ParametersMap());
    ~Tracker();

    enum TrackingMethod {
        MONO,
        STEREO,
        RGBD        
    };

    void inputSignature(const Signature & _signature);
    void threadProcess(void);

    void setMonitor(Monitor * _monitor) { monitor_ = _monitor; }
    Monitor * getMonitor() const { return monitor_; }

private:

    /** \brief Reject the outlier of keypoint matches by fundation matrix.
     * \param[in] cornersFrom The corners in the from signature.
     * \param[in] cornersTo The corners in the to signature.
     * \return The status of the matches. 1: good match, 0: bad match.
     * \author eddy
     */
    void rejectOutlierWithFundationMatrix(const std::vector<cv::Point2f> & _cornersFrom, const std::vector<cv::Point2f> & _cornersTo, std::vector<unsigned char> & _status) const;

    /** \brief Update the feature observed counters.
     * \param[in] wordIds All words observed in current signature.
     * \author eddy
     */
    void updateTrackCounter(std::map<std::size_t, cv::KeyPoint> _wordIds);

    /** \brief Get a mask for new feature extract.
     * \param[in] kptTo KeyPoint in the current signature.
     * \param[in] rows The total rows of mask.
     * \param[in] cols The total cols of mask.
     * \return Mask.
     * \author eddy
     */
    cv::Mat getMask(std::map<std::size_t, cv::KeyPoint> _kptTo, int _rows, int _cols);

    /** \brief The wrapper whole tracking procecude
     * \param[in] fromSignature The former signature.
     * \param[in] toSignature The current signature.
     * \param[in] guess The guess motion between fromSignature and toSignature. (from wheel odom or imu)
     * \author eddy
     */    
    void process(Signature & _fromSignature, Signature & _toSignature);

    Signature lastSignature_;

    int maxFeature_;
    double qualityLevel_;
    int minFeatureDistance_;
    float maxDepth_;
    float minDepth_;
    bool flowBack_;
    int flowWinSize_;
    int flowIterations_;
    float flowEps_;
    int flowMaxLevel_;
    bool cullByFundationMatrix_;
    float fundationPixelError_;
    int minInliers_;

    std::map<std::size_t, std::size_t> trackCnt_;

    TrackingMethod trackingMethod_;
    std::size_t globalFeatureId_;

    boost::mutex mutexDataBuf_;
    std::queue<Signature> signatureBuf_;

    Estimator * estimator_;
    Monitor * monitor_;

};

}   // namespace

#endif  // VISFS_TRACKER