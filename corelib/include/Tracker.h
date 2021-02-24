#ifndef VISFS_TRACKER
#define VISFS_TRACKER

#include <boost/thread.hpp>

#include "Parameters.h"
#include "Signature.h"
#include "Estimator.h"
#include "Extrapolator.h"
#include "Sensor/PointCloud.h"

namespace VISFS {

class Estimator;

class Tracker {
public:
    Tracker(const ParametersMap & _parameters = ParametersMap());
    ~Tracker();

    enum TrackingMethod {
        MONO,
        STEREO,
        RGBD        
    };

    void inputSignature(const Signature & _signature);
    void threadProcess(void);

    void setEstimator(Estimator * _estimator) { estimator_ = _estimator; }
    Estimator * getEstimator() const { return estimator_; }

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
     * \param[in] kptBlocked KeyPoint should be blocked. 
     * \return Mask.
     * \author eddy
     */
    cv::Mat getMask(const std::map<std::size_t, cv::KeyPoint> & _kptTo,  const int _rows, const int _cols, const std::map<std::size_t, cv::KeyPoint> & _kptBlocked) const;

    /** \brief The wrapper whole tracking procecude
     * \param[in] fromSignature The former signature.
     * \param[in] toSignature The current signature.
     * \param[in] guess The guess motion between fromSignature and toSignature. (from wheel odom or imu)
     * \author eddy
     */    
    void imageProcess(Signature & _fromSignature, Signature & _toSignature);

    /** \brief Pretreatment of the signature. For now, cull out the outliers, can add toSignature or other parameters to do other things.
     * \param[in] fromSignature The former signature.
     * \param[in] outliers The outliers estimated by estimator.
     * \author eddy
     */   
    void pretreatment(Signature & _fromSignature, const std::set<std::size_t> & _outliers);

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

};

}   // namespace

#endif  // VISFS_TRACKER