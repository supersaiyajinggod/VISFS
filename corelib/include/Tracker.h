#ifndef VISFS_TRACKER
#define VISFS_TRACKER

#include "Parameters.h"
#include "Signature.h"

namespace VISFS {

class Tracker {
public:
    Tracker(const ParametersMap & _parameters = ParametersMap());
    ~Tracker();

    enum TrackingMethod {
        MONO,
        STEREO,
        RGBD        
    };

private:

    /** \brief Reject the outlier of keypoint matches by fundation matrix.
     * \param[in] cornersFrom The corners in the from signature.
     * \param[in] cornersTo The corners in the to signature.
     * \return The status of the matches. 1: good match, 0: bad match.
     * \author eddy
     */
    void rejectOutlierWithFundationMatrix(const std::vector<cv::Point2f> & _cornersFrom, const std::vector<cv::Point2f> & _cornersTo, std::vector<unsigned char> & _status) const;

    /** \brief The wrapper whole tracking procecude
     * \param[in] fromSignature The former signature.
     * \param[in] toSignature The current signature.
     * \param[in] guess The guess motion between fromSignature and toSignature. (from wheel odom or imu)
     * \author eddy
     */    
    void process(Signature & _fromSignature, Signature & _toSignature, Eigen::Isometry3d _guess = Eigen::Isometry3d::Identity());

    Signature lastSignature_;

    int maxFeature_;
    double qualityLevel_;
    int minFeatureDistance_;
    bool flowBack_;
    float maxDepth_;
    float minDepth_;
    int flowWinSize_;
    int flowIterations_;
    float flowEps_;
    int flowMaxLevel_;
    bool cullByFundationMatrix_;
    float fundationPixelError_;

    TrackingMethod trackingMethod_;
    std::size_t globalFeatureId_;
};

}   // namespace

#endif  // VISFS_TRACKER