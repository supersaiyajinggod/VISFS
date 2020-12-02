#ifndef VISFS_LOCAL_MAP
#define VISFS_LOCAL_MAP

#include "Signature.h"
#include "Parameters.h"

namespace VISFS {

class FeatureStatusInSiganature {
public:
    FeatureStatusInSiganature(const cv::Point2f & _uv, const cv::Point2f & _uvRight, const cv::Point3f & _point3d) :
        uv(_uv), uvRight(_uvRight), point3d(_point3d) {}

    cv::Point2f uv, uvRight;    // Feature's pixel coordinate in image frame of each camera.
    cv::Point3f point3d;        // Feature's 3d coordinate in current signature robot frame.
    cv::Point2f velocity, velocityRight;    // Featrue motion speed on pixel.
};

class Feature {
public:
    Feature(const std::size_t _featureId, const std::size_t _signatureId, Eigen::Vector3d _featruePose) : 
        featureId_(_featureId), startSignatureId_(_signatureId), endSignatureId_(_signatureId), featureGlobalPose_(_featruePose), stateInLocalMap_(NEW_ADDED) {}

    enum eFeatureState {
        NEW_ADDED           =   0,
        STABLE              =   1
    };

    std::size_t getId() const { return featureId_; }
    std::size_t getStartSignatureId() const { return startSignatureId_; }
    std::size_t getEndSignatureId() const { return endSignatureId_; }
    void setEndSignatureId(const std::size_t _id) { endSignatureId_ = _id; }
    Eigen::Vector3d getFeaturePose() const { return featureGlobalPose_; }
    void setFeaturePose(const Eigen::Vector3d & _pose) { featureGlobalPose_ = _pose; } 
    eFeatureState getFeatureState() const { return stateInLocalMap_; }
    void setFeatureState(eFeatureState _state) { stateInLocalMap_ = _state; }
    std::size_t getObservedTimes() const { return featureStatusInSigantures_.size(); }

    std::map<std::size_t, FeatureStatusInSiganature> featureStatusInSigantures_;    // <signatrueId, FeatureStatusInSiganature>

private:
    std::size_t featureId_;
    std::size_t startSignatureId_;
    std::size_t endSignatureId_;
    Eigen::Vector3d featureGlobalPose_;
    eFeatureState stateInLocalMap_;
};

class LocalMap {
public:
    LocalMap(const ParametersMap & _parameters);
    ~LocalMap(){}

    bool insertSignature(const Signature & _signature, const Eigen::Vector3d & _translation);
    void removeSignature();

	/** \brief Update the local map.
      * \param[in] poses The pair of signature id and signature global pose.  
      * \param[in] point3d The pair of feature id and feature global pose. 
      * \param[in] outliers The outliers in the local map, calculated by local optimize.
	  * \author eddy
      */      
    void updateLocalMap(const std::map<std::size_t, Eigen::Isometry3d> & _poses, std::map<std::size_t, Eigen::Vector3d> & _point3d, std::set<std::size_t> & _outliers);

private:
    std::vector<std::size_t> findCorrespondences(const std::map<std::size_t, cv::KeyPoint> & _wordsFrom, const std::map<std::size_t, cv::KeyPoint> & _wordsTo);
    inline void clearCounters(); 
    inline bool checkCounters();

    bool keySignature_;
    int localMapSize_;
    int maxFeature_;
    float minParallax_;
    double minTranslation_; // To convenient calculation, we automatically calculate 3*translation^2.
    
    int newFeatureCount_;   // Count the number of new feature since the last key signature.
    int signatureCount_;    // Count the number of signature since the last key signature.
    float parallaxCount_;   // Count the total parallax since the last key signature.
    Eigen::Vector3d translationCount_; // Count the total translation since the last key signature.

    std::map<std::size_t, Signature> signatures_;
    std::map<std::size_t, Feature> features_;

};

}   // namespace

#endif  // VISFS_LOCAL_MAP