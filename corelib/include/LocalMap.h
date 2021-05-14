#ifndef VISFS_LOCAL_MAP
#define VISFS_LOCAL_MAP

#include "Signature.h"
#include "Optimizer/Optimizer.h"
#include "Parameters.h"
#include "Map/2d/Submap2D.h"

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

	/** \brief Check the local map has enough vertex and edges to do local optimize.
      * \return Ture: Map is avaliable.
	  * \author eddy
      */  
    bool checkMapAvaliable();

	/** \brief Update the local map.
      * \param[in] poses The pair of signature id and signature global pose.  
      * \param[in] point3d The pair of feature id and feature global pose. 
      * \param[in] outliers The error edge in the local map, calculated by local optimize.
      * \param[out] errorVertex The error vertex in local map, which should be blocked.
	  * \author eddy
      */      
    void updateLocalMap(const std::map<std::size_t, Eigen::Isometry3d> & _poses, const std::map<std::size_t, std::tuple<Eigen::Vector3d, bool>> & _point3d, const std::vector<std::tuple<std::size_t, std::size_t>> & _outliers, std::set<std::size_t> & _errorVertex);

	/** \brief Get all signature's poses.
      * \param[out] poses The signature vertexs of local map graph. With map strcture: <signature id, signature pose>.  
	  * \author eddy
      */  
    bool getSignaturePoses(std::map<std::size_t, Eigen::Isometry3d> & _poses);

	/** \brief Get link constraint between signatures.
      * \param[out] links The constraint. With map strcture: <link id, <vertex From, vertex To, transform, covariance>>>.  
	  * \author eddy
      */  
    bool getSignatureLinks(std::map<std::size_t,std::tuple<std::size_t, std::size_t, Eigen::Isometry3d>> & _links);

	/** \brief Get all features' pose that has been multiple signature observed with each observations.
      * \param[out] points The feature poses. With map strcture: <feature id, feature poses>.
      * \param[out] observations The observations of every feature. With map structure: <feature id, <signature id, observation>>
	  * \author eddy
      */  
    bool getFeaturePosesAndObservations(std::map<std::size_t, std::tuple<Eigen::Vector3d, bool>> & _points, std::map<std::size_t, std::map<std::size_t, Optimizer::FeatureBA>> & _observations);

    const std::vector<Sensor::PointCloud> getLaserHitPointCloud(std::size_t _signatureId); 

    inline std::shared_ptr<const Map::Submap2D> getMatchingSubmap2D() { return activeSubmap2D_->submaps().front(); }

    std::vector<std::shared_ptr<const Map::Submap2D>> insertMatchingSubMap2d(const std::vector<Sensor::RangeData> & _rangeDatas, const Eigen::Isometry3d & _globalPose);

    inline bool hasMatchingSubmap2D() { return !activeSubmap2D_->submaps().empty(); }

private:
	/** \brief Find corresponding pairs between two groups of features.
      * \param[in] wordsFrom The a group of features.
      * \param[in] wordsTo Another group of features.
      * \return The words has same id.
	  * \author eddy
      */  
    std::vector<std::size_t> findCorrespondences(const std::map<std::size_t, cv::KeyPoint> & _wordsFrom, const std::map<std::size_t, cv::KeyPoint> & _wordsTo);

	/** \brief Clear all counters in unified.
	  * \author eddy
      */  
    inline void clearCounters(); 
    inline bool checkCounters();

    bool keySignature_;
    int localMapSize_;
    int maxFeature_;
    float minParallax_;
    double minTranslation_; // To convenient calculation, we automatically calculate 3*translation^2.
    int minInliers_;
    int numRangeDataLimit_;
    Map::GridType gridType_;
    double mapResolution_;
    bool insertFreeSpace_;
    double hitProbability_;
    double missProbability_;
    
    int newFeatureCount_;   // Count the number of new feature since the last key signature.
    int signatureCount_;    // Count the number of signature since the last key signature.
    float parallaxCount_;   // Count the total parallax since the last key signature.
    Eigen::Vector3d translationCount_; // Count the total translation since the last key signature.

    std::map<std::size_t, Signature> signatures_;
    std::map<std::size_t, Feature> features_;
    std::unique_ptr<Map::ActiveSubmaps2D> activeSubmap2D_;

};

}   // namespace

#endif  // VISFS_LOCAL_MAP