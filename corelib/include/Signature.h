#ifndef VISFS_SIGNATURE
#define VISFS_SIGNATURE

#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "CameraModels/GeometricCamera.h"

namespace VISFS {

class TrackInfo {
public:
    TrackInfo() :
    trackTime(0.0),
    inliers(0),
    inlierMeanDistance(0.f),
    inlierDistribution(0.f),
    matches(0) {}

    double trackTime;
    std::size_t inliers;
    float inlierMeanDistance;
    float inlierDistribution;
    std::vector<std::size_t> inliersIDs;
    std::size_t matches;
    std::vector<std::size_t> matchesIDs;
    std::vector<std::size_t> projectedIDs;  // "From" IDs
};

class EstimateInfo {
public:
    EstimateInfo() :
    lost(false),
    features(0),
    localMapSize(0),
    localScanMapSize(0),
    localKeyFrames(0),
    localBundleOutliers(0),
    localBundleConstraints(0),
    localBundleTime(0.f),
    keyFrameAdded(false),
    timeEstimation(0.f),
    timeParticleFiltering(0.f),
    stamp(0.0),
    interval(0.0),
    estimateTime(0.0),
    distanceTravelled(0.0),
    memoryUsage(0),
    gravityRollError(0.0),
    gravityPitchError(0.0) {}

	bool lost;
	int features;
	int localMapSize;
	int localScanMapSize;
	int localKeyFrames;
	int localBundleOutliers;
	int localBundleConstraints;
	float localBundleTime;
	std::map<std::size_t, Eigen::Isometry3d> localBundlePoses;
	std::map<std::size_t, GeometricCamera> localBundleModels;
	bool keyFrameAdded;
	float timeEstimation;
	float timeParticleFiltering;
	double stamp;
	double interval;
    double estimateTime;
    cv::Mat covariance;
	Eigen::Isometry3d transform;
	Eigen::Isometry3d transformFiltered;
	Eigen::Isometry3d transformGroundTruth;
	Eigen::Isometry3d guessVelocity;
	float distanceTravelled;
	int memoryUsage; //MB
	double gravityRollError;
	double gravityPitchError;

	int type;
	std::map<std::size_t, cv::KeyPoint> words;
	std::map<std::size_t, cv::Point3f> localMap;

	std::vector<cv::Point2f> refCorners;
	std::vector<cv::Point2f> newCorners;
	std::vector<std::size_t> cornerInliers;
};

class Signature {
public:
    static std::size_t nextId_;
    Signature();
    Signature(const double & _timestamp, const cv::Mat & _imageLeft, const cv::Mat & _imageRight, const boost::shared_ptr<GeometricCamera> & _cameraLeft, const boost::shared_ptr<GeometricCamera> & _cameraRight);

    std::size_t getId() const { return id_; }
    double getTimeStamp() const { return timestamp_; }
    Eigen::Isometry3d getPose() const { return pose_; }
    void setPose(const Eigen::Isometry3d & _pose) { pose_ = _pose; } 
    void setPose(const Eigen::Matrix3d & _R, const Eigen::Vector3d & _t);
    Eigen::Isometry3d getGuessPose() const { return guess_; }
    void setGuessPose(const Eigen::Isometry3d & _guess) { guess_ = _guess; }
    const std::map<std::size_t, cv::KeyPoint> & getWords() const { return words_; }
    void setWords(const std::map<std::size_t, cv::KeyPoint> & _words) { words_ = _words; }
    const std::map<std::size_t, cv::Point3f> & getWords3d() const { return words3d_; }
    void setWords3d(const std::map<std::size_t, cv::Point3f> & _words3d) { words3d_ = _words3d; }
    const std::map<std::size_t, cv::Mat> & getWordsDescriptors() const { return wordsDescriptors_; }
    void setWordsDescriptors(const std::map<std::size_t, cv::Mat> & _wordsDescriptors) { wordsDescriptors_ = _wordsDescriptors; }
    const std::map<std::size_t, cv::KeyPoint> & getCovisibleWords() const { return covisibleWords_; }
    void setCovisibleWords(const std::map<std::size_t, cv::KeyPoint> & _words) {  covisibleWords_ = _words; }
    const std::map<std::size_t, cv::Point3f> & getCovisibleWords3d() const { return covisibleWords3d_; }
    void setCovisibleWords3d(const std::map<std::size_t, cv::Point3f> & _words3d) { covisibleWords3d_ = _words3d; } 
    const std::map<std::size_t, cv::KeyPoint> & getKeyPointsMatchesFormer() const { return keyPointsMatchesFormer_; }
    void setkeyPointsMatchesFormer(const std::map<std::size_t, cv::KeyPoint> & _kpts) { keyPointsMatchesFormer_ = _kpts; }
    const std::map<std::size_t, cv::KeyPoint> & getKeyPointsNewExtract() const { return keyPointsNewExtracted_; }
    void setKeyPointsNewExtract(const std::map<std::size_t, cv::KeyPoint> & _kpts) { keyPointsNewExtracted_ = _kpts; }
    const std::map<std::size_t, cv::KeyPoint> & getKeyPointsMatchesImageRight() const { return keyPointsMatchesImageRight_; }
    void setKeyPointMatchesImageRight(const std::map<std::size_t, cv::KeyPoint> & _kpts) { keyPointsMatchesImageRight_ = _kpts; }
    const std::vector<unsigned char> & getLeftRightPairStatus() const { return leftRightPairStatus_; }
    void setLeftRightPairStatus(const std::vector<unsigned char> & _status) { leftRightPairStatus_ = _status; }

    bool empty() const;
    const cv::Mat & getImage() const { return imageLeft_; }
    const cv::Mat & getImageLeft() const { return imageLeft_; }
    const cv::Mat & getImageRight() const { return imageRight_; }
    const GeometricCamera & getCameraModel() const { return *cameraLeft_; }
    const GeometricCamera & getCameraModelLeft() const { return *cameraLeft_; }
    const GeometricCamera & getCameraModelRight() const { return *cameraRight_; }


    TrackInfo & getTrackInfo() { return trackInfo_; }
    void setTrackInfo(const TrackInfo & _trackInfo) { trackInfo_ = _trackInfo; }
    EstimateInfo & getEstimateInfo() { return estimateInfo_; }
    void setEstimateInfo(const EstimateInfo & _estimateInfo) { estimateInfo_ = _estimateInfo; }


private:
    std::size_t id_;
    double timestamp_;
    cv::Mat imageLeft_;
    cv::Mat imageRight_;
    boost::shared_ptr<GeometricCamera> cameraLeft_;
    boost::shared_ptr<GeometricCamera> cameraRight_;
    Eigen::Isometry3d pose_;
    Eigen::Isometry3d guess_;

    std::map<std::size_t, cv::KeyPoint> words_;
    std::map<std::size_t, cv::Point3f> words3d_; // word in robot/base_link
    std::map<std::size_t, cv::Mat> wordsDescriptors_;

    std::map<std::size_t, cv::KeyPoint> covisibleWords_;    // words in former signature
    std::map<std::size_t, cv::Point3f> covisibleWords3d_;  // words3d in former signature

    std::map<std::size_t, cv::KeyPoint> keyPointsMatchesFormer_;  // keypoints matches with the former signature
    std::map<std::size_t, cv::KeyPoint> keyPointsNewExtracted_;   // new keypoints in this signature
    std::map<std::size_t, cv::KeyPoint> keyPointsMatchesImageRight_;  // all keypoints, (keyPointsMatchesFormer_ +  keyPointsNewExtracted_) also keeping the order, matches in right image.
    std::vector<unsigned char> leftRightPairStatus_;    // all keypoints' match status in left and right image.

    TrackInfo trackInfo_;
    EstimateInfo estimateInfo_;

};

}   // namespace

#endif  // VISFS_SIGNATURE