#ifndef VISFS_SIGNATURE
#define VISFS_SIGNATURE

#include <map>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "CameraModels/GeometricCamera.h"

namespace VISFS {

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

};

}   // namespace

#endif  // VISFS_SIGNATURE