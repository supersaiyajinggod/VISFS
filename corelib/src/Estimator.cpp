#include <opencv2/core/eigen.hpp>

#include "Estiamtor.h"
#include "MultiviewGeometry.h"

namespace VISFS {

void Estimator::addSignature(const Signature & _signature) {
    boost::lock_guard<boost::mutex> lock(mutexDataRW_);
    signatureThreadBuf_.emplace(_signature);
}

void Estimator::process(const Signature & _signature) {
    Eigen::Isometry3d transform;
    cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1);
    std::vector<std::size_t> matches;
    std::vector<std::size_t> inliers;
    std::map<std::size_t, cv::Point3f> words3dFrom = _signature.getCovisibleWords3d();
    std::map<std::size_t, cv::KeyPoint> wordsTo = _signature.getKeyPointsMatchesFormer();
    std::map<std::size_t, cv::Point3f> words3dTo = _signature.getWords3d();
    if (words3dFrom.size() >= minInliers_ && wordsTo.size() >= minInliers_) {
        transform = estimateMotion3DTo2D(words3dFrom, wordsTo, _signature.getCameraModel(), minInliers_, pnpIterations_,
                                            pnpReprojError_, pnpFlags_, refineIterations_, words3dTo, covariance, matches, inliers,
                                            _signature.getGuessPose());
    } else {
        std::cout << "Not enough features in images, old=" << words3dFrom.size() << ", new=" << wordsTo.size() << ", min=" << minInliers_ << "." << std::endl;
    }
    
    // If motion of guess > threshold, do bundle adjustment.
    if (1) {
        std::map<std::size_t, Eigen::Isometry3d> poses;
        std::map<std::size_t,std::tuple<std::size_t, std::size_t, Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> links;
        std::vector<GeometricCamera> cameraModels;
        std::map<std::size_t, Eigen::Vector3d> points3D;
        std::map<std::size_t, std::map<std::size_t, FeatureBA>> wordReferences;
        std::set<std::size_t> outliers;

        poses.insert(std::make_pair(1, Eigen::Isometry3d::Identity()));
        poses.insert(std::make_pair(2, transform));

		if (covariance.at<double>(0, 0) <= COVARIANCE_EPSILON)
			covariance.at<double>(0, 0) = COVARIANCE_EPSILON;
		if (covariance.at<double>(1, 1) <= COVARIANCE_EPSILON)
			covariance.at<double>(1, 1) = COVARIANCE_EPSILON;
		if (covariance.at<double>(2, 2) <= COVARIANCE_EPSILON)
			covariance.at<double>(2, 2) = COVARIANCE_EPSILON;
		if (covariance.at<double>(3, 3) <= COVARIANCE_EPSILON)
			covariance.at<double>(3, 3) = COVARIANCE_EPSILON;
		if (covariance.at<double>(4, 4) <= COVARIANCE_EPSILON)
			covariance.at<double>(4, 4) = COVARIANCE_EPSILON;
		if (covariance.at<double>(5, 5) <= COVARIANCE_EPSILON)
			covariance.at<double>(5, 5) = COVARIANCE_EPSILON; 
        Eigen::MatrixXd m(covariance.rows, covariance.cols);
        cv::cv2eigen(covariance, m);
        links.insert(std::make_pair(1, std::make_tuple(1, 2, transform, m.inverse())));

        //camera model

    }
}

}   // namespace