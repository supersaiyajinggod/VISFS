#include "Estimator.h"
#include "Optimizer.h"
#include "MultiviewGeometry.h"

#include <opencv2/core/eigen.hpp>

namespace VISFS {

void Estimator::addSignature(const Signature & _signature) {
    boost::lock_guard<boost::mutex> lock(mutexDataRW_);
    signatureThreadBuf_.emplace(_signature);
}

void Estimator::threadProcess() {
    while (1) {
        Signature signature;
        {
            boost::lock_guard<boost::mutex> lock(mutexDataRW_);
            if (!signatureThreadBuf_.empty()) {
                signature = signatureThreadBuf_.front();
                signatureThreadBuf_.pop();
            }
        }
        process(signature);

        //publish process result
        boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(5));
    }
}

void Estimator::process(Signature & _signature) {
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
    if (!transform.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero())) && inliers.size() > minInliers_) {
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
        Eigen::Isometry3d transformRobotToImage = _signature.getCameraModel().getTansformImageToRobot().inverse();
        cameraModels.emplace_back(_signature.getCameraModelLeft());
        cameraModels.emplace_back(_signature.getCameraModelRight());

        for (std::size_t i = 0; i < inliers.size(); ++i) {
            std::size_t wordId = inliers[i];
            const cv::Point3f & cvpt3d = _signature.getCovisibleWords3d().find(wordId)->second;
            points3D.emplace(wordId, Eigen::Vector3d(cvpt3d.x, cvpt3d.y, cvpt3d.z));

            std::map<std::size_t, FeatureBA> ptMap;
            float depthFrom = transformPoint(cvpt3d, transformRobotToImage).z;
            const cv::KeyPoint & kptFrom = _signature.getCovisibleWords().find(wordId)->second;
            ptMap.emplace(1, FeatureBA(kptFrom, depthFrom));
            float depthTo = 0.f;
            if (_signature.getWords3d().find(wordId) != _signature.getWords3d().end()) {
                depthTo = transformPoint(_signature.getWords3d().find(wordId)->second, transformRobotToImage).z;
            }
            const cv::KeyPoint & kptTo = _signature.getWords().find(wordId)->second;
            ptMap.emplace(2, FeatureBA(kptTo, depthTo));
            wordReferences.emplace(wordId, ptMap);
        }

        std::map<std::size_t, Eigen::Isometry3d> optimizedPoses;
        std::set<std::size_t> sbaOutliers;
        optimizedPoses = optimizer_->poseOptimize(1, poses, links, cameraModels, points3D, wordReferences, sbaOutliers);

        // Update BA result
        if (optimizedPoses.size() == 2 && !optimizedPoses.begin()->second.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))
                && !optimizedPoses.rbegin()->second.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))){
            if (sbaOutliers.size()) {
                std::vector<std::size_t> newInliers(inliers.size());
                std::size_t oi = 0;
                for (std::size_t i = 0; i < inliers.size(); ++i) {
                    if (sbaOutliers.find(inliers[i]) == sbaOutliers.end()) {
                        newInliers[oi++] = inliers[i];
                    }
                }
                newInliers.resize(oi);
                inliers = newInliers;
            }
            if (static_cast<int>(inliers.size()) < minInliers_) {
                std::cout << "Not enough inliers after bundle adjustment " << inliers.size() << "/" << minInliers_ << " between signature " << _signature.getId() - 1  << " and " << _signature.getId() << std::endl;
                transform = Eigen::Isometry3d(Eigen::Matrix4d::Zero());
            } else {
                transform = optimizedPoses.rbegin()->second;
            }

            // Update 3d points in signture.
            std::map<std::size_t, cv::Point3f> cpyWords3dTo = _signature.getWords3d();
            Eigen::Isometry3d invT = transform.inverse();
            for (auto iter = points3D.begin(); iter != points3D.end(); ++iter) {
                if (cpyWords3dTo.find(iter->first) != cpyWords3dTo.end())
                    cpyWords3dTo.find(iter->first)->second = transformPoint(cv::Point3f(iter->second[0], iter->second[1], iter->second[2]), invT);
            }
            _signature.setWords3d(cpyWords3dTo);           
        } else {
            transform = Eigen::Isometry3d(Eigen::Matrix4d::Zero());
            _signature.setPose(transform);
        }
    }
}

}   // namespace