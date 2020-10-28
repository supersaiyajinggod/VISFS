#include "Estimator.h"
#include "MultiviewGeometry.h"
#include "ProcessInfo.h"
#include "Math.h"
#include "Timer.h"

#include <opencv2/core/eigen.hpp>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

namespace VISFS {

Estimator::Estimator(const ParametersMap & _parameters) :
    pose_(Eigen::Isometry3d::Identity()),
    velocityGuess_(Eigen::Isometry3d(Eigen::Matrix4d::Zero())),
    previousStamps_(0.0),
    minInliers_(Parameters::defaultEstimatorMinInliers()),
    pnpIterations_(Parameters::defaultEstimatorPnPIterations()),
    pnpReprojError_(Parameters::defaultEstimatorPnPReprojError()),
    pnpFlags_(Parameters::defaultEstimatorPnPFlags()),
    refineIterations_(Parameters::defaultEstimatorRefineIterations()),
    toleranceTranslation_(Parameters::defaultEstimatorToleranceTranslation()),
    toleranceRotation_(Parameters::defaultEstimatorToleranceRotation()),
    force3D_(Parameters::defaultEstimatorForce3DoF()) {

    Parameters::parse(_parameters, Parameters::kEstimatorMinInliers(), minInliers_);
    Parameters::parse(_parameters, Parameters::kEstimatorPnPIterations(), pnpIterations_);
    Parameters::parse(_parameters, Parameters::kEstimatorPnPReprojError(), pnpReprojError_);
    Parameters::parse(_parameters, Parameters::kEstimatorPnPFlags(), pnpFlags_);
    Parameters::parse(_parameters, Parameters::kEstimatorRefineIterations(), refineIterations_);
    Parameters::parse(_parameters, Parameters::kEstimatorForce3DoF(), force3D_);

    optimizer_ = new Optimizer(_parameters);
}

Estimator::~Estimator() {
    delete optimizer_;
}

void Estimator::inputSignature(const Signature & _signature) {
    boost::lock_guard<boost::mutex> lock(mutexDataRW_);
    signatureThreadBuf_.emplace(_signature);
}

void Estimator::outputSignature(const Signature & _signature) {
    boost::lock_guard<boost::mutex> lock(mutexResultRW_);
    processResultBuf_.emplace(_signature);
}

Signature Estimator::getEstimatedSignature() {
    Signature signature;
    boost::lock_guard<boost::mutex> lock(mutexResultRW_);
    if (!processResultBuf_.empty()) {
        signature = processResultBuf_.front();
        processResultBuf_.pop();
    }
    return signature;
}

void Estimator::threadProcess() {
    while (1) {
        Signature signature;

        if (!signatureThreadBuf_.empty()) {
            {
                boost::lock_guard<boost::mutex> lock(mutexDataRW_);
                signature = signatureThreadBuf_.front();
                signatureThreadBuf_.pop();
            }
            // UTimer timer;
            process(signature);
            // timer.elapsed("Estimator");

            //publish process result
            outputSignature(signature);
        }
        
        boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(5));
    }
}

void Estimator::process(Signature & _signature) {
    TrackInfo & trackInfo = _signature.getTrackInfo();
    EstimateInfo & estimateInfo = _signature.getEstimateInfo();
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
        transform = Eigen::Isometry3d(Eigen::Matrix4d::Zero());
        std::cout << "Not enough features in images, old=" << words3dFrom.size() << ", new=" << wordsTo.size() << ", min=" << minInliers_ << "." << std::endl;
    }
    
    // If motion of guess > threshold, do bundle adjustment.
    if (!transform.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero())) && inliers.size() > minInliers_) {
        std::map<std::size_t, Eigen::Isometry3d> poses;
        std::map<std::size_t,std::tuple<std::size_t, std::size_t, Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> links;
        std::vector<boost::shared_ptr<GeometricCamera>> cameraModels;
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
        cameraModels.push_back(_signature.getCameraModelLeftPtr());
        cameraModels.push_back(_signature.getCameraModelRightPtr());
        // std::cout << "Estimate camera K : \n" << cameraModels[0]->eigenKdouble() << "  \n baselne : " << cameraModels[0]->getBaseLine() << std::endl;

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
        // std::cout << "sbaOutliers.size(): " << sbaOutliers.size() << std::endl;
        // for (auto iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++ iter) {
        //     std::cout << "optimizedPoses id : " << iter->first << "  pose is :\n" << iter->second.matrix() << std::endl;
        // }

        // Update BA result
        if (optimizedPoses.size() == 2 && !optimizedPoses.begin()->second.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))
                && !optimizedPoses.rbegin()->second.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
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
        }
    }

    Eigen::Isometry3d wheelOdom;
    if (_signature.getWheelOdomPose(wheelOdom)) {
        // std::cout << "Signatrue id: " << _signature.getId() << ", estimator get wheel odom: \n" << wheelOdom.matrix() << std::endl;
        // std::cout << "BA transform: \n" << transform.matrix() << std::endl;
        double wheelX, wheelY, wheelZ, wheelRoll, wheelPitch, wheelYaw, visualX, visualY, visualZ, visualRoll, visualPitch, visualYaw;
        pcl::getTranslationAndEulerAngles(wheelOdom, wheelX, wheelY, wheelZ, wheelRoll, wheelPitch, wheelYaw);
        pcl::getTranslationAndEulerAngles(transform, visualX, visualY, visualZ, visualRoll, visualPitch, visualYaw);
        const double deltaX = wheelX - visualX;
        const double deltaY = wheelY - visualY;
        const double deltaZ = wheelZ - visualZ;
        const double deltaRoll = wheelRoll - visualRoll;
        const double deltaPitch = wheelPitch - visualPitch;
        const double deltaYaw = wheelYaw - visualYaw;
        if ((deltaX*deltaX + deltaY*deltaY) > toleranceTranslation_*toleranceTranslation_ ) {
            std::cout << "Signatrue id : " << _signature.getId() << " has a large Translation. deltaX: " << deltaX << " deltaY: " << deltaY << std::endl;
            transform = wheelOdom;
        }
        // if (deltaYaw*deltaYaw > toleranceRotation_*toleranceRotation_) {
        //     std::cout << "Signatrue id : " << _signature.getId() << " has a large rotation. deltaYaw: " << deltaYaw << std::endl;
        // }
    }

    if (force3D_) {
        double x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transform, x, y, z, roll, pitch, yaw);
        Eigen::Affine3d pose;
        pcl::getTransformation(x, y, 0, 0, 0, yaw, pose);
        transform = Eigen::Isometry3d(pose.matrix());
    }

    trackInfo.inliersIDs = inliers;
    trackInfo.matchesIDs = matches;
    estimateInfo.covariance = covariance;
    for (auto iter = _signature.getCovisibleWords3d().begin(); iter != _signature.getCovisibleWords3d().end(); ++iter) {
        estimateInfo.localMap.emplace(iter->first, transformPoint(iter->second, transform));
    }
    estimateInfo.localMapSize = _signature.getCovisibleWords3d().size();
    estimateInfo.words = _signature.getWords();
    estimateInfo.features = _signature.getWords().size();
    estimateInfo.transform = transform;
    estimateInfo.stamp = _signature.getTimeStamp();
    const double dt = _signature.getTimeStamp() - previousStamps_;
    estimateInfo.interval = dt;

    if (transform.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
        estimateInfo.lost = true;
        estimateInfo.guessVelocity = Eigen::Isometry3d(Eigen::Matrix4d::Zero());
        _signature.setPose(transform);
    } else {
        estimateInfo.lost = false;
        pose_ = pose_ * transform;
        Eigen::Vector3d translation = transform.translation();
        estimateInfo.distanceTravelled = static_cast<float>(uNorm(translation.x(), translation.y(), translation.z()));
        // estimateInfo.memoryUsage = UProcessInfo::getMemoryUsage()/(1024*1024);
        Eigen::Isometry3d velocity =  guessVelocity(transform, dt);
        estimateInfo.guessVelocity = velocity;
        _signature.setPose(pose_);
    }
    _signature.setTrackInfo(trackInfo);
    _signature.setEstimateInfo(estimateInfo);
    previousStamps_ = _signature.getTimeStamp();
    
}


Eigen::Isometry3d Estimator::guessVelocity(const Eigen::Isometry3d & _t, const double _dt) {
    double vx, vy, vz, vroll, vpitch, vyaw;
    if (_dt > 0) {
        pcl::getTranslationAndEulerAngles(_t, vx, vy, vz, vroll, vpitch, vyaw);
        vx /= _dt;
        vy /= _dt;
        vz /= _dt;
        vroll /= _dt;
        vpitch /= _dt;
        vyaw /= _dt;
        Eigen::Affine3d velocity;
        pcl::getTransformation(vx, vy, vz, vroll, vpitch, vyaw, velocity);
        velocityGuess_ = Eigen::Isometry3d(velocity.matrix());
        return velocityGuess_;
    } else {
        std::cout << "Error with value dt: " << _dt << std::endl;
    }

    return Eigen::Isometry3d::Identity();
}

}   // namespace