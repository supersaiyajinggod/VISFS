#include "Estimator.h"
#include "MultiviewGeometry.h"
#include "ProcessInfo.h"
#include "Math.h"
#include "Timer.h"
#include "Stl.h"
#include "Log.h"

#include <opencv2/core/eigen.hpp>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

namespace VISFS {

Estimator::Estimator(const ParametersMap & _parameters) :
    pose_(Eigen::Isometry3d::Identity()),
    previousWheelOdom_(Eigen::Isometry3d::Identity()),
    velocityGuess_(Eigen::Isometry3d(Eigen::Matrix4d::Zero())),
    tracker_(nullptr),
    monitor_(nullptr),
    previousStamps_(0.0),
    sensorStrategy_(Parameters::defaultSystemSensorStrategy()),
    minInliers_(Parameters::defaultEstimatorMinInliers()),
    pnpIterations_(Parameters::defaultEstimatorPnPIterations()),
    pnpReprojError_(Parameters::defaultEstimatorPnPReprojError()),
    pnpFlags_(Parameters::defaultEstimatorPnPFlags()),
    refineIterations_(Parameters::defaultEstimatorRefineIterations()),
    toleranceTranslation_(Parameters::defaultEstimatorToleranceTranslation()),
    toleranceRotation_(Parameters::defaultEstimatorToleranceRotation()),
    force3Dof_(Parameters::defaultEstimatorForce3DoF()) {

    Parameters::parse(_parameters, Parameters::kSystemSensorStrategy(), sensorStrategy_);
    Parameters::parse(_parameters, Parameters::kEstimatorMinInliers(), minInliers_);
    Parameters::parse(_parameters, Parameters::kEstimatorPnPIterations(), pnpIterations_);
    Parameters::parse(_parameters, Parameters::kEstimatorPnPReprojError(), pnpReprojError_);
    Parameters::parse(_parameters, Parameters::kEstimatorPnPFlags(), pnpFlags_);
    Parameters::parse(_parameters, Parameters::kEstimatorRefineIterations(), refineIterations_);
    Parameters::parse(_parameters, Parameters::kEstimatorToleranceTranslation(), toleranceTranslation_);
    Parameters::parse(_parameters, Parameters::kEstimatorToleranceRotation(), toleranceRotation_);
    Parameters::parse(_parameters, Parameters::kEstimatorForce3DoF(), force3Dof_);

    optimizer_ = new Optimizer(_parameters);
    localMap_ = new LocalMap(_parameters);
}

Estimator::~Estimator() {
    delete optimizer_;
    delete localMap_;
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

void Estimator::outputOutliers(const std::set<std::size_t> & _outliers) {
    boost::lock_guard<boost::mutex> lock(mutexOutliersRw_);
    outliersBuf_ = _outliers;
}

std::set<std::size_t> Estimator::getOutliers() {
    boost::lock_guard<boost::mutex> lock(mutexOutliersRw_);
    return outliersBuf_;
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

            if (monitor_) {
                monitor_->addSignature(signature);
            }
            
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
    Eigen::Isometry3d currentGlobalPose(Eigen::Matrix4d::Zero());
    cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1);
    std::vector<std::size_t> matches;
    std::vector<std::size_t> inliers;
    std::map<std::size_t, cv::Point3f> words3dFrom = _signature.getCovisibleWords3d();
    std::map<std::size_t, cv::KeyPoint> wordsTo = _signature.getKeyPointsMatchesFormer();
    std::map<std::size_t, cv::Point3f> words3dTo = _signature.getWords3d();

    Eigen::Isometry3d wheelOdom;
    if (_signature.getWheelOdomPose(wheelOdom) && sensorStrategy_ == 2) {
        transform = previousWheelOdom_.inverse() * wheelOdom;
        matches = findCorrespondences(words3dFrom, wordsTo);
        inliers = matches;
        _signature.setPose(pose_ * transform);
        LOG_DEBUG << "DeltaWheelOdom pose: \n" << transform.matrix();
        LOG_DEBUG << "Set guess pose:\n" << _signature.getPose().matrix() << std::endl;
    } else {
        if (words3dFrom.size() >= minInliers_ && wordsTo.size() >= minInliers_) {
            transform = estimateMotion3DTo2D(words3dFrom, wordsTo, _signature.getCameraModel(), minInliers_, pnpIterations_,
                                                pnpReprojError_, pnpFlags_, refineIterations_, words3dTo, covariance, matches, inliers,
                                                _signature.getDeltaPoseGuess());
            _signature.setPose(pose_ * transform);
            if (sensorStrategy_ == 2)
                _signature.setWheelOdomPose(previousWheelOdom_ * transform);

        } else {
            transform = Eigen::Isometry3d(Eigen::Matrix4d::Zero());
            LOG_ERROR << "Not enough features in images, old=" << words3dFrom.size() << ", new=" << wordsTo.size() << ", min=" << minInliers_ << ".";
        }
        LOG_DEBUG << "Signature id : " << _signature.getId() << ", pose 3d->2d pnp is :\n" << transform.matrix();
    }

    if (transform.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
        LOG_ERROR << "Error: transform is Zero. The initial estimate failed.";
    } else {
        localMap_->insertSignature(_signature, transform.translation());
    }

    // If motion of guess > threshold, do bundle adjustment.
    std::map<std::size_t, Eigen::Isometry3d> optimizedPoses;
    std::map<std::size_t, std::tuple<Eigen::Vector3d, bool>> points3D;
    std::set<std::size_t> sbaOutliers;
    if (!transform.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero())) && inliers.size() > minInliers_ && localMap_->checkMapAvaliable()) {
        std::map<std::size_t, Eigen::Isometry3d> poses;
        std::map<std::size_t,std::tuple<std::size_t, std::size_t, Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> links;
        std::vector<boost::shared_ptr<GeometricCamera>> cameraModels;
        std::map<std::size_t, std::map<std::size_t, FeatureBA>> wordReferences;
        std::set<std::size_t> outliers;

        localMap_->getSignaturePoses(poses);
        if (sensorStrategy_ == 2)
            localMap_->getSignatureLinks(links);

        //camera model
        Eigen::Isometry3d transformRobotToImage = _signature.getCameraModel().getTansformImageToRobot().inverse();
        cameraModels.push_back(_signature.getCameraModelLeftPtr());
        cameraModels.push_back(_signature.getCameraModelRightPtr());

        localMap_->getFeaturePosesAndObservations(points3D, wordReferences);

        std::size_t rootId = poses.rbegin()->first - 1;
        optimizedPoses = optimizer_->localOptimize(rootId, poses, links, cameraModels, points3D, wordReferences, sbaOutliers);
        // std::cout << "sbaOutliers.size(): " << sbaOutliers.size() << std::endl;
        // for (auto iter = poses.begin(); iter != poses.end(); ++ iter) {
        //     std::cout << "poses id : " << iter->first << "  pose is :\n" << iter->second.matrix() << std::endl;
        // }
        // for (auto iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++ iter) {
        //     std::cout << "optimizedPoses id : " << iter->first << "  pose is :\n" << iter->second.matrix() << std::endl;
        // }

        // if (force3Dof_) {
        //     for (auto pose : optimizedPoses) {
        //         double x, y, z, roll, pitch, yaw;
        //         pcl::getTranslationAndEulerAngles(pose.second, x, y, z, roll, pitch, yaw);
        //         Eigen::Affine3d poseForce;
        //         pcl::getTransformation(x, y, 0, 0, 0, yaw, poseForce);
        //         pose.second = Eigen::Isometry3d(poseForce.matrix());             
        //     }
        // }

        // Update BA result
        if (optimizedPoses.size() == 6 && !optimizedPoses.begin()->second.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))
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
                LOG_ERROR << "Not enough inliers after bundle adjustment " << inliers.size() << "/" << minInliers_ << " between signature " << _signature.getId() - 1  << " and " << _signature.getId();
                transform = Eigen::Isometry3d(Eigen::Matrix4d::Zero());
            } else {
                auto iter = optimizedPoses.rbegin();
                currentGlobalPose = iter++->second;
                transform = iter->second.inverse() * currentGlobalPose;
                // std::cout << "currentGlobalPose: \n" << currentGlobalPose.matrix() << std::endl;
                // std::cout << "transform: \n" << transform.matrix() << std::endl;
            }

            // covariance = computeCovariance(words3dFrom, _signature.getWords3d(), transform, inliers);
            covariance = cv::Mat::eye(6, 6, CV_64FC1);
            // std::cout << "covariance: \n" << covariance << std::endl;

            // Update 3d points in signture.
            // TODO: cull ba outliers.
            // std::map<std::size_t, cv::Point3f> cpyWords3dTo = _signature.getWords3d();
            // Eigen::Isometry3d invT = currentGlobalPose.inverse();
            // for (auto iter = points3D.begin(); iter != points3D.end(); ++iter) {
            //     if (cpyWords3dTo.find(iter->first) != cpyWords3dTo.end())
            //         cpyWords3dTo.find(iter->first)->second = transformPoint(cv::Point3f(iter->second[0], iter->second[1], iter->second[2]), invT);
            // }
            // _signature.setWords3d(cpyWords3dTo);
                      
        } else {
            currentGlobalPose = pose_ * transform;
            LOG_ERROR << "BA failed, use default transform." << std::endl;
        }
    }

    if (_signature.getWheelOdomPose(wheelOdom)) {
        //TODO: calculate pnp to do some abnormal process work.
        // std::cout << "wheelOdom pose: \n" << wheelOdom.matrix() << std::endl;
        Eigen::Isometry3d deltaWheelOdom = previousWheelOdom_.inverse() * wheelOdom;
        // std::cout << "deltaWheelOdom pose: \n" << deltaWheelOdom.matrix() << std::endl;
        double wheelX, wheelY, wheelZ, wheelRoll, wheelPitch, wheelYaw, visualX, visualY, visualZ, visualRoll, visualPitch, visualYaw;
        pcl::getTranslationAndEulerAngles(deltaWheelOdom, wheelX, wheelY, wheelZ, wheelRoll, wheelPitch, wheelYaw);
        pcl::getTranslationAndEulerAngles(transform, visualX, visualY, visualZ, visualRoll, visualPitch, visualYaw);
        const double deltaX = wheelX - visualX;
        const double deltaY = wheelY - visualY;
        const double deltaZ = wheelZ - visualZ;
        const double deltaRoll = wheelRoll - visualRoll;
        const double deltaPitch = wheelPitch - visualPitch;
        const double deltaYaw = wheelYaw - visualYaw;
        
        // std::cout << "wheelX: " << wheelX << "   wheelY: " << wheelY << " wheel Yaw: " << wheelYaw <<  std::endl; 
        // std::cout << "wheel - visual, deltaX: " << deltaX << " deltaY:" << deltaY << " Tolerance:" << toleranceTranslation_ << std::endl; 
        if (wheelX != 0.0 && wheelY != 0.0) {
            if ((deltaX*deltaX + deltaY*deltaY) / (wheelX*wheelX + wheelY*wheelY) > toleranceTranslation_ ) {
                LOG_INFO << "Signatrue id : " << _signature.getId() << " has a large Translation. deltaX: " << deltaX << " deltaY: " << deltaY;
                transform = deltaWheelOdom;
                currentGlobalPose = pose_ * transform;
            }
        } else {
            transform = deltaWheelOdom;
            currentGlobalPose = pose_ * transform;
        }

        // if (abs(wheelYaw) > 0.0001) {
        //     if (abs(deltaYaw)/abs(wheelYaw) > toleranceRotation_) {
        //         std::cout << "wheel - visual, deltaYaw: " << deltaYaw << " Tolerance:" << toleranceRotation_ << std::endl;
        //         std::cout << "Signatrue id : " << _signature.getId() << " has a large rotation. deltaYaw: " << deltaYaw << std::endl;
        //     }            
        // }

        previousWheelOdom_ = wheelOdom;

        tempWheel = tempWheel + Eigen::Vector3d(wheelX, wheelY, wheelYaw);
        tempVisual = tempVisual + Eigen::Vector3d(visualX, visualY, visualYaw);
        // std::cout << "Signatrue id: " << _signature.getId() << "  wheel Odom: " << tempWheel.transpose() << std::endl;
        // std::cout << "Signatrue id: " << _signature.getId() << "  visual Odom: " << tempVisual.transpose() << std::endl;
    }

    if (force3Dof_) {
        double x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(currentGlobalPose, x, y, z, roll, pitch, yaw);
        Eigen::Affine3d pose;
        pcl::getTransformation(x, y, 0, 0, 0, yaw, pose);
        currentGlobalPose = Eigen::Isometry3d(pose.matrix());
        LOG_DEBUG << "CurrentGlobalPose After force 3d: \n" << currentGlobalPose.matrix() << std::endl;
    }

    if (optimizedPoses.size() == 6 && !optimizedPoses.begin()->second.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))
            && !optimizedPoses.rbegin()->second.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
        optimizedPoses.rbegin()->second = currentGlobalPose;
        localMap_->updateLocalMap(optimizedPoses, points3D, sbaOutliers);
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
        pose_ = currentGlobalPose;
        // std::cout << "pose_: \n" << pose_.matrix() << std::endl;
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
    // std::cout << "visual pose: \n" << _signature.getPose().matrix() << std::endl;
    localMap_->removeSignature();
    outputOutliers(sbaOutliers);

    std::map<std::size_t, cv::KeyPoint> blockWords;
    for (auto outlier : sbaOutliers) {
        if (wordsTo.find(outlier) != wordsTo.end()) {
            blockWords.emplace(outlier, wordsTo.at(outlier));
        }
    }
    _signature.setBlockedWords(blockWords);
    
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
        LOG_ERROR << "Error with value dt: " << _dt;
    }

    return Eigen::Isometry3d::Identity();
}

std::vector<std::size_t> Estimator::findCorrespondences(const std::map<std::size_t, cv::Point3f> & _words3dFrom, const std::map<std::size_t, cv::KeyPoint> & _words2dTo) {
    std::vector<std::size_t> matches;
    std::vector<size_t> ids = uKeys(_words2dTo);
    std::size_t oi = 0;
    
    matches.resize(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
        std::map<std::size_t, cv::Point3f>::const_iterator iter = _words3dFrom.find(ids[i]);
        if (iter != _words3dFrom.end() && isFinite(iter->second)) {
            matches[oi++] = ids[i];
        }
    }
    matches.resize(oi);

    return matches;
}

}   // namespace