#include "LocalMap.h"
#include "MultiviewGeometry.h"
#include "Stl.h"
#include "Timer.h"
#include "Log.h"

namespace VISFS {

LocalMap::LocalMap(const ParametersMap & _parameters) :
    keySignature_(true),
    localMapSize_(Parameters::defaultLocalMapMapSize()),
    maxFeature_(Parameters::defaultTrackerMaxFeatures()),
    minParallax_(Parameters::defaultLocalMapMinParallax()),
    minTranslation_(3*Parameters::defaultLocalMapMinTranslation()*Parameters::defaultLocalMapMinTranslation()),
    newFeatureCount_(0),
    signatureCount_(0),
    parallaxCount_(0.f),
    translationCount_(Eigen::Vector3d(0.0, 0.0, 0.0)),
    minInliers_(Parameters::defaultEstimatorMinInliers()) {

    Parameters::parse(_parameters, Parameters::kLocalMapMapSize(), localMapSize_);
    Parameters::parse(_parameters, Parameters::kTrackerMaxFeatures(), maxFeature_);
    Parameters::parse(_parameters, Parameters::kLocalMapMinParallax(), minParallax_);
    Parameters::parse(_parameters, Parameters::kLocalMapMinTranslation(), minTranslation_);
    minTranslation_ = 3 * minTranslation_ * minTranslation_;
    Parameters::parse(_parameters, Parameters::kEstimatorMinInliers(), minInliers_);
}

bool LocalMap::insertSignature(const Signature & _signature, const Eigen::Vector3d & _translation) {
    if (_signature.getWords3d().size() == 0 || _signature.empty()) {
        LOG_ERROR << "Error: LocalMap::insertSignature failed.";
        return false;
    }

    const std::map<std::size_t, cv::KeyPoint> & featuresInRight = _signature.getKeyPointsMatchesImageRight();
    const std::map<std::size_t, cv::KeyPoint> & featuresInNewSignatrue = _signature.getWords();
    const std::map<std::size_t, cv::KeyPoint> & featuresInFormer = _signature.getCovisibleWords();
    const std::map<std::size_t, cv::Point3f> & features3d = _signature.getWords3d();
    const Eigen::Isometry3d Twr = _signature.getPose();

    for (auto iter = featuresInNewSignatrue.begin(); iter != featuresInNewSignatrue.end(); iter++) {
        auto jter = features_.find(iter->first);
        if (jter == features_.end()) {   // Make sure outliers will not be treated as new feature.
            // add new feature
            if (features_.size() > maxFeature_) {
                if(iter->first <= features_.rbegin()->first)
                    continue;
            }
            if(features3d.find(iter->first) == features3d.end()) {
                continue;
            } else {
                if(!isFinite(features3d.at(iter->first))) {
                    continue;
                }
            }
            const cv::Point3f feature3d = features3d.at(iter->first);
            Feature newFeature(iter->first, _signature.getId(), Twr * Eigen::Vector3d(feature3d.x, feature3d.y, feature3d.z));
            newFeature.featureStatusInSigantures_.emplace(_signature.getId(), FeatureStatusInSiganature(iter->second.pt, featuresInRight.at(iter->first).pt, features3d.at(iter->first)));
            features_.emplace(iter->first, newFeature);
            ++newFeatureCount_;
        } else {
            // update feature
            jter->second.featureStatusInSigantures_.emplace(_signature.getId(), FeatureStatusInSiganature(iter->second.pt, featuresInRight.at(iter->first).pt, features3d.at(iter->first)));
            jter->second.setEndSignatureId(_signature.getId());
            if (jter->second.getObservedTimes() > (localMapSize_)) {
                if (jter->second.getFeatureState() == Feature::eFeatureState::NEW_ADDED) {
                    jter->second.setFeatureState(Feature::eFeatureState::STABLE);
                }
            }
        }
    }

    // add signature
    signatures_.emplace(_signature.getId(), _signature);

    // KeySignature check.
    keySignature_ = false;
    ++signatureCount_;
    translationCount_ += _translation.cwiseAbs();
    if (newFeatureCount_ > 0.2 * maxFeature_) {
        keySignature_ = true;
        clearCounters();
        // std::cout << "Feature condition satisfied!!!" << std::endl;
    } else if ((signatureCount_ > 10) 
            && ((translationCount_.x()*translationCount_.x() + translationCount_.y()*translationCount_.y() + translationCount_.z()*translationCount_.z()) > minTranslation_)) {
            keySignature_ = true;
            clearCounters();
            // std::cout << "Translation condition satisfied!!!" << std::endl;
    } else {
        // Compute parallax
        std::vector<std::size_t> matches =  findCorrespondences(featuresInFormer, featuresInNewSignatrue);
        float parallaxSum = 0.f;
        const int parallaxNum = static_cast<int>(matches.size());
        for (auto id : matches) {
            auto keyPointFrom = featuresInFormer.at(id);
            auto keyPointTo = featuresInNewSignatrue.at(id);
            const float du = keyPointFrom.pt.x - keyPointTo.pt.x;
            const float dv = keyPointFrom.pt.y - keyPointTo.pt.y;
            parallaxSum += std::max(0.f, sqrt(du*du + dv*dv));
        }
        parallaxCount_ += (parallaxSum / static_cast<float>(parallaxNum));
        if (parallaxCount_ >= minParallax_) {
            keySignature_ = true;
            clearCounters();
            // std::cout << "Keyframe condition satisfied!. parallax!, compute result= " << parallaxCount_ << ", minParallax_= " << minParallax_ << "." << std::endl; 
        }
    }

    LOG_DEBUG << "Signature: " << _signature.getId() << " is " << (keySignature_? "key" : "none key") << " signature.";

    return true;
}

void LocalMap::removeSignature() {
    if (signatures_.size() != localMapSize_ + 1) {
        if (signatures_.size() <= localMapSize_) {
            return;
        } else {
            LOG_FATAL << "Error: size of local Map is : " << signatures_.size() << " . Which is larger than " << localMapSize_ + 1;
            assert(signatures_.size() < localMapSize_ + 2);
        }
    } else {
        std::size_t rmId;
        if (keySignature_) {
            rmId = signatures_.begin()->first;
        } else {
            rmId = (++signatures_.rbegin())->first;
        }
        for (auto iter = features_.begin(); iter != features_.end();) {
            if (iter->second.featureStatusInSigantures_.find(rmId) != iter->second.featureStatusInSigantures_.end()) {
                iter->second.featureStatusInSigantures_.erase(rmId);
            }
            // Check size
            // std::cout << "Feature id :" << iter->first << ", observedTimes: " << iter->second.getObservedTimes() << std::endl;
            assert(iter->second.getObservedTimes() <= localMapSize_+1);
        
            if ((iter->second.getObservedTimes() == 0) 
                    && ((iter->second.getFeatureState() == Feature::eFeatureState::STABLE) || (iter->second.getEndSignatureId() < signatures_.begin()->first))) {
                iter = features_.erase(iter);
            } else {
                ++iter;
            }
        }        

        signatures_.erase(rmId);
    }

    LOG_DEBUG << "After remove, size of local Map is : " << signatures_.size() << " feature size: " << features_.size();
}

void LocalMap::updateLocalMap(const std::map<std::size_t, Eigen::Isometry3d> & _poses, const std::map<std::size_t, std::tuple<Eigen::Vector3d, bool>> & _point3d, const std::vector<std::tuple<std::size_t, std::size_t>> & _outliers, std::set<std::size_t> & _errorVertex) {
    for (auto pose : _poses) {
        if (!(signatures_.find(pose.first) == signatures_.end())) {
            signatures_.at(pose.first).setPose(pose.second);
        } else {
            LOG_FATAL << "[Error]: LocalMap, update signature pose unexist.";
        }
    }
    for (auto featurePose : _point3d) {
        if (!(features_.find(featurePose.first) == features_.end())) {
            Feature & feature = features_.at(featurePose.first);
            if (feature.getFeatureState() == Feature::eFeatureState::NEW_ADDED) {
                auto [pose, fixSymbol] = featurePose.second;
                feature.setFeaturePose(pose);
            }
        } else {
            LOG_FATAL << "[Error]: LocalMap, update feature pose unexist.";
        }
    }
    int i = 0;
    for (auto outlier : _outliers) {
        // if (!(features_.find(outlier) == features_.end())) {
        //     const Feature & feature = features_.at(outlier);
        //     if (feature.getObservedTimes() >= 2) {
        //         features_.erase(outlier);
        //         i++;
        //     } else {
        //         _outliers.erase(outlier);
        //     }
        // } else {
        //     LOG_ERROR << "[Error]: LocalMap, cull out unexist feature.";
        // }
        auto [featureId, signatureId] = outlier;
        auto iter = features_.find(featureId);
        if (iter != features_.end()) {
            auto jter = iter->second.featureStatusInSigantures_.find(signatureId);
            if (jter != iter->second.featureStatusInSigantures_.end()) {
                iter->second.featureStatusInSigantures_.erase(jter);
                // Discuss which vertex should be blocked.
                bool c1 = iter->second.getObservedTimes() == 0;
                bool c2 = iter->second.getFeatureState() == Feature::NEW_ADDED;
                auto secondSignature = signatures_.rbegin();
                secondSignature++;
                secondSignature++;
                bool c3 = iter->second.getStartSignatureId() < secondSignature->second.getId();
                if (c1 && c2 && c3) {
                    _errorVertex.emplace(featureId);
                    LOG_DEBUG << "Cull out feature: " << featureId << ". With condition c1: " << c1 << " c2: " << c2 << " c3: " << c3; 
                }
            } else {
                LOG_ERROR << "LocalMap: find feature but not find observation matches signature.";
            }
        } else {
            LOG_ERROR << "LocalMap: No such feature in localmap";
        }
    }
}

bool LocalMap::getSignaturePoses(std::map<std::size_t, Eigen::Isometry3d> & _poses) {
    for (auto iter = signatures_.begin(); iter != signatures_.end(); ++iter) {
        _poses.emplace(iter->first, iter->second.getPose());
    }
    // for (auto pose : _poses) {
    //     LOG_INFO << "signature id in _pose: " << pose.first << ", with pose:\n" << pose.second.matrix();
    // }
    return true;
}

bool LocalMap::getSignatureLinks(std::map<std::size_t,std::tuple<std::size_t, std::size_t, Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> & _links) {
    Eigen::Matrix<double, 6, 6> covariance, information;
    covariance.setZero();
    covariance(0, 0) = 0.0001;
    covariance(1, 1) = 0.0001;
    covariance(2, 2) = 0.0001;
    covariance(3, 3) = 0.0001;
    covariance(4, 4) = 0.0001;
    covariance(5, 5) = 0.0001;
    information = covariance.inverse();

    std::size_t i = 1;
    for (auto iter = signatures_.begin();;) {
        auto fromId = iter->first;
        auto fromPose = iter++->second.getWheelOdomPose();
        auto toId = iter->first;
        auto toPose = iter->second.getWheelOdomPose();

        if ((!fromPose.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) && (!toPose.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero())))) {
            auto transform = fromPose.inverse()*toPose;
            _links.emplace(i, std::make_tuple(fromId, toId, transform, information));
        }
        
        if (++i == signatures_.size())
            break;
    }

    // for (auto link : _links) {
    //     auto [fromId, toId, transfrom, information] = link.second;
    //     std::cout << "link id  : " << link.first << " from id : " << fromId << " to id : " << toId << std::endl;
    // }

    return true;
}

bool LocalMap::getFeaturePosesAndObservations(std::map<std::size_t, std::tuple<Eigen::Vector3d, bool>> & _points, std::map<std::size_t, std::map<std::size_t, FeatureBA>> & _observations) {
    const Eigen::Isometry3d transformRobotToImage = signatures_.begin()->second.getCameraModel().getTansformImageToRobot().inverse();
    for (auto feature : features_) {
        if (feature.second.getObservedTimes() > 1) {
            bool fixSymbol = feature.second.getFeatureState() == Feature::STABLE ? true : false;
            _points.emplace(feature.first, std::make_tuple(feature.second.getFeaturePose(), fixSymbol));

            std::map<std::size_t, FeatureBA> ptMap;
            for (auto observation : feature.second.featureStatusInSigantures_) {
                float depth = transformPoint(observation.second.point3d, transformRobotToImage).z;
                const cv::KeyPoint kpt = cv::KeyPoint(observation.second.uv, 1.f);
                ptMap.emplace(observation.first, FeatureBA(kpt, depth));
            }

            _observations.emplace(feature.first, ptMap);
        }
    }

    assert(_points.size() == _observations.size());
    return true;
}

bool LocalMap::checkMapAvaliable() {
    if ((signatures_.size() < 2) || features_.size() < minInliers_) {
        return false;
    }    

    return true;
}

std::vector<std::size_t> LocalMap::findCorrespondences(const std::map<std::size_t, cv::KeyPoint> & _wordsFrom, const std::map<std::size_t, cv::KeyPoint> & _wordsTo) {
    std::vector<std::size_t> matches;
    std::vector<size_t> ids = uKeys(_wordsTo);
    std::size_t oi = 0;
    
    matches.resize(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
        std::map<std::size_t, cv::KeyPoint>::const_iterator iter = _wordsFrom.find(ids[i]);
        if (iter != _wordsFrom.end()) {
            matches[oi++] = ids[i];
        }
    }
    matches.resize(oi);

    return matches;
}

inline void LocalMap::clearCounters() {
    // Print check
    // std::string time = getCurrentReadableTime();
    // std::cout << time << ", newFeatureCount_: " << newFeatureCount_ << " parallaxCount_: " << parallaxCount_ << " signatureCount_: " << signatureCount_ << " translationCount_: " << translationCount_.transpose() << std::endl;
    newFeatureCount_ = 0;
    signatureCount_ = 0;
    parallaxCount_ = 0.f;
    translationCount_.setZero();
}

inline bool LocalMap::checkCounters() {
    const bool c1 = newFeatureCount_ > 50;
    const bool c2 = parallaxCount_ > 50.f;
    const bool c3 = signatureCount_ > 10;
    const bool c4 = ((translationCount_.x()*translationCount_.x() + translationCount_.y()*translationCount_.y() + translationCount_.z()*translationCount_.z()) > 3*0.05*0.05);

    if (c1 || c2 || (c3 && c4))
        return true;
    
    return false;
}

}   // namespace