#include <iostream>
#include <assert.h>

#include "Tracker.h"
#include "MultiviewGeometry.h"
#include "Stl.h"
#include "Math.h"
#include "Timer.h"
#include "Log.h"

namespace VISFS {

Tracker::Tracker(const ParametersMap & _parameters) :
    maxFeature_(Parameters::defaultTrackerMaxFeatures()),
    qualityLevel_(Parameters::defaultTrackerQualityLevel()),
    minFeatureDistance_(Parameters::defaultTrackerMinDistance()),
    maxDepth_(Parameters::defaultTrackerMaxDepth()),
    minDepth_(Parameters::defaultTrackerMinDepth()),
    flowBack_(Parameters::defaultTrackerFlowBack()),
    flowWinSize_(Parameters::defaultTrackerFlowWinSize()),
    flowIterations_(Parameters::defaultTrackerFlowIterations()),
    flowEps_(Parameters::defaultTrackerFlowEps()),
    flowMaxLevel_(Parameters::defaultTrackerFlowMaxLevel()),
    cullByFundationMatrix_(Parameters::defaultTrackerCullByFundationMatrix()),
    fundationPixelError_(Parameters::defaultTrackerFundationPixelError()),
    minInliers_(Parameters::defaultEstimatorMinInliers()),
    trackingMethod_(TrackingMethod::STEREO),
    globalFeatureId_(0),
    estimator_(nullptr) {
    
    Parameters::parse(_parameters, Parameters::kTrackerMaxFeatures(), maxFeature_);
    Parameters::parse(_parameters, Parameters::kTrackerQualityLevel(), qualityLevel_);
    Parameters::parse(_parameters, Parameters::kTrackerMinDistance(), minFeatureDistance_);
    Parameters::parse(_parameters, Parameters::kTrackerMaxDepth(), maxDepth_);
    Parameters::parse(_parameters, Parameters::kTrackerMinDepth(), minDepth_);
    Parameters::parse(_parameters, Parameters::kTrackerFlowBack(), flowBack_);
    Parameters::parse(_parameters, Parameters::kTrackerFlowWinSize(), flowWinSize_);
    Parameters::parse(_parameters, Parameters::kTrackerFlowIterations(), flowIterations_);
    Parameters::parse(_parameters, Parameters::kTrackerFlowEps(), flowEps_);
    Parameters::parse(_parameters, Parameters::kTrackerFlowMaxLevel(), flowMaxLevel_);
    Parameters::parse(_parameters, Parameters::kTrackerCullByFundationMatrix(), cullByFundationMatrix_);
    Parameters::parse(_parameters, Parameters::kTrackerFundationPixelError(), fundationPixelError_);
    Parameters::parse(_parameters, Parameters::kEstimatorMinInliers(), minInliers_);
}

Tracker::~Tracker() {}

void Tracker::inputSignature(const Signature & _signature) {
    boost::lock_guard<boost::mutex> lock(mutexDataBuf_);
    signatureBuf_.emplace(_signature);
}

void Tracker::threadProcess() {
    while (1) {
        Signature signature;
        Eigen::Isometry3d guessPose;

        if (!signatureBuf_.empty()) {
            {
                boost::lock_guard<boost::mutex> lock(mutexDataBuf_);
                signature = signatureBuf_.front();
                signatureBuf_.pop(); 
            }
            // UTimer timer;
            std::set<std::size_t> outliers = estimator_->getOutliers();
            pretreatment(lastSignature_, outliers);
            process(lastSignature_, signature);
            // timer.elapsed("Tracker");
            
            if (estimator_) {
                estimator_->inputSignature(signature);
            } else {
                LOG_FATAL << "Estimator not set!";
            }

            lastSignature_ = signature;
        }

        boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(5));
    }
}

void Tracker::rejectOutlierWithFundationMatrix(const std::vector<cv::Point2f> & _cornersFrom, const std::vector<cv::Point2f> & _cornersTo, std::vector<unsigned char> & _status) const {
    assert(_cornersFrom.size() == _cornersTo.size());

	std::vector<unsigned char> statusFundationMatrix;
	cv::findFundamentalMat(_cornersFrom, _cornersTo, cv::FM_RANSAC, static_cast<double>(fundationPixelError_), 0.99, statusFundationMatrix);
	assert(_status.size() == statusFundationMatrix.size());
	for (std::size_t i = 0; i < _status.size(); ++i) {
		if (_status[i] && statusFundationMatrix[i]) {
			_status[i] = 1;
		} else {
			_status[i] = 0;
		}
	}	
}

void Tracker::updateTrackCounter(std::map<std::size_t, cv::KeyPoint> _wordIds) {
    for (auto iter = trackCnt_.begin(); iter != trackCnt_.end();) {
        auto jter = _wordIds.find(iter->first);
        if (jter != _wordIds.end()) {
            iter->second++;
            _wordIds.erase(jter);
            ++iter;
        } else {
            iter = trackCnt_.erase(iter);
        }
    }

    for (auto word : _wordIds) {
        trackCnt_.emplace(word.first, 1);
    }

}

cv::Mat Tracker::getMask(const std::map<std::size_t, cv::KeyPoint> & _kptTo,  const int _rows, const int _cols, const std::map<std::size_t, cv::KeyPoint> & _kptBlocked) const {
    std::vector<std::pair<std::size_t, std::pair<std::size_t, cv::Point2f>>> currentWords;

    for (auto kpt : _kptTo) {
        auto iter = trackCnt_.find(kpt.first);
        if (iter != trackCnt_.end()) {
            currentWords.emplace_back(std::make_pair(iter->second, std::make_pair(iter->first, kpt.second.pt)));
        }
    }

    std::sort(currentWords.begin(), currentWords.end(), [](const std::pair<std::size_t, std::pair<std::size_t, cv::Point2f>> & a, const std::pair<std::size_t, std::pair<std::size_t, cv::Point2f>> & b){
        return a.first > b.first;
    });

    cv::Mat mask = cv::Mat(_rows, _cols, CV_8UC1, cv::Scalar(255));
    for (auto word : currentWords) {
        if (mask.at<unsigned char>(word.second.second) == 255)
            cv::circle(mask, word.second.second, minFeatureDistance_, 0, -1);
    }
    for (auto word : _kptBlocked) {
        if (mask.at<unsigned char>(word.second.pt) == 255)
            cv::circle(mask, word.second.pt, minFeatureDistance_/2, 0, -1);        
    }

    return mask;
}

void Tracker::pretreatment(Signature & _fromSignature, const std::set<std::size_t> & _outliers) {
    if (_fromSignature.empty() || _outliers.empty()) {
        return;
    }

    std::map<std::size_t, cv::KeyPoint> blockedWords;
    auto words = _fromSignature.getWords();
    auto words3d = _fromSignature.getWords3d();
    if (!words.empty()) {
        for (auto outlier : _outliers) {
            if (words.find(outlier) != words.end()) {
                blockedWords.emplace(outlier, words.at(outlier));
                words.erase(outlier);
                words3d.erase(outlier);
            }

            if (trackCnt_.find(outlier) != trackCnt_.end())
                trackCnt_.erase(outlier);
        }
    }

    _fromSignature.setBlockedWords(blockedWords);
}

void Tracker::process(Signature & _fromSignature, Signature & _toSignature) {
    if (_fromSignature.empty() || _toSignature.empty()) {
        LOG_ERROR << "The from signature or the to signature is empty.";
        return;
    }

    std::vector<cv::KeyPoint> kptsFrom;
    std::vector<std::size_t> orignalWordsFromIds;
    std::vector<cv::Point2f> cornersFrom;
    cv::Mat imageFrom = _fromSignature.getImage();
    cv::Mat imageTo = _toSignature.getImage();

    if (_fromSignature.getWords().empty()) {
        std::map<std::size_t, cv::KeyPoint> words;
        cv::goodFeaturesToTrack(imageFrom, cornersFrom, maxFeature_, qualityLevel_, static_cast<double>(minFeatureDistance_));
        for (auto corner : cornersFrom) {
            cv::KeyPoint tempkpt(corner, 1.f);
            kptsFrom.emplace_back(tempkpt);
            words.insert(std::pair<std::size_t, cv::KeyPoint>(globalFeatureId_, tempkpt));
            ++globalFeatureId_;
        }
        orignalWordsFromIds = uKeys(words);
        _fromSignature.setWords(words);
    } else {    // Process the former extracted keypoints.
        kptsFrom = uValues(_fromSignature.getWords());
        orignalWordsFromIds = uKeys(_fromSignature.getWords());
    }

    // Generate the _fromSignature keypoints in 3d.
    std::vector<cv::Point3f> kptsFrom3D;
    if (kptsFrom.size() == _fromSignature.getWords3d().size()) {
        kptsFrom3D = uValues(_fromSignature.getWords3d());
    } else {
        LOG_WARN << "Generate the kptsFrom3D new calculate.  kptsFrom.size(): " << kptsFrom.size() << " ,_fromSignature.getWords3d().size(): " << _fromSignature.getWords3d().size();
        if (trackingMethod_ == STEREO) {
            // opticalFlow fromSignatrue right image to match corners.
            std::vector<unsigned char> status;
            std::vector<float> err;
            cv::Mat imageFromRight = _fromSignature.getImageRight();
            std::vector<cv::Point2f> cornersFromRight;
            cv::calcOpticalFlowPyrLK(imageFrom, imageFromRight, cornersFrom, cornersFromRight, status, err, cv::Size(flowWinSize_, flowWinSize_), flowMaxLevel_,
                                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, flowIterations_, flowEps_),
                                cv::OPTFLOW_LK_GET_MIN_EIGENVALS | 0, 1e-4);
            std::vector<cv::KeyPoint> kptsFromRight;
            for (auto corner : cornersFromRight) {
                cv::KeyPoint tempkptsRight(corner, 1.f);
                kptsFromRight.emplace_back(tempkptsRight);
            }
            // _fromSignature.setKeyPointMatchesImageRight(kptsFromRight);
            // _fromSignature.setLeftRightPairStatus(status);
            // generate3DPoints
            LOG_INFO << "From signature, cornersFrom size: " << kptsFrom.size() << " , cornersFromRight: " << kptsFromRight.size() << " , status: " << status.size();
            kptsFrom3D = generateKeyPoints3DStereo(kptsFrom, kptsFromRight, _fromSignature.getCameraModelLeft(), _fromSignature.getCameraModelRight(), minDepth_, maxDepth_);
            assert(orignalWordsFromIds.size() == kptsFrom3D.size());
            std::map<std::size_t, cv::Point3f> words3d;
            for (std::size_t i = 0; i < orignalWordsFromIds.size(); ++i) {
                words3d.insert(std::pair<std::size_t, cv::Point3f>(orignalWordsFromIds[i], kptsFrom3D[i]));
            }
            _fromSignature.setWords3d(words3d);

        } else if (trackingMethod_ == RGBD) {
            LOG_FATAL << "TODO";
        }
    }
    assert(kptsFrom.size() == kptsFrom3D.size());

    // Do a initial estimate of the _toSignature key points' pixel position.
    cv::KeyPoint::convert(kptsFrom, cornersFrom);
    std::vector<cv::Point2f> cornersTo;
    // bool guessSet = !(_guess.isApprox(Eigen::Isometry3d::Identity()));
    bool guessSet = !_toSignature.getDeltaPoseGuess().isApprox(Eigen::Isometry3d::Identity());
    if (guessSet && !kptsFrom3D.empty()) {
        Eigen::Isometry3d Tri = _fromSignature.getCameraModel().getTansformImageToRobot();
        Eigen::Isometry3d guessCameraRef = (_toSignature.getDeltaPoseGuess() * Tri).inverse();
        Eigen::Matrix3d eigenR = guessCameraRef.rotation();
		cv::Mat R = (cv::Mat_<double>(3,3) <<
				guessCameraRef(0, 0), guessCameraRef(0, 1), guessCameraRef(0, 2),
				guessCameraRef(1, 0), guessCameraRef(1, 1), guessCameraRef(1, 2),
				guessCameraRef(2, 0), guessCameraRef(2, 1), guessCameraRef(2, 2));
        cv::Mat	rvec(1, 3, CV_64FC1);
		cv::Rodrigues(R, rvec);
        Eigen::Vector3d eigent = guessCameraRef.translation();
        cv::Mat tvec = (cv::Mat_<double>(1,3) << eigent.x(), eigent.y(), eigent.z());
        cv::Mat K = _fromSignature.getCameraModel().cvKdouble();
        cv::projectPoints(kptsFrom3D, rvec, tvec, K, cv::Mat(), cornersTo);
    }

    // Find features in the new left image.
    std::vector<unsigned char> status;
    std::vector<float> err;
	cv::calcOpticalFlowPyrLK(imageFrom, imageTo, cornersFrom, cornersTo, status, err, cv::Size(flowWinSize_, flowWinSize_), flowMaxLevel_,
								cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, flowIterations_, flowEps_),
								cv::OPTFLOW_LK_GET_MIN_EIGENVALS | (guessSet?cv::OPTFLOW_USE_INITIAL_FLOW:0), 1e-4);
    if (flowBack_) {
        std::vector<unsigned char> reverseStatus;
        std::vector<cv::Point2f> cornersReverse = cornersFrom;
		cv::calcOpticalFlowPyrLK(imageTo, imageFrom, cornersTo, cornersReverse, reverseStatus, err, cv::Size(flowWinSize_, flowWinSize_), flowMaxLevel_,
								cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, flowIterations_, flowEps_),
                                cv::OPTFLOW_LK_GET_MIN_EIGENVALS | cv::OPTFLOW_USE_INITIAL_FLOW, 1e-4);
        for (std::size_t i = 0; i < status.size(); ++i) {
            // std::cout << "status[i]: " << (int)status[i] << "  reverseStatus[i]: " << (int)reverseStatus[i] << " distance: " << L2Norm<float, cv::Point2f>(cornersReverse[i], cornersFrom[i]) << std::endl;
            if (status[i] && reverseStatus[i] && L2Norm<float, cv::Point2f>(cornersReverse[i], cornersFrom[i]) <= 1.5) {
                status[i] = 1;
            } else {
                status[i] = 0;
            }
        }
    }
    if(!flowBack_ && cullByFundationMatrix_) {
        rejectOutlierWithFundationMatrix(cornersFrom, cornersTo, status);
    }

    // Reduce feature vector
    std::vector<cv::KeyPoint> kptsTo(kptsFrom.size());
    std::vector<cv::Point2f> cornersToKept(cornersTo.size());
    std::vector<cv::Point3f> kptsFrom3DKept(kptsFrom3D.size());
    std::vector<std::size_t> orignalWordsFromIdsCpy = orignalWordsFromIds;
    std::size_t index = 0;
    for (std::size_t i = 0; i < status.size(); ++i) {
        if (status[i] && uIsInBounds(cornersTo[i].x, 0.f, static_cast<float>(imageTo.cols)) && uIsInBounds(cornersTo[i].y, 0.f, static_cast<float>(imageTo.rows))) {
            if (orignalWordsFromIdsCpy.size()) {
                orignalWordsFromIds[index] = orignalWordsFromIdsCpy[i];
            }
            kptsFrom[index] = cv::KeyPoint(cornersFrom[i], 1);
            cornersToKept[index] = cornersTo[i];
            kptsFrom3DKept[index] = kptsFrom3D[i];
            kptsTo[index++] = cv::KeyPoint(cornersTo[i], 1);
        }
    }
    if (orignalWordsFromIds.size())
        orignalWordsFromIds.resize(index);
    kptsFrom.resize(index);
    kptsTo.resize(index);
    kptsFrom3DKept.resize(index);
    cornersToKept.resize(index);

    if (kptsTo.size() < minInliers_) {
        LOG_ERROR << "After Reduce feature vector, kptsTo.size(): " << kptsTo.size();
        LOG_ERROR << "Lost tracking !!!!!!!!!!!!!!!!!!!!!!!!";
        return;
    }
    
    std::map<std::size_t, cv::KeyPoint> covisibleWords;
    std::map<std::size_t, cv::Point3f> covisibleWords3d;
    std::map<std::size_t, cv::KeyPoint> wordsTo;
    for (std::size_t i = 0; i < orignalWordsFromIds.size(); ++i) {
        std::size_t id = orignalWordsFromIds[i];
        covisibleWords.insert(std::pair<std::size_t, cv::KeyPoint>(id, kptsFrom[i]));
        covisibleWords3d.insert(std::pair<std::size_t, cv::Point3f>(id, kptsFrom3DKept[i]));
        wordsTo.insert(std::pair<std::size_t, cv::KeyPoint>(id, kptsTo[i]));
    }
    _toSignature.setCovisibleWords(covisibleWords);
    _toSignature.setCovisibleWords3d(covisibleWords3d);
    _toSignature.setkeyPointsMatchesFormer(wordsTo);

    // Make up new corners.
    std::vector<cv::Point2f> newCornersInTo;
    int backUpCornersCnt = maxFeature_ - static_cast<int>(kptsTo.size());
    if (backUpCornersCnt > 0 && !_toSignature.getImage().empty()) {
        cv::Mat mask = getMask(wordsTo, imageTo.rows, imageTo.cols, _fromSignature.getBlockedWords());
        cv::goodFeaturesToTrack(imageTo, newCornersInTo, backUpCornersCnt, qualityLevel_, minFeatureDistance_, mask);
        std::map<std::size_t, cv::KeyPoint> wordsToNewExtract;
        for (auto corner : newCornersInTo) {
            cv::KeyPoint tempKpts(corner, 1.f);
            wordsToNewExtract.insert(std::pair<std::size_t, cv::KeyPoint>(globalFeatureId_, tempKpts));
            wordsTo.insert(std::pair<std::size_t, cv::KeyPoint>(globalFeatureId_, tempKpts));
            ++globalFeatureId_;
        }
        _toSignature.setKeyPointsNewExtract(wordsToNewExtract);
        // cv::namedWindow("mask");
        // cv::imshow("mask", mask);
        // cv::waitKey(5);
    }

    // Stereo
    std::map<std::size_t, cv::KeyPoint> wordsToRight;
    std::vector<std::size_t> wordsToIds;
    if (trackingMethod_ == STEREO && !_toSignature.getImageRight().empty() && wordsTo.size() > 0) {
        wordsToIds = uKeys(wordsTo);
        std::vector<cv::Point2f> allCornersInLeft;
        cv::KeyPoint::convert(uValues(wordsTo), allCornersInLeft);
        status.clear();
        err.clear();
        cv::Mat imageToRight = _toSignature.getImageRight();
        std::vector<cv::Point2f> cornersToRight;
        cv::calcOpticalFlowPyrLK(imageTo, imageToRight, allCornersInLeft, cornersToRight, status, err, cv::Size(flowWinSize_, flowWinSize_), flowMaxLevel_,
							cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, flowIterations_, flowEps_),
							cv::OPTFLOW_LK_GET_MIN_EIGENVALS | 0, 1e-4);
        if (flowBack_) {
            std::vector<unsigned char> reverseStatus;
            std::vector<cv::Point2f> cornersReverse = allCornersInLeft;
		    cv::calcOpticalFlowPyrLK(imageToRight, imageTo, cornersToRight, cornersReverse, reverseStatus, err, cv::Size(flowWinSize_, flowWinSize_), flowMaxLevel_,
								cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, flowIterations_, flowEps_),
                                cv::OPTFLOW_LK_GET_MIN_EIGENVALS | cv::OPTFLOW_USE_INITIAL_FLOW, 1e-4);
            for (std::size_t i = 0; i < status.size(); ++i) {
                if (status[i] && reverseStatus[i] && L2Norm<float, cv::Point2f>(cornersReverse[i], allCornersInLeft[i]) <= 0.5) {
                    status[i] = 1;
                } else {
                    // std::cout << "Cull out stereo: " << "status: " << (int)status[i] << "  reverseStatus" << (int)reverseStatus[i] << " distance: " << L2Norm<float, cv::Point2f>(cornersReverse[i], allCornersInLeft[i]) << std::endl;
                    status[i] = 0;
                }
            }            
        }

        assert(cornersToRight.size() == wordsToIds.size());

        for (std::size_t i = 0; i < status.size(); ++i) {
            if (status[i] && uIsInBounds(cornersToRight[i].x, 0.f, static_cast<float>(imageTo.cols)) && uIsInBounds(cornersToRight[i].y, 0.f, static_cast<float>(imageTo.rows))) {
                wordsToRight.insert(std::pair<std::size_t, cv::KeyPoint>(wordsToIds[i], cv::KeyPoint(cornersToRight[i], 1.f)));
            } else {
                wordsTo.erase(wordsToIds[i]);
            }
        }
    }

    std::vector<cv::Point3f> kptsTo3D;
    std::map<std::size_t, cv::Point3f> wordsTo3D;
    wordsToIds = uKeys(wordsTo);
    if (trackingMethod_ == STEREO) {
        kptsTo3D = generateKeyPoints3DStereo(uValues(wordsTo), uValues(wordsToRight), _toSignature.getCameraModelLeft(), _toSignature.getCameraModelRight(), minDepth_, maxDepth_);
        for (std::size_t i = 0; i < wordsToIds.size(); ++i) {
            if (isFinite(kptsTo3D[i])) {
                wordsTo3D.insert(std::pair<std::size_t, cv::Point3f>(wordsToIds[i], kptsTo3D[i]));
            } else {
                wordsTo.erase(wordsToIds[i]);
                wordsToRight.erase(wordsToIds[i]);
            }
            
        }        
    } else if (trackingMethod_ == RGBD) {

    }

    assert(wordsTo.size() == wordsToRight.size());
    assert(wordsTo.size() == wordsTo3D.size());

    LOG_DEBUG << "After tracker pocess, valiable words count: " << wordsTo.size();

    _toSignature.setKeyPointMatchesImageRight(wordsToRight);
    _toSignature.setWords(wordsTo);
    _toSignature.setWords3d(wordsTo3D);
    updateTrackCounter(wordsTo);

}

}   // namespace