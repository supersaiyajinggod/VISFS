#include <assert.h>
#include <limits>

#include <Eigen/Core>

#include "MultiviewGeometry.h"
#include "Math.h"
#include "Stl.h"
#include "Conversion.h"
#include "Log.h"

namespace VISFS {

bool isFinite(const cv::Point3f & _point) {
    return uIsFinite(_point.x) && uIsFinite(_point.y) && uIsFinite(_point.z);
}

cv::Point3f transformPoint(const cv::Point3f & _points, const GeometricCamera & _model) {
    Eigen::Vector4d tempPoint = _model.getTansformImageToRobot() * Eigen::Vector4d(_points.x, _points.y, _points.z, 1.f);
    return cv::Point3f(tempPoint[0], tempPoint[1], tempPoint[2]);
}

cv::Point3f transformPoint(const cv::Point3f & _points, const Eigen::Isometry3d & _transform) {
    Eigen::Vector4d tempPoint = _transform * Eigen::Vector4d(_points.x, _points.y, _points.z, 1.f);
    return cv::Point3f(tempPoint[0], tempPoint[1], tempPoint[2]);
}

std::vector<float> computeReprojErrors(
	const std::vector<cv::Point3f> & _objectPoints,
	const std::vector<cv::Point2f> & _imagePoints,
	const cv::Mat & _cameraModel,
	const cv::Mat & _distCoeffs,
	const cv::Mat & _rvec,
	const cv::Mat & _tvec,
	const float _reProjErrorThreshold,
	std::vector<std::size_t> & _inliers) {
    assert(_objectPoints.size() == _imagePoints.size());
    std::size_t cnt = _objectPoints.size();
    std::vector<cv::Point2f> projPoints;
    cv::projectPoints(_objectPoints, _rvec, _tvec, _cameraModel, _distCoeffs, projPoints);

    _inliers.resize(cnt, 0);
    std::vector<float> error(cnt);
    std::size_t oi = 0;
    for (std::size_t i = 0; i < cnt; ++i) {
        float e = static_cast<float>(cv::norm(_imagePoints[i] - projPoints[i]));
        if (e <= _reProjErrorThreshold) {
            _inliers[oi] = i;
            error[oi++] = e;
        }
    }
    _inliers.resize(oi);
    error.resize(oi);
    return error;
}

std::vector<cv::Point3f> generateKeyPoints3DStereo(const std::vector<cv::KeyPoint> & _kptsLeft, const std::vector<cv::KeyPoint> & _kptsRight_, const GeometricCamera & _cameraLeft, const GeometricCamera & _cameraRight, float _minDepth, float _maxDepth) {
    assert(_kptsLeft.size() == _kptsRight_.size());

    std::vector<cv::Point3f> kpts3D;
    kpts3D.resize(_kptsLeft.size());
    const float badPoint = std::numeric_limits<float>::quiet_NaN();
    for (std::size_t i = 0; i < _kptsLeft.size(); ++i) {
        cv::Point3f point(badPoint, badPoint, badPoint);
        float disparity = _kptsLeft[i].pt.x - _kptsRight_[i].pt.x;
        if (disparity != 0.f) {
            cv::Point3f tempPoint = projectDisparityTo3D(_kptsLeft[i].pt, disparity, _cameraLeft, _cameraRight);
            if (isFinite(tempPoint) && (_minDepth < 0.f || tempPoint.z > _minDepth) && (_maxDepth <= 0.f || tempPoint.z <= _maxDepth)) {
                point = tempPoint;  // point in camera coordinate system.
                point = transformPoint(point, _cameraLeft);
            }
        }
        kpts3D.at(i) = point;
    }
    return kpts3D;
}

cv::Point3f projectDisparityTo3D(const cv::Point2f & _corner, float _disparity, const GeometricCamera & _cameraLeft, const GeometricCamera & _cameraRight) {
    cv::Mat leftK = _cameraLeft.cvKfloat();
    cv::Mat rightK = _cameraRight.cvKfloat();
    float baseLine = _cameraLeft.getBaseLine();
    if (_disparity > 0.f && baseLine > 0.f && leftK.at<float>(0, 0) > 0.f) {
        float c = 0.f;
        if (leftK.at<float>(0, 2) > 0.f && rightK.at<float>(0, 2) > 0.f) {
            c = rightK.at<float>(0, 2) - leftK.at<float>(0, 2);
        }
        float W = baseLine/(_disparity + c);
        return cv::Point3f((_corner.x - leftK.at<float>(0, 2))*W, (_corner.y - leftK.at<float>(1, 2))*W, leftK.at<float>(0, 0)*W);
    }
    const float badPoint = std::numeric_limits<float>::quiet_NaN();
    return cv::Point3f(badPoint, badPoint, badPoint);
}

Eigen::Isometry3d estimateMotion3DTo2D(
    const std::map<std::size_t, cv::Point3f> & _words3dFrom,
    const std::map<std::size_t, cv::KeyPoint> & _words2dTo,
    const GeometricCamera & _cameraModel,
    int _minInliers,
    int _iterations,
    double _reProjError,
    int _flagPnP,
    int _refineIterations,
    const std::map<std::size_t, cv::Point3f> & _words3dTo,
    cv::Mat & _covariance,
    std::vector<std::size_t> & _matchesOut,
    std::vector<std::size_t> & _inliersOut,
	const Eigen::Isometry3d & _guess) {
    Eigen::Isometry3d transform(Eigen::Matrix4d::Zero());
    std::vector<std::size_t> matches, inliers;
    _covariance = cv::Mat::eye(6, 6, CV_64FC1);

    // find correspondences
    std::vector<size_t> ids = uKeys(_words2dTo);
    std::vector<cv::Point3f> objectPoints(ids.size());
    std::vector<cv::Point2f> imagePoints(ids.size());
    std::size_t oi = 0;
    matches.resize(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
        std::map<std::size_t, cv::Point3f>::const_iterator iter = _words3dFrom.find(ids[i]);
        if (iter != _words3dFrom.end() && isFinite(iter->second)) {
            const cv::Point3f & pt = iter->second;
            objectPoints[oi] = pt;
            imagePoints[oi] = _words2dTo.find(ids[i])->second.pt;
            matches[oi++] = ids[i];
        }
    }
    objectPoints.resize(oi);
    imagePoints.resize(oi);
    matches.resize(oi);

    if (static_cast<int>(matches.size()) >= _minInliers) {
        cv::Mat K = _cameraModel.cvKdouble();
        cv::Mat D = _cameraModel.cvDdouble();
        Eigen::Isometry3d guessCameraFrame = (_guess * _cameraModel.getTansformImageToRobot()).inverse();
		cv::Mat R = (cv::Mat_<double>(3,3) <<
				guessCameraFrame(0, 0), guessCameraFrame(0, 1), guessCameraFrame(0, 2),
				guessCameraFrame(1, 0), guessCameraFrame(1, 1), guessCameraFrame(1, 2),
				guessCameraFrame(2, 0), guessCameraFrame(2, 1), guessCameraFrame(2, 2));
		cv::Mat rvec(1,3, CV_64FC1);
		cv::Rodrigues(R, rvec);
        Eigen::Vector3d eigent = guessCameraFrame.translation();
        cv::Mat tvec = (cv::Mat_<double>(1,3) << eigent.x(), eigent.y(), eigent.z());

        // Calculate
        VISFS::solvePnPRansac(objectPoints, imagePoints, K, D, rvec, tvec, false, _iterations, _reProjError, _minInliers, inliers, _flagPnP, _refineIterations);

        if (static_cast<int>(inliers.size()) >= _minInliers) {
            cv::Rodrigues(rvec, R);
            Eigen::Matrix3d rotation;
            rotation << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
						R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
						R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
            Eigen::Vector3d translate(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
            Eigen::Isometry3d pnp = Eigen::Isometry3d::Identity();
            pnp.prerotate(rotation);
            pnp.pretranslate(translate);
            transform = (_cameraModel.getTansformImageToRobot() * pnp).inverse();

            // compute variance (like in PCL computeVariance() method of sac_model.h)
            if (_words3dTo.size()) {
                std::vector<float> errSqrdDists(inliers.size());
                std::vector<float> errSqrdAngles(inliers.size());
                oi = 0;
                for (std::size_t i = 0; i < inliers.size(); ++i) {
                    std::map<std::size_t, cv::Point3f>::const_iterator iter = _words3dTo.find(matches[inliers[i]]);
                    if (iter != _words3dTo.end() && isFinite(iter->second)) {
                        const cv::Point3f & objPoint = objectPoints[inliers[i]];
                        cv::Point3f newPoint = transformPoint(iter->second, transform);
                        errSqrdDists[oi] = uNormSquared(objPoint.x-newPoint.x, objPoint.y-newPoint.y, objPoint.z-newPoint.z);

                        Eigen::Vector4f v1(objPoint.x-transform.translation().x(), objPoint.y-transform.translation().y(), objPoint.z-transform.translation().z(), 0);
                        Eigen::Vector4f v2(newPoint.x-transform.translation().x(), newPoint.y-transform.translation().y(), newPoint.z-transform.translation().z(), 0);
                        errSqrdAngles[oi++] = getAngle3D(v1, v2);
                    }
                }
                errSqrdDists.resize(oi);
                errSqrdAngles.resize(oi);

                if (errSqrdDists.size()) {
                    std::sort(errSqrdDists.begin(), errSqrdDists.end());
                    double medianErrSqr = 2.1981 * static_cast<double>(errSqrdDists[errSqrdDists.size() >> 1]);
                    assert(uIsFinite(medianErrSqr));
                    _covariance(cv::Range(0, 3), cv::Range(0, 3)) *= medianErrSqr;
                    std::sort(errSqrdAngles.begin(), errSqrdAngles.end());
                    medianErrSqr = 2.1981 * static_cast<double>(errSqrdAngles[errSqrdAngles.size() >> 1]);
                    assert(uIsFinite(medianErrSqr));
                    _covariance(cv::Range(3, 6), cv::Range(3, 6)) *= medianErrSqr;
                } else {
                    LOG_ERROR << "Not enough close points to compute covariance!";
                }

                if (static_cast<float>(oi) / static_cast<float>(inliers.size()) < 0.2f) {
                    LOG_WARN << "A very low number of inliers have valid depth " << oi << " / " << inliers.size() << " , the transform returned may be wrong!";
                }
            } else {
                // compute variance, which is the rms of reprojection errors
                std::vector<cv::Point2f> imagePointsReproj;
                cv::projectPoints(objectPoints, rvec, tvec, K, cv::Mat(), imagePointsReproj);
                float err = 0.f;
                for (std::size_t i = 0; i < inliers.size(); ++i) {
                    err += uNormSquared(imagePoints.at(inliers[i]).x-imagePointsReproj.at(inliers[i]).x, imagePoints.at(inliers[i]).y-imagePointsReproj.at(inliers[i]).y);
                }
                assert(uIsFinite(err));
                _covariance *= std::sqrt(err/static_cast<float>(inliers.size()));
            }
        }
    }

    _matchesOut = matches;
    _inliersOut.resize(inliers.size());
    for (std::size_t i = 0; i < inliers.size(); ++i) {
        _inliersOut.at(i) = matches[inliers[i]];
    }

    return transform;
}


void solvePnPRansac(
	const std::vector<cv::Point3f> & _objectPoints,
	const std::vector<cv::Point2f> & _imagePoints,
	const cv::Mat & _cameraMatrix,
	const cv::Mat & _distCoeffs,
	cv::Mat & _rvec,
	cv::Mat & _tvec,
	bool _useExtrinsicGuess,
	int _iterationsCount,
	float _reprojectionError,
	int _minInliersCount,
	std::vector<std::size_t> & _inliers,
	int _flags,
	int _refineIterations,
	float _refineSigma) {
    if (_minInliersCount < 4) {
        _minInliersCount = 4;
    }
    std::vector<int> inliersInt;
    cv::solvePnPRansac(_objectPoints, _imagePoints, _cameraMatrix, _distCoeffs, _rvec, _tvec,
                        _useExtrinsicGuess, _iterationsCount, _reprojectionError, 0.99, inliersInt, _flags);
    float inlierThreshold = _reprojectionError;
    if (static_cast<int>(inliersInt.size()) >= _minInliersCount && _refineIterations > 0) {
        float errorThreshold = inlierThreshold;
        int refineIterations = 0;
        bool inlierChanged = false, oscillating = false;
        _inliers = uIntVector2Ul(inliersInt);
        std::vector<std::size_t> newInliers, prevInliers = _inliers, inliersSizes;
        cv::Mat newModelRvec = _rvec;
        cv::Mat newModelTvec = _tvec;

        do {
            // Get inliers from current model.
            std::vector<cv::Point3f> oPointsInliers(prevInliers.size());
            std::vector<cv::Point2f> iPointsInliers(prevInliers.size());
            for (std::size_t i = 0; i < prevInliers.size(); ++i) {
                oPointsInliers[i] = _objectPoints[prevInliers[i]];
                iPointsInliers[i] = _imagePoints[prevInliers[i]];
            }
            // Optimize the model coefficients.
            cv::solvePnP(oPointsInliers, iPointsInliers, _cameraMatrix, _distCoeffs, newModelRvec, newModelTvec, true, _flags);
            inliersSizes.push_back(prevInliers.size());

            // Select the new inliers based on the optimized coefficients and new threshold.
            std::vector<float> error = computeReprojErrors(_objectPoints, _imagePoints, _cameraMatrix, _distCoeffs, newModelRvec, newModelTvec, errorThreshold, newInliers);
            if (static_cast<int>(newInliers.size()) < _minInliersCount) {
                ++refineIterations;
                if (refineIterations >= _refineIterations) {
                    break;
                }
                continue;
            }

            // Estimate the variance and the new threshold.
            float mean = uMean(error);
            float variance = uVariance(error, mean);
            errorThreshold = std::min(inlierThreshold, _refineSigma*sqrt(variance));

            inlierChanged = false;
            std::swap(prevInliers, newInliers);
            // If the number of inliers changed, then we are still optimizing.
            if (newInliers.size() != prevInliers.size()) {
                // Check if the number of inliers is oscillating in between two values
                if (static_cast<int>(inliersSizes.size()) >= _minInliersCount) {
                    if (inliersSizes[inliersSizes.size()-1] == inliersSizes[inliersSizes.size()-3] &&
                        inliersSizes[inliersSizes.size()-2] == inliersSizes[inliersSizes.size()-4]) {
                        oscillating = true;
                        break;
                    }
                }
                inlierChanged = true;
                continue;
            }
            // Check the value of the inlier set.
            for (std::size_t i = 0; i < prevInliers.size(); ++i) {
                // If the value of the inliers changed, then we are still optimizing
                if (prevInliers[i] != newInliers[i]) {
                    inlierChanged = true;
                    break;
                }
            }
        } while (inlierChanged && ++refineIterations < _refineIterations);

        // If the new set of inliers is empty, we didn't do a good job refineing.
        if (static_cast<int>(prevInliers.size()) < _minInliersCount) {
            LOG_ERROR << "RANSAC refineModel: Refinement failed: got very low inliers " << prevInliers.size() << ".";
        }
        if (oscillating) {
            LOG_WARN << "RANSAC refineModel: Detected oscillations in the model refinement.";
        }

        std::swap(_inliers, newInliers);
        _rvec = newModelRvec;
        _tvec = newModelTvec;
    }

}

cv::Mat computeCovariance(
	const std::map<std::size_t, cv::Point3f> & _pointsInCoor1,
	const std::map<std::size_t, cv::Point3f> & _pointsInCoor2,
	const Eigen::Isometry3d & _transformCoor2ToCoor1,
	const std::vector<std::size_t> & _inliers) {
        cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1);
    if (_inliers.empty()) {
        LOG_ERROR << "The corresponding index is empty. return huge covariance.";
        covariance *= 9999.0;
        return covariance;
    } else if (_pointsInCoor1.empty() || _pointsInCoor2.empty()) {
        LOG_ERROR << "The input points are empty. return huge covariance.";
        covariance *= 9999.0;
        return covariance;
    } else {
        std::vector<float> errSqrdDists(_inliers.size());
        std::vector<float> errSqrdAngles(_inliers.size());
        std::size_t oi = 0;
        for (std::size_t i = 0; i < _inliers.size(); ++i) {
            std::map<std::size_t, cv::Point3f>::const_iterator iter1 = _pointsInCoor1.find(_inliers[i]);
            std::map<std::size_t, cv::Point3f>::const_iterator iter2 = _pointsInCoor2.find(_inliers[i]);
            if (iter1 != _pointsInCoor1.end() && isFinite(iter1->second) && iter2 != _pointsInCoor2.end() && isFinite(iter2->second)) {
                const cv::Point3f & ptCoor1 = iter1->second;
                const cv::Point3f & ptCoor2Proj = transformPoint(iter2->second, _transformCoor2ToCoor1);
                errSqrdDists[oi] = uNormSquared(ptCoor1.x - ptCoor2Proj.x, ptCoor1.y - ptCoor2Proj.y, ptCoor1.z- ptCoor2Proj.z);

                auto translationCoor2TtoCoor1 = _transformCoor2ToCoor1.translation();
                Eigen::Vector4f v1(ptCoor1.x- translationCoor2TtoCoor1.x(), ptCoor1.y - translationCoor2TtoCoor1.y(), ptCoor1.z - translationCoor2TtoCoor1.z(), 0);
                Eigen::Vector4f v2(ptCoor2Proj.x - translationCoor2TtoCoor1.x(), ptCoor2Proj.y - translationCoor2TtoCoor1.y(), ptCoor2Proj.z - translationCoor2TtoCoor1.z(), 0);
                errSqrdAngles[oi++] = getAngle3D(v1, v2);
            }
        }
        errSqrdDists.resize(oi);
        errSqrdAngles.resize(oi);

        if (errSqrdDists.size()) {
            std::sort(errSqrdDists.begin(), errSqrdDists.end());
            double medianErrSqr = 2.1981 * static_cast<double>(errSqrdDists[errSqrdDists.size() >> 1]);
            assert(uIsFinite(medianErrSqr));
            covariance(cv::Range(0, 3), cv::Range(0, 3)) *= medianErrSqr;
            std::sort(errSqrdAngles.begin(), errSqrdAngles.end());
            medianErrSqr = 2.1981 * static_cast<double>(errSqrdAngles[errSqrdAngles.size() >> 1]);
            assert(uIsFinite(medianErrSqr));
            covariance(cv::Range(3, 6), cv::Range(3, 6)) *= medianErrSqr;
        } else {
            LOG_WARN << "Not enough close points to compute covariance!";
        }

        if (static_cast<float>(oi) / static_cast<float>(_inliers.size()) < 0.2f) {
            LOG_WARN << "A very low number of inliers have valid depth " << oi << " / " << _inliers.size() << " , the transform returned may be wrong!";
        }
    }
}

}   // namespace