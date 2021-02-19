#include "MsgConversion.h"
#include "Stl.h"
#include <tf_conversions/tf_eigen.h>
#include <cv_bridge/cv_bridge.h>

cv::KeyPoint keypointFromROS(const rtabmap_ros::KeyPoint & _msg) {
	return cv::KeyPoint(_msg.pt.x, _msg.pt.y, _msg.size, _msg.angle, _msg.response, _msg.octave, _msg.class_id);
}

void keypointToROS(const cv::KeyPoint & _kpt, rtabmap_ros::KeyPoint & _msg) {
	_msg.angle = _kpt.angle;
	_msg.class_id = _kpt.class_id;
	_msg.octave = _kpt.octave;
	_msg.pt.x = _kpt.pt.x;
	_msg.pt.y = _kpt.pt.y;
	_msg.response = _kpt.response;
	_msg.size = _kpt.size;
}

std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_ros::KeyPoint> & _msg) {
	std::vector<cv::KeyPoint> v(_msg.size());
	for (unsigned int i = 0; i < _msg.size(); ++i) {
		v[i] = keypointFromROS(_msg[i]);
	}
	return v;
}

void keypointsToROS(const std::vector<cv::KeyPoint> & _kpts, std::vector<rtabmap_ros::KeyPoint> & _msg) {
	_msg.resize(_kpts.size());
	for (unsigned int i = 0; i < _msg.size(); ++i) {
		keypointToROS(_kpts[i], _msg[i]);
	}
}

void point2fToROS(const cv::Point2f & _kpt, rtabmap_ros::Point2f & _msg) {
	_msg.x = _kpt.x;
	_msg.y = _kpt.y;
}

void points2fToROS(const std::vector<cv::Point2f> & _kpts, std::vector<rtabmap_ros::Point2f> & _msg) {
	_msg.resize(_kpts.size());
	for (unsigned int i = 0; i < _msg.size(); ++i) {
		point2fToROS(_kpts[i], _msg[i]);
	}
}

void point3fToROS(const cv::Point3f & _kpt, rtabmap_ros::Point3f & _msg) {
	_msg.x = _kpt.x;
	_msg.y = _kpt.y;
	_msg.z = _kpt.z;
}

void points3fToROS(const std::vector<cv::Point3f> & _kpts, std::vector<rtabmap_ros::Point3f> & _msg) {
	_msg.resize(_kpts.size());
	for (unsigned int i = 0; i < _msg.size(); ++i) {
		point3fToROS(_kpts[i], _msg[i]);
	}
}

cv::Mat getImageFromROS(const sensor_msgs::ImageConstPtr & _imageMsg) {
    cv_bridge::CvImageConstPtr ptr;
    if (_imageMsg->encoding == "8UC1") {
        sensor_msgs::Image image;
        image.header = _imageMsg->header;
        image.height = _imageMsg->height;
        image.width = _imageMsg->width;
        image.is_bigendian = _imageMsg->is_bigendian;
        image.step = _imageMsg->is_bigendian;
        image.data = _imageMsg->data;
        image.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
    } else {
        ptr = cv_bridge::toCvCopy(_imageMsg, sensor_msgs::image_encodings::MONO8);
    }
    cv::Mat cvImage = ptr->image.clone();
    return cvImage;
}

void transformToGeometryMsg(const Eigen::Isometry3d & _transform, geometry_msgs::Transform & _msg) {
	if (!_transform.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
        tf::StampedTransform transformMsg;
		tf::transformEigenToTF(_transform, transformMsg);
		// make sure the quaternion is normalized
        tf::Quaternion q = transformMsg.getRotation();
        q.normalize();
        transformMsg.setRotation(q);
        transformTFToMsg(transformMsg, _msg);
	} else {
		_msg = geometry_msgs::Transform();
	}
}

void odomInfoToROS(const VISFS::TrackInfo & _trackInfo, const VISFS::EstimateInfo & _estimateInfo, rtabmap_ros::OdomInfo & _msg) {
    _msg.lost = _estimateInfo.lost;
    _msg.matches = _trackInfo.matches;
    if (_estimateInfo.covariance.type() == CV_64FC1 && _estimateInfo.covariance.cols == 6 && _estimateInfo.covariance.rows == 6) {
        memcpy(_msg.covariance.data(), _estimateInfo.covariance.data, 36*sizeof(double));
    }
    _msg.features = _estimateInfo.features;
    _msg.localMapSize = _estimateInfo.localMapSize;
    _msg.localKeyFrames = _estimateInfo.localKeyFrames;
    _msg.localBundleOutliers = _estimateInfo.localBundleOutliers;
    _msg.localBundleConstraints = _estimateInfo.localBundleConstraints;
    _msg.localBundleTime = _estimateInfo.localBundleTime;
    _msg.timeEstimation = _estimateInfo.timeEstimation;
    _msg.timeParticleFiltering = _estimateInfo.timeParticleFiltering;
    _msg.stamp = _estimateInfo.stamp;
    _msg.interval = _estimateInfo.interval;
    _msg.distanceTravelled = _estimateInfo.distanceTravelled;
    _msg.memoryUsage = _estimateInfo.memoryUsage;

    std::vector<std::size_t> keys = uKeys(_estimateInfo.words);
    std::vector<int> wordsId(keys.size());
    for (std::size_t i = 0; i < keys.size(); ++i) {
        wordsId[i] = keys[i];
    }
    _msg.wordsKeys = wordsId;
    keypointsToROS(uValues(_estimateInfo.words), _msg.wordsValues);

    std::vector<int> matchesIDs;
    std::vector<int> inliersIDs;
    matchesIDs.resize(_trackInfo.matchesIDs.size());
    inliersIDs.resize(_trackInfo.inliersIDs.size());
    for (std::size_t i = 0; i < _trackInfo.matchesIDs.size(); ++i) {
        matchesIDs[i] = _trackInfo.matchesIDs[i];
    }
    for (std::size_t i = 0; i < _trackInfo.inliersIDs.size(); ++i) {
        inliersIDs[i] = _trackInfo.inliersIDs[i];
    }
    _msg.wordMatches = matchesIDs;
    _msg.wordInliers = inliersIDs;

	points2fToROS(_estimateInfo.refCorners, _msg.refCorners);
	points2fToROS(_estimateInfo.newCorners, _msg.newCorners);
    std::vector<int> cornerInliers;
    cornerInliers.resize(_estimateInfo.cornerInliers.size());
    for (std::size_t i = 0; i < _estimateInfo.cornerInliers.size(); ++i) {
        cornerInliers[i] = _estimateInfo.cornerInliers[i];
    }
    _msg.cornerInliers = cornerInliers;

	transformToGeometryMsg(_estimateInfo.transform, _msg.transform);
	transformToGeometryMsg(_estimateInfo.transformFiltered, _msg.transformFiltered);
	transformToGeometryMsg(_estimateInfo.transformGroundTruth, _msg.transformGroundTruth);
	transformToGeometryMsg(_estimateInfo.guessVelocity, _msg.guessVelocity);

    keys = uKeys(_estimateInfo.localMap);
    std::vector<int> localMapKeys(keys.size());
    for (std::size_t i = 0; i < keys.size(); ++i) {
        localMapKeys[i] = keys[i];
    }
    _msg.localMapKeys = localMapKeys;
    points3fToROS(uValues(_estimateInfo.localMap), _msg.localMapValues);
}

void laserScanToTimedPointCloudWithIntensities(const sensor_msgs::LaserScanConstPtr & _laserScan, VISFS::Sensor::TimedPointCloudWithIntensities & _pointCloud) {
    assert(_laserScan->range_min > 0.f);
    assert(_laserScan->range_max > _laserScan->range_min);
    if (_laserScan->angle_increment > 0.f) {
        assert(_laserScan->angle_max > _laserScan->angle_min);
    } else {
        assert(_laserScan->angle_min > _laserScan->angle_max);
    }

    const double rangeMin = static_cast<double>(_laserScan->range_min);
    const double rangeMax = static_cast<double>(_laserScan->range_max);
    const double timeIncrement = static_cast<double>(_laserScan->time_increment);
    const double angleIncrement = static_cast<double>(_laserScan->angle_increment);
    double angle = static_cast<double>(_laserScan->angle_min);
    for (std::size_t i = 0; i < _laserScan->ranges.size(); ++i) {
        const double range = static_cast<double>(_laserScan->ranges[i]);
        if (rangeMin <= range && range <= rangeMax) {
            const Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::UnitZ());
            const VISFS::Sensor::TimedRangefinderPoint point {
                rotation * (range * Eigen::Vector3d::UnitX()),
                i * timeIncrement
            };
            _pointCloud.points.emplace_back(point);
            if (_laserScan->intensities.size() == _laserScan->ranges.size()) {
                _pointCloud.intensities.emplace_back(_laserScan->intensities[i]);
            } else {
                _pointCloud.intensities.emplace_back(0.f);
            }
        }
        angle += angleIncrement;
    }
    double timeStamp = timestampFromROS(_laserScan->header.stamp);
    if (!_pointCloud.points.empty()) {
        const double duration = _pointCloud.points.back().time;
        timeStamp += duration;
        for (auto & point : _pointCloud.points) {
            point.time -= duration;
        }
    }
    _pointCloud.time = timeStamp;
}