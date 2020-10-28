#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include "InterfaceROS.h"
#include "Stl.h"

VISFSInterfaceROS::VISFSInterfaceROS(ros::NodeHandle & _n, ros::NodeHandle & _pnh) :
    pnh_(_pnh),
    // srv_(_pnh),
    queueSize_(10),
    cameraFrameId_("camera_link"),
    robotFrameId_("base_link"),
    odomFrameId_("odom"),
    publishTf_(false) {

    bool subscribeWheelOdometry = false;
    bool approxSync = true;
    double baseLine = 0.05;

    _pnh.param("subscribe_wheel_odom", subscribeWheelOdometry, subscribeWheelOdometry);
    _pnh.param("approx_sync", approxSync, approxSync);
    _pnh.param("queue_size", queueSize_, queueSize_);
    _pnh.param("camera_frame_id", cameraFrameId_, cameraFrameId_);
    _pnh.param("robot_frame_id", robotFrameId_, robotFrameId_);
    _pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
    _pnh.param("publish_tf", publishTf_, publishTf_);
    _pnh.param("base_line", baseLine, baseLine);

    ROS_INFO("VISFS_ROS_INTERFACE: subscribe_wheel_odom     = %s", subscribeWheelOdometry ? "true" : "false");
    ROS_INFO("VISFS_ROS_INTERFACE: approx_sync              = %s", approxSync ? "true" : "false");
    ROS_INFO("VISFS_ROS_INTERFACE: queue_size               = %d", queueSize_);
    ROS_INFO("VISFS_ROS_INTERFACE: camera_frame_id          = %s", cameraFrameId_.c_str());
    ROS_INFO("VISFS_ROS_INTERFACE: robot_frame_id           = %s", robotFrameId_.c_str());
    ROS_INFO("VISFS_ROS_INTERFACE: odom_frame_id            = %s", odomFrameId_.c_str());
    ROS_INFO("VISFS_ROS_INTERFACE: publish_tf               = %s", publishTf_ ? "true" : "false");
    ROS_INFO("VISFS_ROS_INTERFACE: base_line                = %lf", baseLine);

    // Dynamic reconfigure

    ros::NodeHandle leftImageNodeHandle(_n, "left");
    ros::NodeHandle rightImageNodeHandle(_n, "right");
    ros::NodeHandle leftImagePrivateNodeHandle(_pnh, "left");
    ros::NodeHandle rightImagePrivateNodeHandle(_pnh, "right");
    // Get stereo camera info
    boost::shared_ptr<sensor_msgs::CameraInfo const> cameraInfoLeft = nullptr;
    boost::shared_ptr<sensor_msgs::CameraInfo const> cameraInfoRight = nullptr;
    while (cameraInfoLeft == nullptr || cameraInfoRight == nullptr) {
        cameraInfoLeft = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(leftImageNodeHandle.resolveName("info"), leftImageNodeHandle, ros::Duration(3));
        cameraInfoRight = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(rightImageNodeHandle.resolveName("info"), rightImageNodeHandle, ros::Duration(3));
        ROS_INFO_STREAM("Wait for camera model ......");
    }
    if (cameraInfoLeft != nullptr && cameraInfoRight != nullptr) {
        cameraModelLeft_.fromCameraInfo(*cameraInfoLeft);
        cameraModelRight_.fromCameraInfo(*cameraInfoRight);
        ROS_INFO_STREAM("Get stereo camera model. \nLeft fx, fy, cx, cy: " << cameraModelLeft_.fx() << " " << cameraModelLeft_.fy() << " " << cameraModelLeft_.cx() << " " << cameraModelLeft_.cy()
                                              << "\nRight fx, fy, cx, cy: " << cameraModelRight_.fx() << " " << cameraModelRight_.fy() << " " << cameraModelRight_.cx() << " " << cameraModelRight_.cy());
    }

    parametersInit(pnh_);

    system_ = new VISFS::System(parameters_);
    system_->init(cameraModelLeft_.fx(), cameraModelLeft_.fy(), cameraModelLeft_.cx(), cameraModelLeft_.cy(), 
                    cameraModelRight_.fx(), cameraModelRight_.fy(), cameraModelRight_.cx(), cameraModelRight_.cy(), baseLine);

    // Get wheel odom
    if (subscribeWheelOdometry) {
        wheelOdomSub_ = _n.subscribe("wheel_odom", 10, &VISFSInterfaceROS::wheelOdometryCallback, this, ros::TransportHints().tcpNoDelay());
    }

    // Get stereo image and register callback 
    image_transport::ImageTransport leftIT(leftImageNodeHandle);
    image_transport::ImageTransport rightIT(rightImageNodeHandle);
    image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), leftImagePrivateNodeHandle);
    image_transport::TransportHints hintsRight("raw", ros::TransportHints(), rightImagePrivateNodeHandle);
    imageLeftSub_.subscribe(leftIT, leftImageNodeHandle.resolveName("image"), 1, hintsLeft);
    imageRightSub_.subscribe(rightIT, rightImageNodeHandle.resolveName("image"), 1, hintsRight);
    if (approxSync) {
        approxSync_ = new message_filters::Synchronizer<ImageApproxSyncPolicy>(ImageApproxSyncPolicy(queueSize_), imageLeftSub_, imageRightSub_);
        approxSync_->registerCallback(boost::bind(&VISFSInterfaceROS::stereoImageCallback, this, _1, _2));
    } else {
        exactSync_ = new message_filters::Synchronizer<ImageExactSyncPolicy>(ImageExactSyncPolicy(queueSize_), imageLeftSub_, imageLeftSub_);
        exactSync_->registerCallback(boost::bind(&VISFSInterfaceROS::stereoImageCallback, this, _1, _2));
    }
	odomPub_ = pnh_.advertise<nav_msgs::Odometry>("odom", 1);
	odomInfoPub_ = pnh_.advertise<rtabmap_ros::OdomInfo>("odom_info", 1);

}

void VISFSInterfaceROS::parametersInit(ros::NodeHandle & _pnh) {
    parameters_ = VISFS::Parameters::getDefaultParameters();

    for (auto iter = parameters_.begin(); iter != parameters_.end(); ++iter) {
        std::string vStr;
        bool vBool;
        int vInt;
        double vDouble;
        if (_pnh.getParam(iter->first, vStr)) {
            ROS_INFO("Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
            iter->second = vStr;
        } else if (_pnh.getParam(iter->first, vBool)) {
            ROS_INFO("Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uBool2Str(vBool).c_str());
            iter->second = uBool2Str(vBool);
        } else if (_pnh.getParam(iter->first, vDouble)) {
            ROS_INFO("Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
            iter->second = uNumber2Str(vDouble);
        } else if (_pnh.getParam(iter->first, vInt)) {
            ROS_INFO("Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
            iter->second = uNumber2Str(vInt);
        }

        if (iter->first.compare(VISFS::Parameters::kEstimatorMinInliers()) == 0 && atoi(iter->second.c_str()) < 8) {
            ROS_WARN("Parameter min_inliers must be >= 8, setting to 8...");
            iter->second = uNumber2Str(8);
        }

    }


}

VISFSInterfaceROS::~VISFSInterfaceROS() {
    delete system_;
}

void VISFSInterfaceROS::wheelOdometryCallback(const nav_msgs::Odometry & _wheelOdom) {
    // ROS_INFO("Odom received.");
    // ROS_INFO("%lf, time", _wheelOdom.header.stamp.toSec());
    // ROS_INFO("%f, %f, %f.", _wheelOdom.pose.pose.position.x, _wheelOdom.twist.twist.linear.x, _wheelOdom.twist.twist.angular.z);
    Eigen::Isometry3d wheelPose(Eigen::Isometry3d::Identity());
    Eigen::Isometry3d wheelVelocity(Eigen::Matrix4d::Zero());
    
    Eigen::Quaterniond q = Eigen::Quaterniond(_wheelOdom.pose.pose.orientation.w, _wheelOdom.pose.pose.orientation.x, _wheelOdom.pose.pose.orientation.y, _wheelOdom.pose.pose.orientation.z);
    wheelPose.rotate(q);
    wheelPose.pretranslate(Eigen::Vector3d(_wheelOdom.pose.pose.position.x, _wheelOdom.pose.pose.position.y, _wheelOdom.pose.pose.position.z));

    Eigen::Affine3d wheelVelocityAffine;
    pcl::getTransformation(_wheelOdom.twist.twist.linear.x, _wheelOdom.twist.twist.linear.y, _wheelOdom.twist.twist.linear.z, _wheelOdom.twist.twist.angular.x, _wheelOdom.twist.twist.angular.y, _wheelOdom.twist.twist.angular.z, wheelVelocityAffine);
    wheelVelocity = Eigen::Isometry3d(wheelVelocityAffine.matrix());

    system_->inputWheelOdometry(_wheelOdom.header.stamp.toSec(), wheelPose, wheelVelocity);
}


void VISFSInterfaceROS::stereoImageCallback(const sensor_msgs::ImageConstPtr & _leftImage, const sensor_msgs::ImageConstPtr & _rightImage) {
    cv::Mat cvImageLeft, cvImageRight;
    try {
        cvImageLeft = getImageFromROS(_leftImage);
    } catch (cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cvPtrRight;
    try {
        cvImageRight = getImageFromROS(_rightImage);
    } catch (cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    system_->inputStereoImage(_leftImage->header.stamp.toSec(), cvImageLeft, cvImageRight);
}

void VISFSInterfaceROS::publishMessage() {
    double timeStamp;
    Eigen::Isometry3d pose;
    VISFS::TrackInfo trackInfo;
    VISFS::EstimateInfo estimateInfo;
    if (system_->outputOdometryInfo(timeStamp, pose, trackInfo, estimateInfo)) {
        ros::Time stamp = ros::Time().fromSec(timeStamp);

        if (!pose.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
            geometry_msgs::TransformStamped poseMsg;
            poseMsg.child_frame_id = robotFrameId_;
            poseMsg.header.frame_id = odomFrameId_;
            poseMsg.header.stamp = stamp;        
            transformToGeometryMsg(pose, poseMsg.transform);

            if (publishTf_) {
                tfBroadcaster_.sendTransform(poseMsg);
            }
            
            // Publish the odometry message over ROS
            if (odomPub_.getNumSubscribers()) {
                nav_msgs::Odometry odom;
                odom.header.stamp = stamp;
                odom.header.frame_id = odomFrameId_;
                odom.child_frame_id = robotFrameId_;

                // Set the position
                odom.pose.pose.position.x = poseMsg.transform.translation.x;
                odom.pose.pose.position.y = poseMsg.transform.translation.y;
                odom.pose.pose.position.z = poseMsg.transform.translation.z;
                odom.pose.pose.orientation = poseMsg.transform.rotation;

                // Set covariance
                odom.pose.covariance.at(0) = estimateInfo.covariance.at<double>(0, 0)*2;    // xx
                odom.pose.covariance.at(7) = estimateInfo.covariance.at<double>(1, 1)*2;    // yy
                odom.pose.covariance.at(14) = estimateInfo.covariance.at<double>(2, 2)*2;   // zz
                odom.pose.covariance.at(21) = estimateInfo.covariance.at<double>(3, 3)*2;   // rr
                odom.pose.covariance.at(28) = estimateInfo.covariance.at<double>(4, 4)*2;   // pp
                odom.pose.covariance.at(35) = estimateInfo.covariance.at<double>(5, 5)*2;   // yawyaw

                // Set velocity
                bool setTwist = !estimateInfo.guessVelocity.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()));
                if (setTwist) {
                    double x, y, z, roll, pitch, yaw;
                    pcl::getTranslationAndEulerAngles(estimateInfo.guessVelocity, x, y ,z, roll, pitch, yaw);
                    odom.twist.twist.linear.x = x;
                    odom.twist.twist.linear.y = y;
                    odom.twist.twist.linear.z = z;
                    odom.twist.twist.angular.x = roll;
                    odom.twist.twist.angular.y = pitch;
                    odom.twist.twist.angular.z = yaw;
                }

                odom.twist.covariance.at(0) = setTwist ? estimateInfo.covariance.at<double>(0, 0) : BAD_COVARIANCE;
                odom.twist.covariance.at(7) = setTwist ? estimateInfo.covariance.at<double>(1, 1) : BAD_COVARIANCE;
                odom.twist.covariance.at(14) = setTwist ? estimateInfo.covariance.at<double>(2, 2) : BAD_COVARIANCE;
                odom.twist.covariance.at(21) = setTwist ? estimateInfo.covariance.at<double>(3, 3) : BAD_COVARIANCE;
                odom.twist.covariance.at(28) = setTwist ? estimateInfo.covariance.at<double>(4, 4) : BAD_COVARIANCE;
                odom.twist.covariance.at(35) = setTwist ? estimateInfo.covariance.at<double>(5, 5) : BAD_COVARIANCE;

                // Publish the message
                if (setTwist) {
                    odomPub_.publish(odom);
                }
            }

        } else {
            // send null pose to notify that odometry is lost.
            nav_msgs::Odometry odom;
            odom.header.stamp = stamp;
            odom.header.frame_id = robotFrameId_;
            odom.child_frame_id = cameraFrameId_;
            odom.pose.covariance.at(0) = BAD_COVARIANCE;    // xx
            odom.pose.covariance.at(7) = BAD_COVARIANCE;    // yy
            odom.pose.covariance.at(14) = BAD_COVARIANCE;   // zz
            odom.pose.covariance.at(21) = BAD_COVARIANCE;   // rr
            odom.pose.covariance.at(28) = BAD_COVARIANCE;   // pp
            odom.pose.covariance.at(35) = BAD_COVARIANCE;   // yawyaw
            odom.twist.covariance.at(0) = BAD_COVARIANCE;    // xx
            odom.twist.covariance.at(7) = BAD_COVARIANCE;    // yy
            odom.twist.covariance.at(14) = BAD_COVARIANCE;   // zz
            odom.twist.covariance.at(21) = BAD_COVARIANCE;   // rr
            odom.twist.covariance.at(28) = BAD_COVARIANCE;   // pp
            odom.twist.covariance.at(35) = BAD_COVARIANCE;   // yawyaw

            // publish the message
            odomPub_.publish(odom);
        }
        
        if (odomInfoPub_.getNumSubscribers()) {
            rtabmap_ros::OdomInfo odomOnfoMsg;
            odomInfoToROS(trackInfo, estimateInfo, odomOnfoMsg);
            odomOnfoMsg.header.stamp = stamp;
            odomOnfoMsg.header.frame_id = robotFrameId_;
            odomInfoPub_.publish(odomOnfoMsg);
        }
    }
 
}

void VISFSInterfaceROS::transformToGeometryMsg(const Eigen::Isometry3d & _transform, geometry_msgs::Transform & _msg) {
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

void VISFSInterfaceROS::odomInfoToROS(const VISFS::TrackInfo & _trackInfo, const VISFS::EstimateInfo & _estimateInfo, rtabmap_ros::OdomInfo & _msg) {
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

cv::KeyPoint VISFSInterfaceROS::keypointFromROS(const rtabmap_ros::KeyPoint & _msg) {
	return cv::KeyPoint(_msg.pt.x, _msg.pt.y, _msg.size, _msg.angle, _msg.response, _msg.octave, _msg.class_id);
}

void VISFSInterfaceROS::keypointToROS(const cv::KeyPoint & _kpt, rtabmap_ros::KeyPoint & _msg) {
	_msg.angle = _kpt.angle;
	_msg.class_id = _kpt.class_id;
	_msg.octave = _kpt.octave;
	_msg.pt.x = _kpt.pt.x;
	_msg.pt.y = _kpt.pt.y;
	_msg.response = _kpt.response;
	_msg.size = _kpt.size;
}

std::vector<cv::KeyPoint> VISFSInterfaceROS::keypointsFromROS(const std::vector<rtabmap_ros::KeyPoint> & _msg) {
	std::vector<cv::KeyPoint> v(_msg.size());
	for (unsigned int i = 0; i < _msg.size(); ++i) {
		v[i] = keypointFromROS(_msg[i]);
	}
	return v;
}

void VISFSInterfaceROS::keypointsToROS(const std::vector<cv::KeyPoint> & _kpts, std::vector<rtabmap_ros::KeyPoint> & _msg) {
	_msg.resize(_kpts.size());
	for (unsigned int i = 0; i < _msg.size(); ++i) {
		keypointToROS(_kpts[i], _msg[i]);
	}
}

void VISFSInterfaceROS::point2fToROS(const cv::Point2f & _kpt, rtabmap_ros::Point2f & _msg) {
	_msg.x = _kpt.x;
	_msg.y = _kpt.y;
}

void VISFSInterfaceROS::points2fToROS(const std::vector<cv::Point2f> & _kpts, std::vector<rtabmap_ros::Point2f> & _msg) {
	_msg.resize(_kpts.size());
	for (unsigned int i = 0; i < _msg.size(); ++i) {
		point2fToROS(_kpts[i], _msg[i]);
	}
}

void VISFSInterfaceROS::point3fToROS(const cv::Point3f & _kpt, rtabmap_ros::Point3f & _msg) {
	_msg.x = _kpt.x;
	_msg.y = _kpt.y;
	_msg.z = _kpt.z;
}

void VISFSInterfaceROS::points3fToROS(const std::vector<cv::Point3f> & _kpts, std::vector<rtabmap_ros::Point3f> & _msg) {
	_msg.resize(_kpts.size());
	for (unsigned int i = 0; i < _msg.size(); ++i) {
		point3fToROS(_kpts[i], _msg[i]);
	}
}

cv::Mat VISFSInterfaceROS::getImageFromROS(const sensor_msgs::ImageConstPtr & _imageMsg) {
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