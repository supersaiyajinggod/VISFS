#include "InterfaceROS.h"

VISFSInterfaceROS::VISFSInterfaceROS(ros::NodeHandle & _n, ros::NodeHandle & _pnh) :
    pnh_(_pnh),
    // srv_(_pnh),
    queueSize_(10) {

    bool approxSync = true;
    _pnh.param("approx_sync", approxSync, approxSync);
    _pnh.param("queue_size", queueSize_, queueSize_);

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

    double baseLine = 0.05;
    pnh_.param("base_line", baseLine, baseLine);
    system_ = new VISFS::System(parameters_);
    system_->init(cameraModelLeft_.fx(), cameraModelLeft_.fy(), cameraModelLeft_.cx(), cameraModelLeft_.cy(), 
                    cameraModelRight_.fx(), cameraModelRight_.fy(), cameraModelRight_.cx(), cameraModelRight_.cy(), baseLine);

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
            ROS_INFO("Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
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

void VISFSInterfaceROS::stereoImageCallback(const sensor_msgs::ImageConstPtr & _leftImage, const sensor_msgs::ImageConstPtr & _rightImage) {
    // ROS_INFO_STREAM("RUNNING CALLBACK...");
    cv_bridge::CvImageConstPtr cvPtrLeft;
    try {
        cvPtrLeft = cv_bridge::toCvShare(_leftImage);
    } catch (cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cvPtrRight;
    try {
        cvPtrRight = cv_bridge::toCvShare(_rightImage);
    } catch (cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    ros::Time::now();
    system_->inputStereoImage(cvPtrLeft->header.stamp.toSec(), cvPtrLeft->image, cvPtrRight->image);
}

void VISFSInterfaceROS::publishMessage() {
    Eigen::Isometry3d pose;
    VISFS::TrackInfo trackInfo;
    VISFS::EstimateInfo estimateInfo;
    system_->outputOdometryInfo(pose, trackInfo, estimateInfo);

    geometry_msgs::TransformStamped poseMsg;
    rtabmap_ros::OdomInfo odomOnfoMsg;
}

