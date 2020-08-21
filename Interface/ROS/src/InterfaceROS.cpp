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

}

VISFSInterfaceROS::~VISFSInterfaceROS() {
    // delete algorithm;
}

void VISFSInterfaceROS::stereoImageCallback(const sensor_msgs::ImageConstPtr & _leftImage, const sensor_msgs::ImageConstPtr & rightImage) {
    ROS_INFO_STREAM("RUNNING CALLBACK...");
}

