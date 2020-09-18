#ifndef VISFS_INTERFACE_ROS
#define VISFS_INTERFACE_ROS

#include <ros/ros.h>
#include <ros/topic.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <rtabmap_ros/OdomInfo.h>
#include <boost/shared_ptr.hpp>

#include "System.h"
#include "Conversion.h"

#define BAD_COVARIANCE 9999

class VISFSInterfaceROS {
public:
    VISFSInterfaceROS(ros::NodeHandle & _n, ros::NodeHandle & _pnh);
    ~VISFSInterfaceROS();

    void publishMessage();

private:
    void parametersInit(ros::NodeHandle & _pnh);

    inline double timestampFromROS(const ros::Time & _stamp) { return double(_stamp.sec) + double(_stamp.nsec)/1000000000.0; }

    void transformToGeometryMsg(const Eigen::Isometry3d & _transform, geometry_msgs::Transform & _msg);

    void odomInfoToROS(const VISFS::TrackInfo & _trackInfo, const VISFS::EstimateInfo & _estimateInfo, rtabmap_ros::OdomInfo & _msg);

    cv::KeyPoint keypointFromROS(const rtabmap_ros::KeyPoint & _msg);

    void keypointToROS(const cv::KeyPoint & _kpt, rtabmap_ros::KeyPoint & _msg);

    void keypointsToROS(const std::vector<cv::KeyPoint> & _kpts, std::vector<rtabmap_ros::KeyPoint> & _msg);

    void point2fToROS(const cv::Point2f & _kpt, rtabmap_ros::Point2f & _msg);

    void points2fToROS(const std::vector<cv::Point2f> & _kpts, std::vector<rtabmap_ros::Point2f> & _msg);

    void point3fToROS(const cv::Point3f & _kpt, rtabmap_ros::Point3f & _msg);

    void points3fToROS(const std::vector<cv::Point3f> & _kpts, std::vector<rtabmap_ros::Point3f> & _msg);

    std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_ros::KeyPoint> & _msg);

    cv::Mat getImageFromROS(const sensor_msgs::ImageConstPtr & _imageMsg);

	/** \brief Callback for process stereo image coming. 
      * \param[in] Left image. 
      * \param[in] Right iamge.
	  * \author eddy
      */
    void stereoImageCallback(const sensor_msgs::ImageConstPtr & _leftImage, const sensor_msgs::ImageConstPtr & _rightImage);

	/** \brief Dynamic reconfigure callback. 
      * \param[in] Dynamic reconfigure object. 
      * \param[in] Dynamic reconfigure level.
	  * \author eddy
      */
	// void reconfigureCallback(obstacle_detector::DynamicParamConfig & _config, uint32_t _level); 

    ros::NodeHandle pnh_;
    image_transport::SubscriberFilter imageLeftSub_;
    image_transport::SubscriberFilter imageRightSub_;
    image_geometry::PinholeCameraModel cameraModelLeft_;
    image_geometry::PinholeCameraModel cameraModelRight_;
    tf::TransformListener tfListener_;
    // dynamic_reconfigure::Server<xxxxx> srv_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ImageApproxSyncPolicy;
    message_filters::Synchronizer<ImageApproxSyncPolicy> * approxSync_;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ImageExactSyncPolicy;
    message_filters::Synchronizer<ImageExactSyncPolicy> * exactSync_;
	ros::Publisher odomPub_;
	ros::Publisher odomInfoPub_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    int queueSize_;
    std::string cameraFrameId_;
    std::string robotFrameId_;
    std::string odomFrameId_;
    bool publishTf_;

    VISFS::System * system_;
    VISFS::ParametersMap parameters_;

};

#endif  // VISFS_INTERFACE_ROS