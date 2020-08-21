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
#include <dynamic_reconfigure/server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/shared_ptr.hpp>

class VISFSInterfaceROS {
public:
    VISFSInterfaceROS(ros::NodeHandle & _n, ros::NodeHandle & _pnh);
    ~VISFSInterfaceROS();

private:
	/** \brief Callback for process stereo image coming. 
      * \param[in] Left image. 
      * \param[in] Right iamge.
	  * \author eddy
      */
    void stereoImageCallback(const sensor_msgs::ImageConstPtr & _leftImage, const sensor_msgs::ImageConstPtr & rightImage);

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

    int queueSize_;
    std::string cameraFrameId_;
    std::string robotFrameId_;

    VISFS::System * system_;

};

#endif  // VISFS_INTERFACE_ROS