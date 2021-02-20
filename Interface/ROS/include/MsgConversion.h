#ifndef VISFS_INTERFACE_ROS_MSG_CONVERSION_H_
#define VISFS_INTERFACE_ROS_MSG_CONVERSION_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <rtabmap_ros/OdomInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

#include "Signature.h"
#include "Sensor/PointCloud.h"

inline double timestampFromROS(const ros::Time & _stamp) { return double(_stamp.sec) + double(_stamp.nsec)/1000000000.0; }

void odomInfoToROS(const VISFS::TrackInfo & _trackInfo, const VISFS::EstimateInfo & _estimateInfo, rtabmap_ros::OdomInfo & _msg);

void transformToGeometryMsg(const Eigen::Isometry3d & _transform, geometry_msgs::Transform & _msg);

cv::KeyPoint keypointFromROS(const rtabmap_ros::KeyPoint & _msg);

void keypointToROS(const cv::KeyPoint & _kpt, rtabmap_ros::KeyPoint & _msg);

void keypointsToROS(const std::vector<cv::KeyPoint> & _kpts, std::vector<rtabmap_ros::KeyPoint> & _msg);

void point2fToROS(const cv::Point2f & _kpt, rtabmap_ros::Point2f & _msg);

void points2fToROS(const std::vector<cv::Point2f> & _kpts, std::vector<rtabmap_ros::Point2f> & _msg);

void point3fToROS(const cv::Point3f & _kpt, rtabmap_ros::Point3f & _msg);

void points3fToROS(const std::vector<cv::Point3f> & _kpts, std::vector<rtabmap_ros::Point3f> & _msg);

std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_ros::KeyPoint> & _msg);

cv::Mat getImageFromROS(const sensor_msgs::ImageConstPtr & _imageMsg);

void laserScanToTimedPointCloudWithIntensities(const sensor_msgs::LaserScanConstPtr & _laserScan, VISFS::Sensor::TimedPointCloudWithIntensities & _pointCloud);

#endif // VISFS_INTERFACE_ROS_MSG_CONVERSION_H_