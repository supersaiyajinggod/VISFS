#ifndef VISFS_MULTIVIEW_GEOMETRY
#define VISFS_MULTIVIIE_GEOMETRY

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "CameraModels/PinholeModel.h"

namespace VISFS {

/** \brief Return if the point is finite.
  * \param[in] point The point.
  * \return True: finite, False: infinite.
  * \author eddy
  */
bool isFinite(const cv::Point3f & _point);

/** \brief Return if the point transformed.
  * \param[in] point The point.
  * \param[in] model The camera model.
  * \return The transformed point.
  * \author eddy
  */
cv::Point3f transformPoint(const cv::Point3f & _points, const GeometricCamera & _model);

/** \brief Return if the point transformed.
  * \param[in] point The point.
  * \param[in] model The transform.
  * \return The transformed point.
  * \author eddy
  */
cv::Point3f transformPoint(const cv::Point3f & _points, const Eigen::Isometry3d & _transform);

/** \brief Calculate the keypoints' depth with stereo model.
  * \param[in] kptsLeft The keypoints in left image.
  * \param[in] kptsRight The keypoints in right iamge.
  * \param[in] cameraLeft The left camera model.
  * \param[in] cameraRight The right caemra model.
  * \param[in] minDepth The limit of the closest point.
  * \param[in] maxDepth The limit of the farest point.
  * \return The keypoints' depth.
  * \author eddy
  */
std::vector<cv::Point3f> generateKeyPoints3DStereo(const std::vector<cv::KeyPoint> & _kptsLeft, const std::vector<cv::KeyPoint> & _kptsRight, const GeometricCamera & _cameraLeft, const GeometricCamera & _cameraRight, float _minDepth, float _maxDepth);

/** \brief Calculate the corner's depth with stereo model and disparity.
  * \param[in] corner The corner in left image.
  * \param[in] disparity The disparity of the corners between left and right image.(ulx - urx)
  * \param[in] cameraLeft The left camera model.
  * \param[in] cameraRight The right caemra model.
  * \return The Corner's depth.
  * \author eddy
  */
cv::Point3f projectDisparityTo3D(const cv::Point2f & _corner, float _disparity, const GeometricCamera & _cameraLeft, const GeometricCamera & _cameraRight);

/** \brief Calculate the keypoints' depth with depth image.
  * \param[in] kpts The keypoints in the image.
  * \param[in] depthImage The depth image.
  * \param[in] cameraModel The camera model.
  * \param[in] minDepth The limit of the closest point.
  * \param[in] maxDepth The limit of the farest point.
  * \return The keypoints' depth.
  * \author eddy
  */
std::vector<cv::Point3f> generateKeyPoints3DDepth(const std::vector<cv::KeyPoint> & _kpts, const cv::Mat & _depthImage, const GeometricCamera & _cameraModel, float _minDepth, float _maxDepth);

/** \brief Estimate the motion between two image.
  * \param[in] words3dFrom The 3d points in first image.
  * \param[in] words2dTo The keypoint coordnate in second image.
  * \param[in] cameraModel The camera model.
  * \param[in] minInliers The limit of the inliers.
  * \param[in] iterations The iteration times in calculation.
  * \param[in] reProjError PnP reprojection error.
  * \param[in] flagPnP PnP flags: 0=Iterative, 1=EPNP, 2=P3P.
  * \param[in] refineIteration Number of iterations used to refine the transformation found by RANSAC.
  * \param[in] words3dTo The 3d points of keypoint in second image.
  * \param[out] covariance The covariance of the calculation result.
  * \param[out] matchesOut The ids of all matched keypoints.
  * \param[out] inliersOut The ids of all inlier keypoints.
  * \return The transform of motion.
  * \author eddy
  */
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
	const Eigen::Isometry3d & _guess);

/** \brief Use ransac to solve pnp problem.
  * \param[in] objectPoint The 3d points in first image.
  * \param[in] imagePoints The 2d points in the second image.
  * \param[in] cameraModel The intrinsic of camera model.
  * \param[in] distCoeffs The distortion coefficient of camera model.
  * \param[out] rvec The rotation.
  * \param[out] tvec The tanslation.
  * \param[in] iterationsCount The iteration times in calculation.
  * \param[in] reprojectionError PnP reprojection error.
  * \param[in] minInliersCount The limit of the inliers.
  * \param[out] inliers The inliers id.
  * \param[in] flags PnP flags: 0=Iterative, 1=EPNP, 2=P3P.
  * \param[in] refineIterations Number of iterations used to refine the transformation found by RANSAC.
  * \param[in] refineSigma Coefficient of error and variance.
  * \author eddy
  */
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
	int _refineIterations = 1,
	float _refineSigma = 3.0f);

/** \brief Compute .
  * \param[in] objectPoint The 3d points in first image.
  * \param[in] imagePoints The 2d points in the second image.
  * \param[in] cameraModel The intrinsic of camera model.
  * \param[in] distCoeffs The distortion coefficient of camera model.
  * \param[in] rvec The rotation.
  * \param[in] tvec The tanslation.
  * \param[in] reprojectionError PnP reprojection error.
  * \param[out] inliers The ids of all inlier keypoints.
  * \return The reproject errors of all inliers.
  * \author eddy
  */
std::vector<float> computeReprojErrors(
	const std::vector<cv::Point3f> & _objectPoints,
	const std::vector<cv::Point2f> & _imagePoints,
	const cv::Mat & _cameraModel,
	const cv::Mat & _distCoeffs,
	const cv::Mat & _rvec,
	const cv::Mat & _tvec,
	const float _reProjErrorThreshold,
	std::vector<std::size_t> _inliers
);

}   // VISFS

#endif  // VISFS_MULTIVIIE_GEOMETRY
