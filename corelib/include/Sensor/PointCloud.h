#ifndef VISFS_SENSOR_POINT_CLOUD_H_
#define VISFS_SENSOR_POINT_CLOUD_H_

#include <vector>
#include <string>
#include <float.h>
#include <Eigen/Core>

#include "Sensor/RangeFinderPoint.h"

namespace VISFS {
namespace Sensor {

class PointCloud {
public:
    using PointType = RangefinderPoint;

    PointCloud() = default;

    explicit PointCloud(std::vector<PointType> _points);

    PointCloud(std::vector<PointType> _points, std::vector<float> _intensities);

    std::size_t size() const { return points_.size(); }
    
    bool empty() const { return points_.empty(); }

    const std::vector<PointType> & points() const { return points_; }

    const std::vector<float> & intensities() const { return intensities_; }

    const PointType & operator[](const std::size_t _index) const;

    using constIterator = std::vector<PointType>::const_iterator;
    constIterator begin() const { return points_.begin(); }
    constIterator end() const { return points_.end(); }

    void push_back(PointType _value) { points_.push_back(std::move(_value)); }

    template <typename UnaryPredicate>
    PointCloud copy_if (UnaryPredicate _predicate) const {
        std::vector<PointType> points;
        std::vector<float> intensities;

        if (intensities_.empty()) {
            for (std::size_t index = 0; index < size(); ++index) {
                const PointType & point = points_[index];
                if (_predicate(point)) {
                    points.emplace_back(point);
                }
            }
        } else {
            for (std::size_t index = 0; index < size(); ++index) {
                const PointType & point = points_[index];
                if (_predicate(point)) {
                    points.emplace_back(point);
                    intensities.emplace_back(intensities_[index]);
                }
            }
        }

        return PointCloud(points, intensities);
    }

private:
    // For 2D points, the third entry is 0.0;
    std::vector<PointType> points_;
    // Intensities are optional. If non-empty, they must have the same size as points.
    std::vector<float> intensities_;

};

using TimedPointCloud = std::vector<TimedRangefinderPoint>;
struct TimedPointCloudWithIntensities {
    TimedPointCloud points;
    Eigen::Vector3d origin;
    std::vector<float> intensities;
    double time;
};

PointCloud trimPointCloud(const PointCloud & _pointCloud, const std::string & _flied,
    const double _intervalBegin = DBL_MIN, const double _intervalEnd = DBL_MAX);

PointCloud transformPointCloud(const PointCloud & _pointCloud, const Eigen::Isometry3d & _transform);

}   // namespace Sensor
}   // VISFS

#endif