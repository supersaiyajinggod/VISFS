#include "Sensor/PointCloud.h"
#include "Log.h"

namespace VISFS {
namespace Sensor {

PointCloud::PointCloud(std::vector<PointType> _points) :
    points_(std::move(_points)) {}

PointCloud::PointCloud(std::vector<PointType> _points, std::vector<float> _intensities) :
    points_(std::move(_points)), intensities_(std::move(_intensities)) {
    if (!intensities_.empty()) {
        assert(points_.size() == intensities_.size());
    }
}

const PointCloud::PointType & PointCloud::operator[](const std::size_t _index) const {
    return points_[_index];
}

PointCloud trimPointCloud(const PointCloud & _pointCloud, const std::string & _flied,
    const double _intervalBegin, const double _intervalEnd) {
    if (_flied.compare("x") && _flied.compare("y") && _flied.compare("z")) {
        LOG_FATAL << "Unable to find field name in point type. Only x, y or z make sense.";
        assert(0);
    }
    if (_intervalEnd > _intervalBegin) {
        LOG_FATAL << "The intervalEnd must larger than intervalBegin.";
        assert(0);
    }

    return _pointCloud.copy_if([_flied, _intervalBegin, _intervalEnd](const RangefinderPoint & _point){
        if(!_flied.compare("x")) {
            return _intervalBegin <= _point.position.x() && _point.position.x() <= _intervalEnd;
        } else if (!_flied.compare("y")) {
            return _intervalBegin <= _point.position.y() && _point.position.y() <= _intervalEnd;
        } else if (!_flied.compare("z")) {
            return _intervalBegin <= _point.position.z() && _point.position.z() <= _intervalEnd;
        }
    });
}

PointCloud transformPointCloud(const PointCloud & _pointCloud, const Eigen::Isometry3d & _transform) {
    std::vector<RangefinderPoint> points;
    points.reserve(_pointCloud.size());
    for (const RangefinderPoint & point : _pointCloud.points()) {
        points.emplace_back(_transform * point);
    }
    return PointCloud(points, _pointCloud.intensities());
}

}   // namespace Sensor 
}   // namespace VISFS