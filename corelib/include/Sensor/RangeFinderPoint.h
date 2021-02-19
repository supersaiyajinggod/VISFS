#ifndef VISFS_SENSOR_RANGEFINDER_POINT_H_
#define VISFS_SENSOR_RANGEFINDER_POINT_H_

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace VISFS {
namespace Sensor {

struct RangefinderPoint {
    Eigen::Vector3d position;
};

struct TimedRangefinderPoint {
    Eigen::Vector3d position;
    double time;
};

inline RangefinderPoint operator*(const Eigen::Isometry3d & _lhs, const RangefinderPoint & _rhs) {
    RangefinderPoint result = _rhs;
    result.position = _lhs * _rhs.position;
    return result;
}

inline bool operator==(const RangefinderPoint & _lhs, const RangefinderPoint & _rhs) {
    return _lhs.position == _rhs.position;
}

inline TimedRangefinderPoint operator*(const Eigen::Isometry3d & _lhs, const TimedRangefinderPoint & _rhs) {
    TimedRangefinderPoint result = _rhs;
    result.position = _lhs * _rhs.position;
    return result;
}

inline bool operator==(const TimedRangefinderPoint & _lhs, const TimedRangefinderPoint & _rhs) {
    return _lhs.position == _rhs.position && _lhs.time == _rhs.time;
}

}   // Sensor     
}   // VISFS

#endif // VISFS_SENSOR_RANGEFINDER_POINT_H_