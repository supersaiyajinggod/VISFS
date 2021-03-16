#ifndef VISFS_SENSOR_RANGEDATA_H_
#define VISFS_SENSOR_RANGEDATA_H_

#include "Sensor/PointCloud.h"

namespace VISFS {
namespace Sensor {

struct RangeData {
    Eigen::Vector3d origin;
    PointCloud returns;
    PointCloud misses;
};

RangeData transformRangeData(const RangeData & _rangeData, const Eigen::Isometry3d & _transform);

RangeData trimRangeData(const RangeData & _rangeData, const std::string & _flied,
    const double _intervalBegin = DBL_MIN, const double _intervalEnd = DBL_MAX);

}   // namespace Sensor 
}   // namespace VISFS


#endif  // VISFS_SENSOR_RANGEDATA_H_