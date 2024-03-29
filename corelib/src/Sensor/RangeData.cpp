#include "Sensor/RangeData.h"

namespace VISFS {
namespace Sensor {

RangeData transformRangeData(const RangeData & _rangeData, const Eigen::Isometry3d & _transform) {
    return RangeData {
        _transform * _rangeData.origin,
        transformPointCloud(_rangeData.returns, _transform),
        transformPointCloud(_rangeData.misses, _transform)
    };
}

RangeData trimRangeData(const RangeData & _rangeData, const std::string & _flied,
    const double _intervalBegin, const double _intervalEnd) {
    return RangeData{_rangeData.origin,
        trimPointCloud(_rangeData.returns, _flied, _intervalBegin, _intervalEnd),
        trimPointCloud(_rangeData.misses, _flied, _intervalBegin, _intervalEnd)};
}

}   // namespace Sensor 
}   // namespace VISFS