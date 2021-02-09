#ifndef VISFS_MAP_RANGEDATA_INSERTER_INTERFACE_H_
#define VISFS_MAP_RANGEDATA_INSERTER_INTERFACE_H_

#include "Sensor/RangeData.h"
#include "Map/GridInterface.h"

namespace VISFS {
namespace Map {

class RangeDataInserterInterface {
public:
    virtual ~RangeDataInserterInterface() {}

    // Insert rangeData into grid.
    virtual void insert(const Sensor::RangeData & _rangeData, GridInterface * _grid) const = 0;
};

}   // namespace Map 
}   // namespace VISFS

#endif  // VISFS_MAP_RANGEDATA_INSERTER_INTERFACE_H_