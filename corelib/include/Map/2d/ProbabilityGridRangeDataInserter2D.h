#ifndef VISFS_MAP_2D_PROBABILITY_GRID_RANGEDATA_INSERTER_2D_H_
#define VISFS_MAP_2D_PROBABILITY_GRID_RANGEDATA_INSERTER_2D_H_

#include "Sensor/RangeData.h"
#include "Map/RangeDataInserterInterface.h"

namespace VISFS {
namespace Map {

class ProbabilityGridRangeDataInserter2D : public RangeDataInserterInterface {
public:
    ProbabilityGridRangeDataInserter2D(const double _hitProbability, const double _missProbability);

    virtual void insert(const Sensor::RangeData & _rangeData, GridInterface * _grid) const override;

private:
    const std::vector<uint16_t> hitTable_;
    const std::vector<uint16_t> missTable_;

    double hitProbability_; // 0.55
    double missProbability_; // 0.49
};

}   // namespace Map 
}   // namespace VISFS

#endif // VISFS_MAP_2D_PROBABILITY_GRID_RANGEDATA_INSERTER_2D_H_