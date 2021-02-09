#ifndef VISFS_MAP_2D_SUBMAP_2D_H_
#define VISFS_MAP_2D_SUBMAP_2D_H_

#include "Map/Submaps.h"
#include "Map/2d/Grid2d.h"
#include "Map/RangeDataInserterInterface.h"
#include "Sensor/RangeData.h"

namespace VISFS {
namespace Map {

class Submap2D : public Submap {
public:
    Submap2D(const Eigen::Vector2d & _origin, std::unique_ptr<Grid2D> _grid, ValueConversionTables * _conversionTables);

    const Grid2D * getGrid() const { grid_.get(); }

    void insertRangeData(const Sensor::RangeData & _rangeData, const RangeDataInserterInterface * _rangeDataInserter);

    void finish();

private:
    std::unique_ptr<Grid2D> grid_;
    ValueConversionTables * conversionTables_;

};

// The first active submap will be created on the insertion of the first range
// data. Except during this initialization when no or only one single submap
// exists, there are always two submaps into which range data is inserted: an
// old submap that is used for matching, and a new one, which will be used for
// matching next, that is being initialized.
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.

class ActiveSubmaps2D {
public:
    ActiveSubmaps2D(const ActiveSubmaps2D &) = delete;
    ActiveSubmaps2D & operator=(const ActiveSubmaps2D &) = delete;

    ActiveSubmaps2D(int _numRangeLimit, GridType _gridType, double _gridResolution, bool _insertFreeSpace, double _hitProbability, double _missProbability);

    std::vector<std::shared_ptr<const Submap2D>> insertRangeData(const Sensor::RangeData & _rangeData);

    std::vector<std::shared_ptr<const Submap2D>> submaps() const;

private:
    std::unique_ptr<RangeDataInserterInterface> createRangeDataInserter();
    std::unique_ptr<GridInterface> createGrid(const Eigen::Vector2d & _origin);
    void finishSubmap();
    void addSubmap(const Eigen::Vector2d & _origin);

    int numRangeDataLimit_; // 90
    GridType gridType_; // PROBABILITY
    double gridResolution_; // 0.05
    bool inserterFreeSpace_; // true
    double hitProbability_; // 0.55
    double missProbability_; // 0.49

    std::vector<std::shared_ptr<Submap2D>> submaps_;
    std::unique_ptr<RangeDataInserterInterface> rangeDataInserter_;
    ValueConversionTables conversionTables_;
};

}   // Map    
}   // VISFS


#endif // VISFS_MAP_2D_SUBMAP_2D_H_