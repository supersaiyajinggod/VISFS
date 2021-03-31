#include "Log.h"
#include "Map/2d/Submap2D.h"
#include "Map/2d/ProbabilityGridRangeDataInserter2D.h"
#include "Map/2d/ProbabilityGrid.h"

namespace VISFS {
namespace Map {

Submap2D::Submap2D(const Eigen::Isometry3d & _origin, std::unique_ptr<Grid2D> _grid, ValueConversionTables * _conversionTables) :
    Submap(_origin),
    grid_(std::move(_grid)),
    conversionTables_(_conversionTables) {
}

void Submap2D::insertRangeData(const Sensor::RangeData & _rangeData, const RangeDataInserterInterface * _rangeDataInserter) {
    assert(grid_ != nullptr);
    assert(!getInsertionStatus());
    _rangeDataInserter->insert(_rangeData, grid_.get());
    setNumRangeData(getNumRangeData() + 1);
}

void Submap2D::finish() {
    assert(grid_ != nullptr);
    assert(!getInsertionStatus());
    grid_ = grid_->computeCroppedGrid();
    setInsertionStatus(true);
}

ActiveSubmaps2D::ActiveSubmaps2D(int _numRangeLimit, GridType _gridType, double _gridResolution, bool _insertFreeSpace, double _hitProbability, double _missProbability) :
    numRangeDataLimit_(_numRangeLimit),
    gridType_(_gridType),
    gridResolution_(_gridResolution),
    inserterFreeSpace_(_insertFreeSpace),
    hitProbability_(_hitProbability),
    missProbability_(_missProbability),
    rangeDataInserter_(createRangeDataInserter()) {
}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::insertRangeData(const Sensor::RangeData & _rangeData, const Eigen::Isometry3d & _origin) {
    if (submaps_.empty() || submaps_.back()->getNumRangeData() == numRangeDataLimit_) {
        addSubmap(_origin);
    }
    for (auto & submap : submaps_) {
        // auto transform = submap->localPose().inverse() * _origin;
        // auto rangeDataInSubmap = Sensor::transformRangeData(_rangeData, transform);
        auto rangeDataInSubmap = Sensor::transformRangeData(_rangeData, _origin);
        submap->insertRangeData(rangeDataInSubmap, rangeDataInserter_.get());
    }
    if (submaps_.front()->getNumRangeData() == 2 * numRangeDataLimit_) {
        submaps_.front()->finish();
    }
    return submaps();
}

cv::Mat Submap2D::grid2Image() const {
    if (grid_ != nullptr) {
        return grid_->grid2Image();
    } else {
        std::cout << "Can not change the grid to image, because the grid object is nullptr." << std::endl;
        return cv::Mat();
    }
        
}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const {
    return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(), submaps_.end());
}

std::unique_ptr<RangeDataInserterInterface> ActiveSubmaps2D::createRangeDataInserter() {
    switch (gridType_) {
        case GridType::PROBABILITY_GRID :
            return std::make_unique<ProbabilityGridRangeDataInserter2D>(hitProbability_, missProbability_);
        default :
            LOG_FATAL << "Unknown Grid TYPE.";
    }
}

std::unique_ptr<GridInterface> ActiveSubmaps2D::createGrid(const Eigen::Vector2d & _origin) {
    constexpr int kInitialSubmapSize = 100;
    switch (gridType_) {
        case GridType::PROBABILITY_GRID :
            return std::make_unique<ProbabilityGrid>(
                MapLimits(gridResolution_, _origin + 0.5 * kInitialSubmapSize * gridResolution_ * Eigen::Vector2d::Ones(), CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
                &conversionTables_);
        default :
            LOG_FATAL << "Unknown Grid TYPE.";
    }
}

void ActiveSubmaps2D::addSubmap(const Eigen::Isometry3d & _origin) {
    if (submaps_.size() >= 2) {
        assert(submaps_.front()->getInsertionStatus());
        submaps_.erase(submaps_.begin());
    }
    submaps_.push_back(
        std::make_unique<Submap2D>(_origin,
            std::unique_ptr<Grid2D>(static_cast<Grid2D *>(createGrid(_origin.translation().head<2>()).release())),
            &conversionTables_
        )
    );
}

}   // namespace Map
}   // namespace VISFS