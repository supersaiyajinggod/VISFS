#include "Map/2d/ProbabilityGridRangeDataInserter2D.h"
#include "Map/2d/ProbabilityGrid.h"
#include "Map/2d/RayToPixelMask.h"
#include "Map/ProbabilityValues.h"

namespace VISFS {
namespace Map {

namespace {

// Factor for subpixel accuracy of start and end point for ray cast.
constexpr int kSubpixelScale = 1000;

void growAsNeeded(const Sensor::RangeData & _rangeData, ProbabilityGrid * const _probabilityGrid) {
    Eigen::AlignedBox2d boundingBox(_rangeData.origin.head<2>());
    // Padding around bounding box to avoid numerical issues at cell boundaries.
    constexpr double kPadding = 1e-6;
    for (const Sensor::RangefinderPoint & hit : _rangeData.returns) {
        boundingBox.extend(hit.position.head<2>());
    }
    for (const Sensor::RangefinderPoint & miss : _rangeData.misses) {
        boundingBox.extend(miss.position.head<2>());
    }
    _probabilityGrid->growLimits(boundingBox.min() - kPadding * Eigen::Vector2d::Ones());
    _probabilityGrid->growLimits(boundingBox.max() + kPadding * Eigen::Vector2d::Ones());
}

void castRays(const Sensor::RangeData & _rangeData, const std::vector<uint16_t> & _hitTable, const std::vector<uint16_t> & _missTable,
                const bool _insertFreeSpace, ProbabilityGrid * _probabilityGrid) {
    growAsNeeded(_rangeData, _probabilityGrid);

    const MapLimits & limits = _probabilityGrid->limits();
    const double superscaledResolution = limits.resolution() / kSubpixelScale;
    const MapLimits superscaledLimits(superscaledResolution, limits.max(),
                CellLimits(limits.cellLimits().numXcells * kSubpixelScale, limits.cellLimits().numYcells * kSubpixelScale));
    const Eigen::Array2i begin = superscaledLimits.getCellIndex(_rangeData.origin.head<2>());
	// Compute and add the end points.
	std::vector<Eigen::Array2i> ends;
	ends.reserve(_rangeData.returns.size());
	for (const Sensor::RangefinderPoint & hit : _rangeData.returns) {
		ends.emplace_back(superscaledLimits.getCellIndex(hit.position.head<2>()));
		_probabilityGrid->applyLookUpTable(ends.back() / kSubpixelScale, _hitTable);
	}

	if (!_insertFreeSpace) { return; }

	// Now add the misses.
	for (const Eigen::Array2i & end : ends) {
		std::vector<Eigen::Array2i> ray = rayToPixelMask(begin, end, kSubpixelScale);
		for (const Eigen::Array2i & cellIndex : ray) {
			_probabilityGrid->applyLookUpTable(cellIndex, _missTable);
		}
	}

	// Finally, compute and add empty rays based on misses in the range data.
	for (const Sensor::RangefinderPoint & missingEcho : _rangeData.misses) {
		const Eigen::Array2i & missEnd = superscaledLimits.getCellIndex(missingEcho.position.head<2>());
		std::vector<Eigen::Array2i> ray = rayToPixelMask(begin, missEnd, kSubpixelScale);
		for (const Eigen::Array2i & cellIndex : ray) {
			_probabilityGrid->applyLookUpTable(cellIndex, _missTable);
		}
	}
}

}

void ProbabilityGridRangeDataInserter2D::insert(const Sensor::RangeData & _rangeData, GridInterface * _grid) const {
    ProbabilityGrid * const probabilityGrid = static_cast<ProbabilityGrid *>(_grid);
    assert(probabilityGrid != nullptr);
    castRays(_rangeData, hitTable_, missTable_, true, probabilityGrid);
    probabilityGrid->finishUpdate();
}

ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(const double _hitProbability, const double _missProbability) :
    hitProbability_(_hitProbability),
    missProbability_(_missProbability),
    hitTable_(computeLookupTableToApplyCorrespondenceCostOdds(odds(_hitProbability))),
    missTable_(computeLookupTableToApplyCorrespondenceCostOdds(odds(_missProbability))) {
}

}   // namespace Map    
}   // namespace VISFS