#include "Map/2d/ProbabilityGrid.h"

namespace VISFS {
namespace Map {

ProbabilityGrid::ProbabilityGrid(const MapLimits & _limits, ValueConversionTables * _conversionTables) : 
	Grid2D(_limits, kMinCorrespondenceCost, kMaxCorrespondenceCost, _conversionTables),
	conversionTables_(_conversionTables) {}

void ProbabilityGrid::setProbability(const Eigen::Array2i & _cellIndex, const double _probability) {
	uint16_t & cell = (*mutableCorrespondenceCostCells())[toFlatIndex(_cellIndex)];
	assert(cell == kUnknownProbabilityValue);
	cell = correspondenceCostToValue(probabilityToCorrespondenceCost(_probability));
	mutableKnownCellsBox()->extend(_cellIndex.matrix());
}

bool ProbabilityGrid::applyLookUpTable(const Eigen::Array2i & _cellIndex, const std::vector<uint16_t> & _table) {
	assert(_table.size() == kUpdateMarker);
	const int flatIndex = toFlatIndex(_cellIndex);
	uint16_t * cell = &(*mutableCorrespondenceCostCells())[flatIndex];
	if (*cell >= kUpdateMarker) {
		return false;
	}
	mutableUpdateIndices()->emplace_back(flatIndex);
	*cell = _table[*cell];
	mutableKnownCellsBox()->extend(_cellIndex.matrix());
	return true;
}

GridType ProbabilityGrid::getGridType() const {
	return GridType::PROBABILITY_GRID;
}

double ProbabilityGrid::getProbability(const Eigen::Array2i & _cellIndex) const {
	if (!limits().contains(_cellIndex)) return kMinProbability;
	return correspondenceCostToProbability(valueToCorrespondenceCost(
		correspondenceCostCells()[toFlatIndex(_cellIndex)]));
}

std::unique_ptr<Grid2D> ProbabilityGrid::computeCroppedGrid() const {
	Eigen::Array2i offset;
	CellLimits cellLimits;
	computeCroppedLimits(&offset, &cellLimits);
	const double resolution = limits().resolution();
	const Eigen::Vector2d max = limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
	std::unique_ptr<ProbabilityGrid> croppedGrid = 
		std::make_unique<ProbabilityGrid>(MapLimits(resolution, max, cellLimits), conversionTables_);
	for (const Eigen::Array2i & xyIndex : XYIndexRangeIterator(cellLimits)) {
		if (!isKnown(xyIndex + offset)) continue;
		croppedGrid->setProbability(xyIndex, getProbability(xyIndex + offset));
	}

	return std::unique_ptr<Grid2D>(croppedGrid.release());
}


}   // namespace VISFS
}   // namespace VISFS