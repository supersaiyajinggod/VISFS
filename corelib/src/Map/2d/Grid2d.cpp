#include "Map/2d/Grid2d.h"
#include <iostream>

namespace VISFS {
namespace Map {

Grid2D::Grid2D(const MapLimits & _limits, double _minCorrespondenceCost, double _maxCorrespondenceCost, ValueConversionTables * _conversionTables) :
    limits_(_limits),
    correspondenceCostCells_(limits_.cellLimits().numXcells * limits_.cellLimits().numYcells, kUnknownCorrespondenceValue),
    minCorrespondenceCost_(_minCorrespondenceCost),
    maxCorrespondenceCost_(_maxCorrespondenceCost),
    valueToCorrespondenceCostTable_(_conversionTables->getConversionTables(
        _maxCorrespondenceCost, _minCorrespondenceCost, _maxCorrespondenceCost)) {
    assert(minCorrespondenceCost_ < maxCorrespondenceCost_);
} 

void Grid2D::finishUpdate() {
    while (!updateIndices_.empty()) {
        correspondenceCostCells_[updateIndices_.back()] -= kUpdateMarker;
        updateIndices_.pop_back();
    }
}

void Grid2D::computeCroppedLimits(Eigen::Array2i * const _offset, CellLimits * const _limits) const {
    if (knownCellsBox_.isEmpty()) {
        *_offset = Eigen::Array2i::Zero();
        *_limits = CellLimits(1, 1);
        return;
    }
    *_offset = knownCellsBox_.min().array();
    *_limits = CellLimits(knownCellsBox_.sizes().x() + 1, knownCellsBox_.sizes().y() + 1);
}

void Grid2D::growLimits(const Eigen::Vector2d & _point) {
    growLimits(_point, {mutableCorrespondenceCostCells()}, {kUnknownCorrespondenceValue});
}

void Grid2D::growLimits(const Eigen::Vector2d & _point, const std::vector<std::vector<uint16_t> *> & _grids,
                const std::vector<uint16_t> & _gridsUnknownCellValues) {
    assert(updateIndices_.empty());
    while (!limits_.contains(limits_.getCellIndex(_point))) {
        const int xOffset = limits_.cellLimits().numXcells / 2;
        const int yOffset = limits_.cellLimits().numYcells / 2;
        const MapLimits newLimits(limits_.resolution(),
                    limits_.max() + limits_.resolution() * Eigen::Vector2d(yOffset, xOffset),
                    CellLimits(2 * limits_.cellLimits().numXcells, 2 * limits_.cellLimits().numYcells));
        const int stride = newLimits.cellLimits().numXcells;
        const int offset = xOffset + stride * yOffset;
        const int newSize = newLimits.cellLimits().numXcells * newLimits.cellLimits().numYcells;

        for (std::size_t gridIndex = 0; gridIndex < _grids.size(); ++gridIndex) {
            std::vector<uint16_t> newCells(newSize, _gridsUnknownCellValues[gridIndex]);
            for (int i = 0; i < limits_.cellLimits().numYcells; ++i) {
                for (int j = 0; j < limits_.cellLimits().numXcells; ++j) {
                    newCells[offset + j + i * stride] = (*_grids[gridIndex])[j + i * limits_.cellLimits().numXcells];
                }
            }
            *_grids[gridIndex] = newCells;
        }
        limits_ = newLimits;
        if (!knownCellsBox_.isEmpty()) {
            knownCellsBox_.translate(Eigen::Vector2i(xOffset, yOffset));
        }
    }
}

}   // namespace Map
}   // namespace VISFS