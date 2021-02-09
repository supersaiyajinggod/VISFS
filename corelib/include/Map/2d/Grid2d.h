#ifndef VISFS_MAP_2D_GRID2D_H_
#define VISFS_MAP_2D_GRID2D_H_

#include "Map/ProbabilityValues.h"
#include "Map/ValueConversionTables.h"
#include "Map/GridInterface.h"
#include "Map/2d/MapLimits.h"

#include <Eigen/Geometry>

namespace VISFS {
namespace Map {

enum class GridType { PROBABILITY_GRID, TSDF };

class Grid2D : public GridInterface {
public:
    Grid2D(const MapLimits & _limits, double _minCorrespondenceCost, double _maxCorrespondenceCost, ValueConversionTables * _conversionTables);

    /** \brief Return the limits of this Grid2D.
     * \return The limits.
     * \author eddy
     */
    const MapLimits & limits() const { return limits_; }

    /** \brief Finish the update sequence.
     * \author eddy
     */    
    void finishUpdate();

    /** \brief Return the limits of this Grid2D.
     * \return The limits.
     * \author eddy
     */
    float getCorrespondenceCost(const Eigen::Array2i & _cellIndex) const {
        if (!limits().contains(_cellIndex)) return maxCorrespondenceCost_;
        return (*valueToCorrespondenceCostTable_)[correspondenceCostCells()[toFlatIndex(_cellIndex)]];
    }

    /** \brief Return the type of this Grid2D.
     * \return The type.
     * \author eddy
     */
    virtual GridType getGridType() const = 0;

    float getMinCorrespondenceCost() const { return minCorrespondenceCost_; }

    float getMaxCorrespondenceCost() const { return maxCorrespondenceCost_; }

    bool isKnown(const Eigen::Array2i & _cellIndex) const {
        return limits_.contains(_cellIndex) 
                && correspondenceCostCells_[toFlatIndex(_cellIndex)] != kUnknownCorrespondenceValue;
    }

    /** \brief Fills in 'offset' and 'limits' to define a subregion of that contains all known cells.
     * \author eddy
     */
    void computeCroppedLimits(Eigen::Array2i * const _offset, CellLimits * const _limits) const;

    /** \brief Grows the map as necessary to include 'point'. This changes the meaning of these coordinate
     *          going forward. This method must be called immediately after 'finishUpdate', 
     *          before any calls to 'ApplyLookupTable'. 
     * \author eddy
     */
    virtual void growLimits(const Eigen::Vector2d & _point);

    virtual std::unique_ptr<Grid2D> computeCroppedGrid() const = 0;

protected:
    void growLimits(const Eigen::Vector2d & _point, const std::vector<std::vector<uint16_t> *> & _grids,
                const std::vector<uint16_t> & _gridsUnknownCellValues);

    const std::vector<uint16_t> & correspondenceCostCells() const {
        return correspondenceCostCells_;
    }

    const std::vector<int> & updateIndices() const { return updateIndices_; }

    const Eigen::AlignedBox2i & knownCellsBox() const { return knownCellsBox_; }

    std::vector<uint16_t> * mutableCorrespondenceCostCells() {
        return & correspondenceCostCells_;
    }

    std::vector<int> * mutableUpdateIndices() { return & updateIndices_; }

    Eigen::AlignedBox2i * mutableKnownCellsBox() { return & knownCellsBox_; }

    int toFlatIndex(const Eigen::Array2i & _cellIndex) const {
        return limits_.cellLimits().numXcells * _cellIndex.y() + _cellIndex.x();
    }

private:
    MapLimits limits_;
    std::vector<uint16_t> correspondenceCostCells_;
    double minCorrespondenceCost_;
    double maxCorrespondenceCost_;
    std::vector<int> updateIndices_;

    // Bounding box of known cells to efficiently compute cropping limits.
    Eigen::AlignedBox2i knownCellsBox_;
    const std::vector<double> * valueToCorrespondenceCostTable_;
};

}   // namespace Map
}   // namespace VISFS

#endif  // VISFS_MAP_2D_GRID2D_H_