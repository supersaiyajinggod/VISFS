#ifndef VISFS_MAP_2D_PROBABILITY_GRID_H_
#define VISFS_MAP_2D_PROBABILITY_GRID_H_

#include "Map/2d/Grid2d.h"
#include "Map/2d/MapLimits.h"
#include "Map/2d/xyIndex.h"

namespace VISFS {
namespace Map {

class ProbabilityGrid : public Grid2D {
public:
    explicit ProbabilityGrid(const MapLimits & _limits, ValueConversionTables * _conversionTables);

    /** \brief Sets the probability of the cell to the given probability.
     * \param[in] cellIndex The cell want to set.
     * \param[in] probability The probability.
     * \author eddy
     */
    void setProbability(const Eigen::Array2i & _cellIndex, const double _probability);

    /** \brief Get the probability of the cell.
     * \return The probability.
     * \author eddy
     */
    double getProbability(const Eigen::Array2i & _cellIndex) const;

    /** \brief Apply the 'odds' specified when calling computeLookupTableToApplyOdds() to the probability of the cell,
     *          if the cell has not already been updated.Multiple updates of the same cell will be ignored until FinishUpdate() is called.
     * \param[in] cellIndex The cell want to set.
     * \param[in] table The table.
     * \return True: if the cell was updated.
     * \author eddy
     */
    bool applyLookUpTable(const Eigen::Array2i & _cellIndex, const std::vector<uint16_t> & _table);

    GridType getGridType() const override;

    std::unique_ptr<Grid2D> computeCroppedGrid() const override;

private:
    ValueConversionTables * conversionTables_;
};

}   // Map    
}   // VISFS

#endif  // VISFS_MAP_2D_PROBABILITY_GRID_H_