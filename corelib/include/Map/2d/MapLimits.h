#ifndef VISFS_MAP_2D_MAPLIMITS_H_
#define VISFS_MAP_2D_MAPLIMITS_H_

#include "Map/2d/xyIndex.h"

#include <cmath>
#include <Eigen/Core>

namespace VISFS {
namespace Map {

class MapLimits {
public:
	MapLimits(const double _resolution, const Eigen::Vector2d & _max, const CellLimits & _cellLimits) :
		resolution_(_resolution), max_(_max), cellLimits_(_cellLimits) {
		assert(resolution_ > 0.0);
		assert(cellLimits_.numXcells > 0);
		assert(cellLimits_.numYcells > 0);
	}

	/** \brief Returns the cell size in meters. All cells are square and the resolution is the length of one side.
	 * \return The resolution.
	 * \author eddy
	 */
	double resolution() const { return resolution_; }

	/** \brief Returns the corner of limits, i.e., all pixels have positions with small coordinates.
	 * \return The limits.
	 * \author eddy
	 */
	const Eigen::Vector2d & max() const { return max_; }

	/** \brief Returns the limits of the grid in number of cells. 
	 * \return The limits.
	 * \author eddy
	 */
	const CellLimits & cellLimits() const { return cellLimits_; }

	/** \brief Returns the index of the cells containing the 'point'.
	 * \return The index.
	 * \author eddy
	 */
	Eigen::Array2i getCellIndex(const Eigen::Vector2d & _point) const {
		// Index values are row major and the top left has Eigen::Array2i::Zero()
		// and contains (centered_max_x, centered_max_y). We need to flip and
		// rotate.
		return Eigen::Array2i(
			std::lround((max_.y() - _point.y())/resolution_ - 0.5),
			std::lround((max_.x() - _point.x())/resolution_ - 0.5));
	}

	/** \brief Returns the center of the cell at 'index'.
	 * \return The center of the cell.
	 * \author eddy
	 */
	Eigen::Vector2d getCellCenter(const Eigen::Array2i & _index) const {
		return Eigen::Vector2d(
			max_.x() - resolution() * (_index[1] + 0.5),
			max_.y() - resolution() * (_index[0] + 0.5));
	}

	/** \brief Check whether the index in ProbabilityGrid.
	 * \return True: ProbabilityGrid contain index, False: doesn't contain.
	 * \author eddy
	 */	
	bool contains(const Eigen::Array2i & _index) const {
		return (Eigen::Array2i(0, 0) <= _index).all() &&
				(_index < Eigen::Array2i(cellLimits_.numXcells, cellLimits_.numYcells)).all();
	}

	
private:
    double resolution_;
	Eigen::Vector2d max_;
	CellLimits cellLimits_;

};

}   // namespace Map
}   // namespace VISFS

#endif