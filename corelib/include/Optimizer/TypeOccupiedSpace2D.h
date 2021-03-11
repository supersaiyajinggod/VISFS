#ifndef VISFS_OPTIMIZER_TPYE_OCCUPIED_SPACE_2D_H
#define VISFS_OPTIMIZER_TPYE_OCCUPIED_SPACE_2D_H

#include <memory>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/auto_differentiation.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <EXTERNAL/ceres/autodiff.h>
#include <ceres/cubic_interpolation.h>

#include "Sensor/PointCloud.h"
#include "Map/2d/Grid2d.h"
#include "Map/ProbabilityValues.h"

namespace VISFS {
namespace Optimizer {

constexpr int kPadding = INT_MAX / 4;

class GridArrayAdapter {
public:
    enum {DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const Map::Grid2D & _grid) : grid_(_grid) {}

    void GetValue(const int row, const int column, double* const value) const {
        if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
            column >= NumCols() - kPadding) {
            *value = Map::kMaxCorrespondenceCost;
      } else {
        *value = static_cast<double>(grid_.getCorrespondenceCost(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    int NumRows() const {
      return grid_.limits().cellLimits().numYcells + 2 * kPadding;
    }

    int NumCols() const {
      return grid_.limits().cellLimits().numXcells + 2 * kPadding;
    }

private:
    const Map::Grid2D & grid_;
};

class VertexPoint3D : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexPoint3D() {}

	virtual bool read(std::istream &) {
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	virtual bool write(std::ostream &) const {
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	virtual void setToOriginImpl() {
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
	}

	virtual void oplusImpl(const double * update) {
		Eigen::Vector3d::ConstMapType v(update);
		_estimate += v;
	}
};

class EdgeOccupiedObservation : public g2o::BaseBinaryEdge<1, double, g2o::VertexSE3Expmap, VertexPoint3D> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeOccupiedObservation(const std::shared_ptr<ceres::BiCubicInterpolator<GridArrayAdapter>> & _interpolator, Map::MapLimits & _limits) :
		interpolator_(_interpolator),
		limits_(_limits) {}

	virtual bool read(std::istream &) {
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	virtual bool write(std::ostream &) {
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	template<typename T>
	bool operator()(const T * _pose, const T * _point, T * _error) const {
		// _pose: t1 t2 t3 x y z w
		// _point: x y z
		const Eigen::Quaterniond q(_pose[6], _pose[3], _pose[4], _pose[5]);
		const Eigen::Vector3d t(_pose[0], _pose[1], _pose[2]);
		Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();
		Tcw.prerotate(q);
		Tcw.pretranslate(t);
		const Eigen::Isometry3d Twc = Tcw.inverse();
		const Eigen::Vector3d Pc(_point[0], _point[1], _point[2]);
		const Eigen::Vector3d Pw = Twc * Pc;

		interpolator_->Evaluate(
			(limits_.max().x() - Pw[0]) / limits_.resolution() - 0.5 + static_cast<double>(kPadding),
			(limits_.max().y() - Pw[1]) / limits_.resolution() - 0.5 + static_cast<double>(kPadding),
			_error
		);

		return true;
	}

	G2O_MAKE_AUTO_AD_FUNCTIONS

private:
	const std::shared_ptr<ceres::BiCubicInterpolator<GridArrayAdapter>> interpolator_;
	const Map::MapLimits limits_;

};

}   // Optimizer
}   // VISFS

#endif  // VISFS_OPTIMIZER_TPYE_OCCUPIED_SPACE_2D_H