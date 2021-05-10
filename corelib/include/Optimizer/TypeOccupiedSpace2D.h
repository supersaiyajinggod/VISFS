#ifndef VISFS_OPTIMIZER_TPYE_OCCUPIED_SPACE_2D_H
#define VISFS_OPTIMIZER_TPYE_OCCUPIED_SPACE_2D_H

#include <memory>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/auto_differentiation.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/EXTERNAL/ceres/autodiff.h>
#include <ceres/cubic_interpolation.h>

#include "Sensor/PointCloud.h"
#include "Map/2d/Grid2d.h"
#include "Map/ProbabilityValues.h"
#include "Optimizer/OptimizeTypeDefine.h"

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

class EdgeOccupiedObservation : public g2o::BaseBinaryEdge<1, double, VertexPose, VertexPoint3D> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeOccupiedObservation(const std::shared_ptr<ceres::BiCubicInterpolator<GridArrayAdapter>> & _interpolator, const std::shared_ptr<Map::MapLimits> & _limits,
		const Eigen::Isometry3d _transformImageToRobot) :
		interpolator_(_interpolator),
		limits_(_limits),
		transformImageToRobot_(_transformImageToRobot) {}

	virtual bool read(std::istream &) {
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	virtual bool write(std::ostream &) const {
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	template<typename T>
	bool operator()(const T * _pose, const T * _point, T * _error) const {
		// _pose: t1 t2 t3 x y z w
		// _point: x y z
		typename g2o::VectorN<7, T>::ConstMapType pose(_pose);
		typename g2o::VectorN<3, T>::ConstMapType point(_point);
		typename g2o::VectorN<1, T>::MapType error(_error);

		const Eigen::Quaternion<T> q(pose[6], pose[3], pose[4], pose[5]);
		const Eigen::Matrix<T, 3, 1> t(pose[0], pose[1], pose[2]);
		Eigen::Transform<T, 3, 1> Tiw = Eigen::Transform<T, 3, 1>::Identity();
		Tiw.prerotate(q);
		Tiw.pretranslate(t);
		auto Twc = Tiw.inverse() * transformImageToRobot_.inverse().cast<T>();
		// auto Two = submapOrigin_.cast<T>();
		// auto Toc = Two.inverse() * Twc;
		const Eigen::Matrix<T, 3, 1> Pc(point[0], point[1], point[2]);
		Eigen::Matrix<T, 3, 1> Po = Twc * Pc;

		interpolator_->Evaluate(
			(limits_->max().x() - Po[0]) / limits_->resolution() - 0.5 + static_cast<double>(kPadding),
			(limits_->max().y() - Po[1]) / limits_->resolution() - 0.5 + static_cast<double>(kPadding),
			&error[0]
		);

		(void)error;
		return true;
	}

	virtual void computeError() override {
		const VertexPose * Tcw = dynamic_cast<const VertexPose *>(vertex(0));
		const VertexPoint3D * point = dynamic_cast<const VertexPoint3D *>(vertex(1));

		(*this)(Tcw->estimate().toVector().data(), point->estimate().data(), _error.data());
	}

	template <int EdgeDimension, int VertexDimension>
	using ADJacobianType = typename Eigen::Matrix<number_t, EdgeDimension, VertexDimension, Eigen::RowMajor>;

	//! helper function to perform a = b
	template <typename A, typename B>
	static EIGEN_STRONG_INLINE void assign(const Eigen::MatrixBase<A>& a,
											const Eigen::MatrixBase<B>& b) {
		Eigen::MatrixBase<A>& aux = const_cast<Eigen::MatrixBase<A>&>(a);
		aux = b;
	}

	virtual void linearizeOplus() override {
		const g2o::VertexSE3Expmap * Tcw = dynamic_cast<const g2o::VertexSE3Expmap *>(vertex(0));
		const VertexPoint3D * point = dynamic_cast<const VertexPoint3D *>(vertex(1));

		if (this->allVerticesFixed()) {
			int unused[] = {(this->jacobianOplusXn<0>().setZero(), 0), (this->jacobianOplusXn<1>().setZero(), 0)};
			(void)unused;
			return;
		}

		std::tuple<ADJacobianType<EdgeOccupiedObservation::Dimension, VertexXnType<0>::Dimension>, ADJacobianType<this->Dimension, VertexXnType<1>::Dimension>> adJacobians;

		number_t * parameters[] = { const_cast<number_t *>(Tcw->estimate().toVector().data()), const_cast<number_t *>(point->estimate().data()) };
		number_t * jacobians[] = { this->vertexXn<0>()->fixed() ? nullptr : const_cast<number_t *>(std::get<0>(adJacobians).data()),
									this->vertexXn<1>()->fixed() ? nullptr : const_cast<number_t *>(std::get<1>(adJacobians).data()) };
		number_t errorValue[EdgeOccupiedObservation::Dimension];

		using AutoDiffDims = ceres::internal::StaticParameterDims<VertexXnType<0>::Dimension, VertexXnType<1>::Dimension>;
		bool diffState = ceres::internal::AutoDifferentiate<EdgeOccupiedObservation::Dimension, AutoDiffDims, EdgeOccupiedObservation, number_t>(
				*this, parameters, EdgeOccupiedObservation::Dimension, errorValue, jacobians);

		assert(diffState && "Error during Automatic Differentiation");
		if (!diffState) {
			int unused[] = {(this->jacobianOplusXn<0>().setZero(), 0), (this->jacobianOplusXn<1>().setZero(), 0)};
			(void)unused;
			return;
		}

		// copy over the Jacobians (convert row-major -> column-major) for non-fixed vertices
		int unused[] = {
			(this->vertexXn<0>()->fixed() ? (this->jacobianOplusXn<0>().setZero(), 0) : (assign(this->jacobianOplusXn<0>(), std::get<0>(adJacobians)), 0)),
			(this->vertexXn<1>()->fixed() ? (this->jacobianOplusXn<1>().setZero(), 0) : (assign(this->jacobianOplusXn<1>(), std::get<1>(adJacobians)), 0))
		};
		(void)unused;
	}


private:
	const std::shared_ptr<ceres::BiCubicInterpolator<GridArrayAdapter>> interpolator_;
	const std::shared_ptr<Map::MapLimits> limits_;
	const Eigen::Isometry3d transformImageToRobot_;	// Trc

};

// class EdgeOccupiedObservation : public g2o::BaseBinaryEdge<1, double, g2o::VertexSE3Expmap, VertexPoint3D> {
// public:
// 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// 	EdgeOccupiedObservation(const std::shared_ptr<ceres::BiCubicInterpolator<GridArrayAdapter>> & _interpolator, const std::shared_ptr<Map::MapLimits> & _limits,
// 		const Eigen::Isometry3d _transformImageToRobot) :
// 		interpolator_(_interpolator),
// 		limits_(_limits),
// 		transformImageToRobot_(_transformImageToRobot) {}

// 	virtual bool read(std::istream &) {
// 		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
// 		return false;
// 	}

// 	virtual bool write(std::ostream &) const {
// 		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
// 		return false;
// 	}

// 	template<typename T>
// 	bool operator()(const T * _pose, const T * _point, T * _error) const {
// 		// _pose: t1 t2 t3 x y z w
// 		// _point: x y z
// 		typename g2o::VectorN<7, T>::ConstMapType pose(_pose);
// 		typename g2o::VectorN<3, T>::ConstMapType point(_point);
// 		typename g2o::VectorN<1, T>::MapType error(_error);

// 		const Eigen::Quaternion<T> q(pose[6], pose[3], pose[4], pose[5]);
// 		const Eigen::Matrix<T, 3, 1> t(pose[0], pose[1], pose[2]);
// 		Eigen::Transform<T, 3, 1> Tiw = Eigen::Transform<T, 3, 1>::Identity();
// 		Tiw.prerotate(q);
// 		Tiw.pretranslate(t);
// 		auto Twc = Tiw.inverse() * transformImageToRobot_.inverse().cast<T>();
// 		// auto Two = submapOrigin_.cast<T>();
// 		// auto Toc = Two.inverse() * Twc;
// 		const Eigen::Matrix<T, 3, 1> Pc(point[0], point[1], point[2]);
// 		Eigen::Matrix<T, 3, 1> Po = Twc * Pc;

// 		interpolator_->Evaluate(
// 			(limits_->max().x() - Po[0]) / limits_->resolution() - 0.5 + static_cast<double>(kPadding),
// 			(limits_->max().y() - Po[1]) / limits_->resolution() - 0.5 + static_cast<double>(kPadding),
// 			&error[0]
// 		);

// 		(void)error;
// 		return true;
// 	}

// 	virtual void computeError() override {
// 		const g2o::VertexSE3Expmap * Tcw = dynamic_cast<const g2o::VertexSE3Expmap *>(vertex(0));
// 		const VertexPoint3D * point = dynamic_cast<const VertexPoint3D *>(vertex(1));

// 		(*this)(Tcw->estimate().toVector().data(), point->estimate().data(), _error.data());
// 	}

// 	template <int EdgeDimension, int VertexDimension>
// 	using ADJacobianType = typename Eigen::Matrix<number_t, EdgeDimension, VertexDimension, Eigen::RowMajor>;

// 	//! helper function to perform a = b
// 	template <typename A, typename B>
// 	static EIGEN_STRONG_INLINE void assign(const Eigen::MatrixBase<A>& a,
// 											const Eigen::MatrixBase<B>& b) {
// 		Eigen::MatrixBase<A>& aux = const_cast<Eigen::MatrixBase<A>&>(a);
// 		aux = b;
// 	}

// 	virtual void linearizeOplus() override {
// 		const g2o::VertexSE3Expmap * Tcw = dynamic_cast<const g2o::VertexSE3Expmap *>(vertex(0));
// 		const VertexPoint3D * point = dynamic_cast<const VertexPoint3D *>(vertex(1));

// 		if (this->allVerticesFixed()) {
// 			int unused[] = {(this->jacobianOplusXn<0>().setZero(), 0), (this->jacobianOplusXn<1>().setZero(), 0)};
// 			(void)unused;
// 			return;
// 		}

// 		std::tuple<ADJacobianType<EdgeOccupiedObservation::Dimension, VertexXnType<0>::Dimension>, ADJacobianType<this->Dimension, VertexXnType<1>::Dimension>> adJacobians;

// 		number_t * parameters[] = { const_cast<number_t *>(Tcw->estimate().toVector().data()), const_cast<number_t *>(point->estimate().data()) };
// 		number_t * jacobians[] = { this->vertexXn<0>()->fixed() ? nullptr : const_cast<number_t *>(std::get<0>(adJacobians).data()),
// 									this->vertexXn<1>()->fixed() ? nullptr : const_cast<number_t *>(std::get<1>(adJacobians).data()) };
// 		number_t errorValue[EdgeOccupiedObservation::Dimension];

// 		using AutoDiffDims = ceres::internal::StaticParameterDims<VertexXnType<0>::Dimension, VertexXnType<1>::Dimension>;
// 		bool diffState = ceres::internal::AutoDifferentiate<EdgeOccupiedObservation::Dimension, AutoDiffDims, EdgeOccupiedObservation, number_t>(
// 				*this, parameters, EdgeOccupiedObservation::Dimension, errorValue, jacobians);

// 		assert(diffState && "Error during Automatic Differentiation");
// 		if (!diffState) {
// 			int unused[] = {(this->jacobianOplusXn<0>().setZero(), 0), (this->jacobianOplusXn<1>().setZero(), 0)};
// 			(void)unused;
// 			return;
// 		}

// 		// copy over the Jacobians (convert row-major -> column-major) for non-fixed vertices
// 		int unused[] = {
// 			(this->vertexXn<0>()->fixed() ? (this->jacobianOplusXn<0>().setZero(), 0) : (assign(this->jacobianOplusXn<0>(), std::get<0>(adJacobians)), 0)),
// 			(this->vertexXn<1>()->fixed() ? (this->jacobianOplusXn<1>().setZero(), 0) : (assign(this->jacobianOplusXn<1>(), std::get<1>(adJacobians)), 0))
// 		};
// 		(void)unused;
// 	}


// private:
// 	const std::shared_ptr<ceres::BiCubicInterpolator<GridArrayAdapter>> interpolator_;
// 	const std::shared_ptr<Map::MapLimits> limits_;
// 	const Eigen::Isometry3d transformImageToRobot_;	// Trc

// };

}   // Optimizer
}   // VISFS

#endif  // VISFS_OPTIMIZER_TPYE_OCCUPIED_SPACE_2D_H