#ifndef VISFS_OPTIMIZER_TYPE_DEFINE_H
#define VISFS_OPTIMIZER_TYPE_DEFINE_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/eigen_types.h>
#include <g2o/types/slam3d/se3quat.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace VISFS {
namespace Optimizer {

/**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */
class VertexSE3 : public g2o::BaseVertex<6, g2o::SE3Quat> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VertexSE3();

	bool read(std::istream& is);

	bool write(std::ostream& os) const;

	virtual void setToOriginImpl() {
		_estimate = g2o::SE3Quat();
	}

	virtual void oplusImpl(const number_t* update_)  {
		std::cout << "estimate before update: " << estimate().to_homogeneous_matrix() << std::endl; 
		Eigen::Map<const g2o::Vector6> update(update_);
		auto temp = estimate().toVector();
		g2o::Vector3 trans(temp[0]+update[0], temp[1]+update[1], temp[2]+update[2]);
		g2o::Quaternion rotate = g2o::Quaternion(0, update[3], update[4], update[5])*g2o::Quaternion(temp[6], temp[3], temp[4], temp[5]);
		// setEstimate(g2o::SE3Quat::exp(update)*estimate());
		setEstimate(g2o::SE3Quat(rotate, trans));
		std::cout << "estimate after update: " << estimate().to_homogeneous_matrix() << std::endl; 
	}
};

// Derivation from Tcw
class EdgeSE3Expmap : public g2o::BaseBinaryEdge<6, g2o::SE3Quat, VertexSE3, VertexSE3> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	EdgeSE3Expmap() : BaseBinaryEdge<6, g2o::SE3Quat, VertexSE3, VertexSE3>() {}

	bool read(std::istream & is);

	bool write(std::ostream & os) const;

	void computeError() {
		const VertexSE3 * v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
		const VertexSE3 * v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);
		// Extract Pci,w Qci,w Pci+1,w Qci+1,w
		const Eigen::Vector3d sP1 = v1->estimate().translation();
		const Eigen::Quaterniond sQ1 = v1->estimate().rotation();
		const Eigen::Vector3d sP2 = v2->estimate().translation();
		const Eigen::Quaterniond sQ2 = v2->estimate().rotation();
		const Eigen::Vector3d mP12 = _measurement.translation();
		const Eigen::Quaterniond mQ12 = _measurement.rotation();

		_error.block<3, 1>(0, 0) = sQ1*sQ2.inverse()*(-sP2) + sP1 - mP12;
		_error.block<3, 1>(3, 0) = 2 * (mQ12.inverse()*sQ1*sQ2.inverse()).vec();
		std::cout << "computeError: " << _error.matrix().transpose() << std::endl;
	}

	virtual void setMeasurement(const g2o::SE3Quat & measurement) {
		_measurement = measurement;
	}

	virtual double initialEstimatePossible(const g2o::OptimizableGraph &, g2o::OptimizableGraph::Vertex *) { return 1.0; }

	virtual bool setMeasurementData(const double * d) {
		Eigen::Map<const g2o::Vector7> v(d);
		_measurement.fromVector(v);
		return true;
	}

	virtual bool getMeasurementData(double * d) const {
		Eigen::Map<g2o::Vector7> v(d);
		v = _measurement.toVector();
		return true;
	}

	virtual int measurementDimension() const { return 7; }

	void linearizeOplus();

};

// // Derivation from Twc
// class G2O_TYPES_SBA_API EdgeSE3Expmap : public g2o::BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap> {
// public:
// 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// 	EdgeSE3Expmap() : BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>() {}

// 	bool read(std::istream & is) {
// 		return false;
// 	}

// 	bool write(std::ostream & os) const {
// 		return false;
// 	}

// 	void computeError() {
// 		const g2o::VertexSE3Expmap * v1 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
// 		const g2o::VertexSE3Expmap * v2 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
// 		// Extract Pwci Qwci Pwci+1 Qwci+1
// 		const Eigen::Vector3d sP1 = v1->estimate().translation();
// 		const Eigen::Quaterniond sQ1 = v1->estimate().rotation();
// 		const Eigen::Vector3d sP2 = v2->estimate().translation();
// 		const Eigen::Quaterniond sQ2 = v2->estimate().rotation();
// 		const Eigen::Vector3d mP12 = _measurement.translation();
// 		const Eigen::Quaterniond mQ12 = _measurement.rotation();

// 		_error.block<3, 1>(0, 0) = sQ1.inverse()*(sP2 - sP1) - mP12;
// 		_error.block<3, 1>(3, 0) = 2 * (mQ12.inverse()*sQ1.inverse()*sQ2).vec();
// 	}

// 	virtual void setMeasurement(const g2o::SE3Quat & measurement) {
// 		_measurement = measurement;
// 	}

// 	virtual double initialEstimatePossible(const g2o::OptimizableGraph &, g2o::OptimizableGraph::Vertex *) { return 1.0; }

// 	virtual bool setMeasurementData(const double * d) {
// 		Eigen::Map<const g2o::Vector7> v(d);
// 		_measurement.fromVector(v);
// 		return true;
// 	}

// 	virtual bool getMeasurementData(double * d) const {
// 		Eigen::Map<g2o::Vector7> v(d);
// 		v = _measurement.toVector();
// 		return true;
// 	}

// 	virtual int measurementDimension() const { return 7; }

// 	virtual void linearizeOplus() {
// 		const g2o::VertexSE3Expmap * v1 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
// 		const g2o::VertexSE3Expmap * v2 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);		
// 		// Extract Pwci Qwci Pwci+1 Qwci+1
// 		const Eigen::Vector3d sP1 = v1->estimate().translation();
// 		const Eigen::Quaterniond sQ1 = v1->estimate().rotation();
// 		const Eigen::Vector3d sP2 = v2->estimate().translation();
// 		const Eigen::Quaterniond sQ2 = v2->estimate().rotation();
// 		const Eigen::Vector3d mP12 = _measurement.translation();
// 		const Eigen::Quaterniond mQ12 = _measurement.rotation();

// 		_jacobianOplusXi.setZero();
// 		_jacobianOplusXi.block<3, 3>(0, 0) = -sQ1.inverse().toRotationMatrix();
// 		_jacobianOplusXi.block<3, 3>(0, 3) = skewSymmetric(sQ1.inverse() * (sP2 - sP1));
// 		_jacobianOplusXi.block<3, 3>(3, 3) = -(QuaternionLeft(sQ2.inverse()*sQ1)*QuaternionRight(mQ12)).bottomRightCorner<3, 3>();
		
// 		_jacobianOplusXj.setZero();
// 		_jacobianOplusXj.block<3, 3>(0, 0) = sQ1.inverse().toRotationMatrix();
// 		_jacobianOplusXi.block<3, 3>(3, 3) = QuaternionLeft(mQ12.inverse()*sQ1.inverse()*sQ2).bottomRightCorner<3, 3>();
// 	}

// };


// class G2O_TYPES_SBA_API EdgeStereoSE3PointXYZ : public g2o::BaseBinaryEdge<3, g2o::Vector3, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap> {
// public:
// 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// 	EdgeStereoSE3PointXYZ() : g2o::BaseBinaryEdge<3, g2o::Vector3, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {}

// 	bool read(std::istream & is) {
// 		// g2o::internal::readVector(is, _measurement);
// 		// return g2o::readInformationMatrix(is);
// 		return true;
// 	}

// 	bool write(std::ostream & os) const {
// 		// g2o::internal::writeVector(os, measurement());
// 		// return writeInformationMatrix(os);
// 		return true;
// 	}

// 	void computeError() {
// 		const g2o::VertexSE3Expmap * v1 = dynamic_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
// 		const g2o::VertexSBAPointXYZ * v2 = dynamic_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
// 		g2o::Vector3 obs(_measurement);
// 		_error = obs - cameraProject(v1->estimate().inverse().map(v2->estimate()));
// 	}

// 	bool isDepthPositive() {
// 		const g2o::VertexSE3Expmap * v1 = dynamic_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
// 		const g2o::VertexSBAPointXYZ * v2 = dynamic_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
// 		return (v1->estimate().inverse().map(v2->estimate()))(2) > 0;
// 	}

// 	void linearizeOplus() {
// 		const g2o::VertexSE3Expmap * vj = dynamic_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
// 		const g2o::VertexSBAPointXYZ * vi = dynamic_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
// 		g2o::SE3Quat Tcw(vj->estimate().inverse());
// 		g2o::Vector3 Pw = vi->estimate();
// 		g2o::Vector3 Pc = Tcw.map(Pw);
// 		const g2o::Matrix3 Rcw = Tcw.rotation().toRotationMatrix();

// 		number_t X = Pc[0];
// 		number_t Y = Pc[1];
// 		number_t Z = Pc[2];
// 		number_t Z2 = Z * Z;

// 		_jacobianOplusXi(0, 0) = -fx * Rcw(0, 0) / Z + fx * X * Rcw(2, 0) / Z2;
// 		_jacobianOplusXi(0, 1) = -fx * Rcw(0, 1) / Z + fx * X * Rcw(2, 1) / Z2;
// 		_jacobianOplusXi(0, 2) = -fx * Rcw(0, 2) / Z + fx * X * Rcw(2, 2) / Z2;

// 		_jacobianOplusXi(1, 0) = -fy * Rcw(1, 0) / Z + fy * Y * Rcw(2, 0) / Z2;
// 		_jacobianOplusXi(1, 1) = -fy * Rcw(1, 1) / Z + fy * Y * Rcw(2, 1) / Z2;
// 		_jacobianOplusXi(1, 2) = -fy * Rcw(1, 2) / Z + fy * Y * Rcw(2, 2) / Z2;

// 		_jacobianOplusXi(2, 0) = _jacobianOplusXi(0, 0) - bf * Rcw(2, 0) / Z2;
// 		_jacobianOplusXi(2, 1) = _jacobianOplusXi(0, 1) - bf * Rcw(2, 1) / Z2;
// 		_jacobianOplusXi(2, 2) = _jacobianOplusXi(0, 2) - bf * Rcw(2, 2) / Z2;

// 		_jacobianOplusXj(0, 0) = -1.0 * X * Y / Z2 * fx;
// 		_jacobianOplusXj(0, 1) = (1 + (X * X) / Z2) * fx;
// 		_jacobianOplusXj(0, 2) = -1.0 * Y / Z * fx;
// 		_jacobianOplusXj(0, 3) = fx / Z;
// 		_jacobianOplusXj(0, 4) = 0.0;
// 		_jacobianOplusXj(0, 5) = -1.0 * X / Z2 * fx;

// 		_jacobianOplusXj(1, 0) = -1.0 * (1 + Y * Y / Z2) * fy;
// 		_jacobianOplusXj(1, 1) = X * Y / Z2 * fy;
// 		_jacobianOplusXj(1, 2) = X / Z * fy;
// 		_jacobianOplusXj(1, 3) = 0.0;
// 		_jacobianOplusXj(1, 4) = fy / Z;
// 		_jacobianOplusXj(1, 5) = -1.0 * Y / Z2 * fy;

// 		_jacobianOplusXj(2, 0) = _jacobianOplusXj(0, 0) + bf * Y / Z2;
// 		_jacobianOplusXj(2, 1) = _jacobianOplusXj(0, 1) - bf * X / Z2;
// 		_jacobianOplusXj(2, 2) = _jacobianOplusXj(0, 2);
// 		_jacobianOplusXj(2, 3) = _jacobianOplusXj(0, 3);
// 		_jacobianOplusXj(2, 4) = 0.0;
// 		_jacobianOplusXj(2, 5) = _jacobianOplusXj(0, 5) + bf / Z2;
// 	}

// 	g2o::Vector3 cameraProject(const g2o::Vector3 & translation) const {
// 		const number_t invZ = 1.0 / translation[2];

// 		g2o::Vector3 res;
// 		res[0] = translation[0] * invZ * fx + cx;
// 		res[1] = translation[1] * invZ * fy + cy;
// 		res[2] = res[0] - bf * invZ;
// 		return res;
// 	}

// 	number_t fx, fy, cx, cy, bf;

// };

}	// Optimizer
}	// VISFS

#endif  // OPTIMIZER_TYPE_DEFINE_H