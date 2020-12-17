#include "OptimizeTypeDefine.h"
#include "Math.h"

namespace VISFS {

VertexSE3::VertexSE3() : BaseVertex<6, g2o::SE3Quat>() {
}

bool VertexSE3::read(std::istream& is) {
    return true;
}

bool VertexSE3::write(std::ostream& os) const {
  return true;
}    

bool EdgeSE3Expmap::read(std::istream & is) {
	return false;
}

bool EdgeSE3Expmap::write(std::ostream & os) const {
	return false;
}

void EdgeSE3Expmap::linearizeOplus() {
	const VertexSE3 * v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
	const VertexSE3 * v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);		
	// Extract Pci,w Qci,w Pci+1,w Qci+1,w
	const Eigen::Vector3d sP1 = v1->estimate().translation();
	const Eigen::Quaterniond sQ1 = v1->estimate().rotation();
	const Eigen::Vector3d sP2 = v2->estimate().translation();
	const Eigen::Quaterniond sQ2 = v2->estimate().rotation();
	const Eigen::Vector3d mP12 = _measurement.translation();
	const Eigen::Quaterniond mQ12 = _measurement.rotation();

	_jacobianOplusXi.setZero();
	_jacobianOplusXi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
	_jacobianOplusXi.block<3, 3>(0, 3) = -skewSymmetric(sQ1*(sQ2.inverse()*(-sP2)));
	_jacobianOplusXi.block<3, 3>(3, 3) = (QuaternionLeft(sQ2)*QuaternionRight(sQ1.inverse()*mQ12)).bottomRightCorner<3, 3>();
	
	_jacobianOplusXj.setZero();
	_jacobianOplusXj.block<3, 3>(0, 0) = -(sQ1*sQ2.inverse()).toRotationMatrix();
	_jacobianOplusXj.block<3, 3>(0, 3) = sQ1.toRotationMatrix()*skewSymmetric(sQ2.inverse()*(-sP2));
	_jacobianOplusXj.block<3, 3>(3, 3) = -(QuaternionLeft(sQ2)*QuaternionRight(sQ1.inverse()*mQ12)).bottomRightCorner<3, 3>();

	// std::cout << "_jacobianOplusXi: \n" << _jacobianOplusXi.matrix() << std::endl;
	// std::cout << "_jacobianOplusXj: \n" << _jacobianOplusXj.matrix() << std::endl;

}

}