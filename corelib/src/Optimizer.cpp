#include "Optimizer.h"
#include "Stl.h"

#include <g2o/config.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#ifdef G2O_HAVE_CSPARSE
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#endif
#ifdef G2O_HAVE_CHOLMOD
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#endif

namespace VISFS {

Optimizer::Optimizer(const ParametersMap & _parameters) :
	iterations_(Parameters::defaultOptimizerIterations()),
	solver_(Parameters::defaultOptimizerSolver()),
	optimizer_(Parameters::defaultOptimizerOptimizer()),
	pixelVariance_(Parameters::defaultOptimizerPixelVariance()),
	robustKernelDelta_(Parameters::defaultOptimizerRobustKernelDelta()) {
	
	Parameters::parse(_parameters, Parameters::kOptimizerIterations(), iterations_);
	Parameters::parse(_parameters, Parameters::kOptimizerSolver(), solver_);
	Parameters::parse(_parameters, Parameters::kOptimizerOptimizer(), optimizer_);
	Parameters::parse(_parameters, Parameters::kOptimizerPixelVariance(), pixelVariance_);
	Parameters::parse(_parameters, Parameters::kOptimizerRobustKernelDelta(), robustKernelDelta_);

}

// class EdgeSE3Expmap : public g2o::BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap> {
// public:
// 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
// 	EdgeSE3Expmap() : BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>(){}

// 	bool read(std::istream & is) {
// 		return false;
// 	}

// 	bool write(std::ostream & os) const {
// 		return false;
// 	}

// 	void computeError() {
// 		const g2o::VertexSE3Expmap * v1 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
// 		const g2o::VertexSE3Expmap * v2 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
// 		g2o::SE3Quat delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
// 		_error[0] = delta.translation().x();
// 		_error[1] = delta.translation().y();
// 		_error[2] = delta.translation().z();
// 		_error[3] = delta.rotation().x();
// 		_error[4] = delta.rotation().y();
// 		_error[5] = delta.rotation().z();
// 	}

// 	virtual void setMeasurement(const g2o::SE3Quat & meas) {
// 		_measurement = meas;
// 		_inverseMeasurement = meas.inverse();
// 	}

// 	virtual double initialEstimatePossible(const g2o::OptimizableGraph &, g2o::OptimizableGraph::Vertex *) { return 1.0; }

// 	virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet & from_, g2o::OptimizableGraph::Vertex *) {
// 		g2o::VertexSE3Expmap * from = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
// 		g2o::VertexSE3Expmap * to = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
// 		if (from_.count(from) > 0) {
// 			to->setEstimate(static_cast<g2o::SE3Quat>(from->estimate())*_measurement);
// 		} else {
// 			from->setEstimate(static_cast<g2o::SE3Quat>(to->estimate())*_inverseMeasurement);
// 		}
// 	}

// 	virtual bool setMeasurementData(const double * d) {
// 		Eigen::Map<const g2o::Vector7> v(d);
// 		_measurement.fromVector(v);
// 		_inverseMeasurement = _measurement.inverse();
// 		return true;
// 	}

// 	virtual bool getMeasurementData(double * d) const {
// 		Eigen::Map<g2o::Vector7> v(d);
// 		v = _measurement.toVector();
// 		return true;
// 	}

// 	virtual int measurementDimension() const { return 7; }

// 	virtual bool setMeasurementFromState() {
// 		const g2o::VertexSE3Expmap * v1 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
// 		const g2o::VertexSE3Expmap * v2 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
// 		_measurement = v1->estimate().inverse()*v2->estimate();
// 		_inverseMeasurement = _measurement.inverse();
// 		return true;		
// 	}

// protected:
// 	g2o::SE3Quat _inverseMeasurement;

// };

std::map<std::size_t, Eigen::Isometry3d> Optimizer::poseOptimize(
    std::size_t rootId,     // fixed pose
    const std::map<std::size_t, Eigen::Isometry3d> & _poses,    // map<pose index, transform>
    const std::map<std::size_t,std::tuple<std::size_t, std::size_t, Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> & _links,  // map<link index, tuple<the from pose index, the to pose index, transform, infomation matrix>>
    const std::vector<boost::shared_ptr<GeometricCamera>> & _cameraModels, // vector camera model left and right
    std::map<std::size_t, Eigen::Vector3d> & _points3D,
    const std::map<std::size_t, std::map<std::size_t, FeatureBA>> & _wordReferences,
    std::set<std::size_t> & _outliers) {

	assert(_cameraModels.size() >= 1);
	std::map<std::size_t, Eigen::Isometry3d> optimizedPoses;
	if (_poses.size() >= 2 && iterations_ > 0 && _poses.begin()->first > 0) {
		g2o::SparseOptimizer optimizer;
		std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
		if (solver_ == 3) {
			linearSolver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
		} else if (solver_ == 2) {
			linearSolver = g2o::make_unique<g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>>();
		}
#ifdef G2O_HAVE_CHOLMOD
		else if (solver_ == 1) {
			linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
		}
#endif
#ifdef G2O_HAVE_CSPARSE
		else if (solver_ == 0) {
			linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>>();
		}
#endif

		if (optimizer_ == 0) {
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))));
		} else if (solver_ == 1) {
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))));
		}

		// Set poses to g2o
		for (auto iter = _poses.begin(); iter != _poses.end(); ++iter) {
			if (iter->first > 0) {
				const GeometricCamera & cameraModel = *_cameraModels.front();
				// Twi = Twr * Tri
				Eigen::Isometry3d cameraPose = iter->second * cameraModel.getTansformImageToRobot();

				g2o::VertexCam * vCam = new g2o::VertexCam();
				g2o::SBACam cam(Eigen::Quaterniond(cameraPose.linear()), cameraPose.translation());
				Eigen::Matrix3d K = cameraModel.eigenKdouble();
				cam.setKcam(K(0, 0), K(1, 1), K(0, 2), K(1, 2), cameraModel.getBaseLine());
				vCam->setEstimate(cam);
				vCam->setId(iter->first);
				vCam->setFixed(rootId >= 0 && iter->first == rootId);
				optimizer.addVertex(vCam);
			}
		}

		// Set edges to g2o
		for (auto iter = _links.begin(); iter != _links.end(); ++iter) {
			std::size_t fromId;
			std::size_t toId;
			Eigen::Isometry3d transfrom;
			Eigen::Matrix<double, 6, 6> information;
			std::tie(fromId, toId, transfrom, information) = iter->second;

			if (fromId > 0 && toId > 0 && uContains(_poses, fromId) && uContains(_poses, toId)) {
				if (fromId == toId) {
					// TODO
				} else {
					const GeometricCamera & cameraModel = *_cameraModels.front();
					Eigen::Isometry3d Tri = cameraModel.getTansformImageToRobot();
					// Ti1i2 = Tir * Tr1r2 * Tri
					Eigen::Isometry3d Ti1i2 = Tri.inverse()*transfrom*Tri;

					g2o::EdgeSBACam * e = new g2o::EdgeSBACam();
					g2o::VertexCam * v1 = dynamic_cast<g2o::VertexCam *>(optimizer.vertex(fromId));
					g2o::VertexCam * v2 = dynamic_cast<g2o::VertexCam *>(optimizer.vertex(toId));

					// EdgeSE3Expmap * e = new EdgeSE3Expmap();
					// g2o::VertexSE3Expmap * v1 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(fromId));
					// g2o::VertexSE3Expmap * v2 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(toId));
					// Eigen::Isometry3d Tw1i1(v1->estimate().inverse().to_homogeneous_matrix());
					// Eigen::Isometry3d Ti2w2(v2->estimate().to_homogeneous_matrix());
					// Eigen::Isometry3d cameraLink = Tw1i1*Tw1i1*Ti1i2*Ti2w2*Ti2w2;

					Eigen::Isometry3d cameraLink = Ti1i2;
					e->setVertex(0, v1);
					e->setVertex(1, v2);
					e->setMeasurement(g2o::SE3Quat(cameraLink.linear(), cameraLink.translation()));
					e->setInformation(information);

					if (!optimizer.addEdge(e)) {
						delete e;
						std::cout << "Optimizer: Failed adding constraint between " << fromId << " and " << toId << ", skipping." << std::endl;
						return optimizedPoses;
					}
				}

			}
		}

		// Set 3D points to g2o
		const int stepVertexId = static_cast<int>(_poses.rbegin()->first + 1);
		int negVertexOffset = stepVertexId;
		if (_wordReferences.size() && _wordReferences.rbegin()->first > 0) {
			negVertexOffset += _wordReferences.rbegin()->first;
		}
		std::list<g2o::OptimizableGraph::Edge *> edges;
		for (auto iter = _wordReferences.begin(); iter != _wordReferences.end(); ++iter) {
			int id = iter->first;
			if (_points3D.find(id) != _points3D.end()) {
				// Set vertex
				g2o::VertexSBAPointXYZ* vpt3d = new g2o::VertexSBAPointXYZ();
				vpt3d->setEstimate(_points3D.at(id));
				vpt3d->setId(stepVertexId + id);
				vpt3d->setMarginalized(true);
				optimizer.addVertex(vpt3d);

				// Set edge
				for (auto jter = iter->second.begin(); jter != iter->second.end(); ++jter) {
					const GeometricCamera & cameraModel = *_cameraModels.front();
					int cameraId = jter->first;
					if (_poses.find(cameraId) != _poses.end() && optimizer.vertex(cameraId) != 0) {
						const FeatureBA & pt = jter->second;
						double depth = pt.depth;
						double baseLine = 0.0;
						// std::cout << "Add observation pt: " << vpt3d->id()-stepVertexId << " to cam= " << cameraId << " with " << pt.kpt.pt.x << " " << pt.kpt.pt.y << " depth= " << depth << std::endl;

						g2o::OptimizableGraph::Edge * e;
						g2o::VertexCam * vCam = dynamic_cast<g2o::VertexCam *>(optimizer.vertex(cameraId));
						if (_cameraModels.size() > 1) {
							baseLine = cameraModel.getBaseLine();
						}
						if (std::isfinite(depth) && depth > 0.0 && baseLine > 0.0) {
							// Stereo
							g2o::EdgeProjectP2SC * es = new g2o::EdgeProjectP2SC();
							float disparity = baseLine * vCam->estimate().Kcam(0, 0) / depth;
							Eigen::Vector3d obs(pt.kpt.pt.x, pt.kpt.pt.y, pt.kpt.pt.x - disparity);
							es->setMeasurement(obs);
							es->setInformation(Eigen::Matrix3d::Identity()/pixelVariance_);
							e = es;
						} else {
							// Mono
							g2o::EdgeProjectP2MC * em = new g2o::EdgeProjectP2MC();
							Eigen::Vector2d obs(pt.kpt.pt.x, pt.kpt.pt.y);
							em->setMeasurement(obs);
							em->setInformation(Eigen::Matrix2d::Identity()/pixelVariance_);
							e = em;
						}
						e->setVertex(0, vpt3d);
						e->setVertex(1, vCam);

						if (robustKernelDelta_ > 0.0) {
							g2o::RobustKernelHuber * kernel = new g2o::RobustKernelHuber;
							kernel->setDelta(robustKernelDelta_);
							e->setRobustKernel(kernel);
						}

						optimizer.addEdge(e);
						edges.push_back(e);
					}
				}
			}
		}

		// Optimize
		optimizer.initializeOptimization();
		assert(optimizer.verifyInformationMatrices());

		int it = 0;
		int outliersCount = 0;
		int outliersCountFar = 0;

		for (int i = 0; i < (robustKernelDelta_ > 0.0 ? 2 : 1); ++i) {
			it += optimizer.optimize((i==0 && robustKernelDelta_ > 0.0) ? 5 : iterations_);

			// early stop condition
			optimizer.computeActiveErrors();
			double chi2 = optimizer.activeRobustChi2();
			
			if (std::isnan(chi2)) {
				std::cout << "Optimization generated NANs, aborting optimization!" << std::endl;
				return optimizedPoses;
			}

			if (i > 0 && (optimizer.activeRobustChi2() > 1000000000000.0 || !std::isfinite(optimizer.activeRobustChi2()))) {
				std::cout << "g2o: Large optimization error detected, aborting optimization!" << std::endl;
				return optimizedPoses;
			}

			if (robustKernelDelta_ > 0.0) {
				for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
					if ((*iter)->level() == 0 && (*iter)->chi2() > (*iter)->robustKernel()->delta()) {
						(*iter)->setLevel(1);
						++outliersCount;
						double d = 0.0;
						if (dynamic_cast<g2o::EdgeProjectP2SC*>(*iter) != 0) {
							d = dynamic_cast<g2o::EdgeProjectP2SC *>(*iter)->measurement()[0] - dynamic_cast<g2o::EdgeProjectP2SC *>(*iter)->measurement()[2];
						}
						// std::cout << "Ignoring edge " << (*iter)->vertex(0)->id()-stepVertexId << "<->" << (*iter)->vertex(1)->id() << " d: " << d << "  var: " << 1.0/((g2o::EdgeProjectP2SC*)(*iter))->information()(0,0) << " kernel: " << (*iter)->robustKernel()->delta() << " chi2: " << (*iter)->chi2() << std::endl;
						Eigen::Vector3d pt3d;
						if ((*iter)->vertex(0)->id() > negVertexOffset) {
							pt3d = _points3D.at(negVertexOffset - (*iter)->vertex(0)->id());
						} else {
							pt3d = _points3D.at((*iter)->vertex(0)->id() - stepVertexId);
						}
						dynamic_cast<g2o::VertexSBAPointXYZ *>((*iter)->vertex(0))->setEstimate(pt3d);

						_outliers.insert((*iter)->vertex(0)->id() - stepVertexId);

						if (d < 5.0) {
							++outliersCountFar;
						}
					}
				}

				if (i == 0) {
					optimizer.initializeOptimization(0);
				}
			}
		}

		// Optimize end.
		if (optimizer.activeRobustChi2() > 1000000000000.0) {
			std::cout << "g2o: Large optimization error detected, aborting optimization!" << std::endl;
			return optimizedPoses;
		}

		// Update poses
		for (auto iter = _poses.begin(); iter != _poses.end(); ++iter) {
			if (iter->first > 0) {
				const g2o::VertexCam * v = dynamic_cast<const g2o::VertexCam *>(optimizer.vertex(iter->first));
				if (v) {
					Eigen::Isometry3d t(v->estimate().to_homogeneous_matrix());
					// remove transform image to robot
					const GeometricCamera & cameraModel = *_cameraModels.front();
					t = t * cameraModel.getTansformImageToRobot().inverse();
					if (t.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
						std::cout << "Optimized pose " << iter->first << " is null." << std::endl;
						optimizedPoses.clear();
						return optimizedPoses;
					}
					optimizedPoses.emplace(iter->first, t);
				} else {
					std::cout << "Vertex (pose) " << iter->first << " not found!" << std::endl;
				}
			}
		}

		// update point3d
		for (auto iter = _points3D.begin(); iter != _points3D.end(); ++ iter) {
			const g2o::VertexSBAPointXYZ * v;
			int id = iter->first;
			v = dynamic_cast<const g2o::VertexSBAPointXYZ *>(optimizer.vertex(stepVertexId + id));
			if (v) {
				iter->second = v->estimate();
			} else {
				iter->second[0] = iter->second[1] = iter->second[2] = std::numeric_limits<float>::quiet_NaN();
			}
		}

	} else if (_poses.size() == 1 || iterations_ <= 0) {
		optimizedPoses = _poses;
	} else {
		std::cout << "This method should be called at least with 1 pose!" << std::endl;
	}

	return optimizedPoses;
}

}   // namespace