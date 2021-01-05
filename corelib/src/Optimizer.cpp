#include "Optimizer.h"
#include "OptimizeTypeDefine.h"
#include "Stl.h"
#include "Math.h"
#include "Log.h"

#include <g2o/config.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
// #include "g2o/core/io_helper.h"
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
		} else if (optimizer_ == 1) {
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))));
		}

		// Set poses to g2o
		for (auto iter = _poses.begin(); iter != _poses.end(); ++iter) {
			if (iter->first > 0) {
				const GeometricCamera & cameraModel = *_cameraModels.front();
				// Twc = Twr * Trc
				Eigen::Isometry3d cameraPose = iter->second * cameraModel.getTansformImageToRobot();

				g2o::VertexSE3Expmap * vCam = new g2o::VertexSE3Expmap();
				// Tcw = Twc.inverse()
				cameraPose = cameraPose.inverse();
				vCam->setEstimate(g2o::SE3Quat(cameraPose.linear(), cameraPose.translation()));
				vCam->setId(iter->first);
				vCam->setFixed(rootId >= 0 && iter->first == rootId);
				optimizer.addVertex(vCam);
			}
		}

		// Set edges to g2o
		for (auto iter = _links.begin(); iter != _links.end(); ++iter) {
			auto [fromId, toId, transfrom, information] = iter->second;

			if (fromId > 0 && toId > 0 && uContains(_poses, fromId) && uContains(_poses, toId)) {
				if(fromId == toId) {
					// TODO
				} else {
					const GeometricCamera & cameraModels = *_cameraModels.front();
					Eigen::Isometry3d Tri = cameraModels.getTansformImageToRobot();
					//Tc1c2 = Tcr * Tr1r2 * Trc
					Eigen::Isometry3d Tc1c2 = Tri.inverse()*transfrom*Tri;
					Eigen::Isometry3d Tc2c1 = Tc1c2.inverse();
					g2o::EdgeSE3Expmap * e = new g2o::EdgeSE3Expmap();
					g2o::VertexSE3Expmap * v1 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(fromId));
					g2o::VertexSE3Expmap * v2 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(toId));
					e->setVertex(0, v1);
					e->setVertex(1, v2);
					e->setMeasurement(g2o::SE3Quat(Tc2c1.linear(), Tc2c1.translation()));
					e->setInformation(information);

					if (!optimizer.addEdge(e)) {
						delete e;
						LOG_WARN << "Optimizer: Failed adding constraint between " << fromId << " and " << toId << ", skipping.";
						return optimizedPoses;
					}
				}
			}
		}

		// Set 3D points to g2o
		const int stepVertexId = static_cast<int>(_poses.rbegin()->first + 1);
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
						const Eigen::Matrix3d K = cameraModel.eigenKdouble();
						// std::cout << "Add observation pt: " << vpt3d->id()-stepVertexId << " to cam= " << cameraId << " with " << pt.kpt.pt.x << " " << pt.kpt.pt.y << " depth= " << depth << std::endl;

						g2o::OptimizableGraph::Edge * e;
						g2o::VertexSE3Expmap * vCam = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(cameraId));
						if (_cameraModels.size() > 1) {
							baseLine = cameraModel.getBaseLine();
						}
						if (std::isfinite(depth) && depth > 0.0 && baseLine > 0.0) {
							// Stereo
							g2o::EdgeStereoSE3ProjectXYZ * es = new g2o::EdgeStereoSE3ProjectXYZ();
							float disparity = static_cast<float>(baseLine * K(0, 0) / depth);
							Eigen::Vector3d obs(pt.kpt.pt.x, pt.kpt.pt.y, pt.kpt.pt.x - disparity);
							es->setMeasurement(obs);
							es->setInformation(Eigen::Matrix3d::Identity()/pixelVariance_);
							es->fx = K(0, 0);
							es->fy = K(1, 1);
							es->cx = K(0, 2);
							es->cy = K(1, 2);
							es->bf = baseLine * es->fx;
							e = es;
						} else {
							// // Mono
							// g2o::EdgeSE3ProjectXYZ * em = new g2o::EdgeSE3ProjectXYZ();
							// Eigen::Vector2d obs(pt.kpt.pt.x, pt.kpt.pt.y);
							// em->setMeasurement(obs);
							// em->setInformation(Eigen::Matrix2d::Identity()/pixelVariance_);
							// em->fx = K(0, 0);
							// em->fy = K(1, 1);
							// em->cx = K(0, 2);
							// em->cy = K(1, 2);
							// e = em;
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
		optimizer.setVerbose(false);
		optimizer.initializeOptimization();
		assert(optimizer.verifyInformationMatrices());

		int it = 0;
		int outliersCount = 0;
		int outliersCountFar = 0;

		for (int i = 0; i < (robustKernelDelta_ > 0.0 ? 2 : 1); ++i) {
			it += optimizer.optimize((i==0 && robustKernelDelta_ > 0.0) ? iterations_ : iterations_);

			// early stop condition
			optimizer.computeActiveErrors();
			double chi2 = optimizer.activeRobustChi2();
			// std::cout << "iteration " << i << ": " << (int)optimizer.vertices().size() << " nodes, " << (int)optimizer.edges().size() << " edges" << " chi2: " << chi2 << std::endl;
			if (std::isnan(chi2)) {
				LOG_ERROR << "Optimization generated NANs, aborting optimization!";
				return optimizedPoses;
			}

			if (i > 0 && (chi2 > 1000000000000.0 || !std::isfinite(chi2))) {
				LOG_ERROR << "g2o: Large optimization error detected, aborting optimization!";
				return optimizedPoses;
			}

			if (robustKernelDelta_ > 0.0) {
				for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
					if ((*iter)->level() == 0 && (*iter)->chi2() > (*iter)->robustKernel()->delta()) {
						(*iter)->setLevel(1);
						++outliersCount;
						double d = 0.0;

						if (dynamic_cast<g2o::EdgeStereoSE3ProjectXYZ *>(*iter) != 0) {
							d = dynamic_cast<g2o::EdgeStereoSE3ProjectXYZ *>(*iter)->measurement()[0] - dynamic_cast<g2o::EdgeStereoSE3ProjectXYZ *>(*iter)->measurement()[2];
						}
						// std::cout << "Ignoring edge " << (*iter)->vertex(0)->id()-stepVertexId << "<->" << (*iter)->vertex(1)->id() << " d: " << d << "  var: " << 1.0/((EdgeStereoSE3PointXYZ*)(*iter))->information()(0,0) << " kernel: " << (*iter)->robustKernel()->delta() << " chi2: " << (*iter)->chi2() << std::endl;
						
						Eigen::Vector3d pt3d;
						pt3d = _points3D.at((*iter)->vertex(0)->id() - stepVertexId);
						dynamic_cast<g2o::VertexSBAPointXYZ *>((*iter)->vertex(0))->setEstimate(pt3d);

						_outliers.insert((*iter)->vertex(0)->id() - stepVertexId);

						if (d < 5.0) {
							++outliersCountFar;
						}
					}
				}

				if (_outliers.size() > _wordReferences.size()/2) {
					LOG_WARN << "g2o: Large outliers detect, _outliers.size(): " << _outliers.size();
					return optimizedPoses;
				}

				if (i == 0) {
					// std::cout << "Optimizer: _outliers.size(): " << _outliers.size() << std::endl;
					optimizer.initializeOptimization(0);
				}
			}
		}

		// Optimize end.
		if (optimizer.activeRobustChi2() > 1000000000000.0) {
			LOG_ERROR << "g2o: Large optimization error detected, aborting optimization!";
			return optimizedPoses;
		}

		// Update poses
		for (auto iter = _poses.begin(); iter != _poses.end(); ++iter) {
			if (iter->first > 0) {
				const g2o::VertexSE3Expmap * v = dynamic_cast<const g2o::VertexSE3Expmap *>(optimizer.vertex(iter->first));
				if (v) {
					Eigen::Isometry3d t(v->estimate().to_homogeneous_matrix());
					// Twc = Tcw.inverse()
					t = t.inverse();
					// remove transform camera to robot
					const GeometricCamera & cameraModel = *_cameraModels.front();
					t = t * cameraModel.getTansformImageToRobot().inverse();
					if (t.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
						LOG_ERROR << "Optimized pose " << iter->first << " is null.";
						optimizedPoses.clear();
						return optimizedPoses;
					}
					optimizedPoses.emplace(iter->first, t);
				} else {
					LOG_ERROR << "Vertex (pose) " << iter->first << " not found!";
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
		LOG_ERROR << "This method should be called at least with 1 pose!";
	}

	return optimizedPoses;
}


std::map<std::size_t, Eigen::Isometry3d> Optimizer::localOptimize(
    std::size_t _rootId,     // fixed pose
    const std::map<std::size_t, Eigen::Isometry3d> & _poses,    // map<pose index, transform>
    const std::map<std::size_t,std::tuple<std::size_t, std::size_t, Eigen::Isometry3d, Eigen::Matrix<double, 6, 6>>> & _links,  // map<link index, tuple<the from pose index, the to pose index, transform, infomation matrix>>
    const std::vector<boost::shared_ptr<GeometricCamera>> & _cameraModels, // vector camera model left and right
    std::map<std::size_t, std::tuple<Eigen::Vector3d, bool>> & _points3D,
    const std::map<std::size_t, std::map<std::size_t, FeatureBA>> & _wordReferences,
    std::vector<std::tuple<std::size_t, std::size_t>> & _outliers) {

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
		} else if (optimizer_ == 1) {
			optimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))));
		}

		// Set poses to g2o
		for (auto iter = _poses.begin(); iter != _poses.end(); ++iter) {
			if (iter->first > 0) {
				const GeometricCamera & cameraModel = *_cameraModels.front();
				// Twc = Twr * Trc
				Eigen::Isometry3d cameraPose = iter->second * cameraModel.getTansformImageToRobot();

				g2o::VertexSE3Expmap * vCam = new g2o::VertexSE3Expmap();
				// Tcw = Twc.inverse()
				cameraPose = cameraPose.inverse();
				vCam->setEstimate(g2o::SE3Quat(cameraPose.linear(), cameraPose.translation()));
				vCam->setId(iter->first);
				vCam->setFixed(_rootId >= 0 && iter->first == _rootId);
				optimizer.addVertex(vCam);
			}
		}

		// Set edges to g2o
		for (auto iter = _links.begin(); iter != _links.end(); ++iter) {
			auto [fromId, toId, transfrom, information] = iter->second;

			if (fromId > 0 && toId > 0 && uContains(_poses, fromId) && uContains(_poses, toId)) {
				if(fromId == toId) {
					// TODO
				} else {
					const GeometricCamera & cameraModels = *_cameraModels.front();
					Eigen::Isometry3d Tri = cameraModels.getTansformImageToRobot();
					//Tc1c2 = Tcr * Tr1r2 * Trc
					Eigen::Isometry3d Tc1c2 = Tri.inverse()*transfrom*Tri;
					Eigen::Isometry3d Tc2c1 = Tc1c2.inverse();

					g2o::EdgeSE3Expmap * e = new g2o::EdgeSE3Expmap();
					g2o::VertexSE3Expmap * v1 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(fromId));
					g2o::VertexSE3Expmap * v2 = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(toId));
					e->setVertex(0, v1);
					e->setVertex(1, v2);
					e->setMeasurement(g2o::SE3Quat(Tc2c1.linear(), Tc2c1.translation()));
					e->setInformation(information);

					if (!optimizer.addEdge(e)) {
						delete e;
						LOG_ERROR << "Optimizer: Failed adding constraint between " << fromId << " and " << toId << ", skipping.";
						return optimizedPoses;
					}
				}
			}
		}

		// Set 3D points to g2o
		const int stepVertexId = static_cast<int>(_poses.rbegin()->first + 1);
		std::list<g2o::OptimizableGraph::Edge *> edges;
		for (auto iter = _wordReferences.begin(); iter != _wordReferences.end(); ++iter) {
			int id = iter->first;
			if (_points3D.find(id) != _points3D.end()) {
				// Set vertex
				auto [pointPose, fixSymbol] = _points3D.at(id);
				g2o::VertexSBAPointXYZ* vpt3d = new g2o::VertexSBAPointXYZ();
				vpt3d->setEstimate(pointPose);
				vpt3d->setId(stepVertexId + id);
				vpt3d->setFixed(fixSymbol);
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
						const Eigen::Matrix3d K = cameraModel.eigenKdouble();
						// std::cout << "Add observation pt: " << vpt3d->id()-stepVertexId << " to cam= " << cameraId << " with " << pt.kpt.pt.x << " " << pt.kpt.pt.y << " depth= " << depth << std::endl;

						g2o::OptimizableGraph::Edge * e;
						g2o::VertexSE3Expmap * vCam = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(cameraId));
						if (_cameraModels.size() > 1) {
							baseLine = cameraModel.getBaseLine();
						}
						if (std::isfinite(depth) && depth > 0.0 && baseLine > 0.0) {
							// Stereo
							g2o::EdgeStereoSE3ProjectXYZ * es = new g2o::EdgeStereoSE3ProjectXYZ();
							float disparity = static_cast<float>(baseLine * K(0, 0) / depth);
							Eigen::Vector3d obs(pt.kpt.pt.x, pt.kpt.pt.y, pt.kpt.pt.x - disparity);
							es->setMeasurement(obs);
							es->setInformation(Eigen::Matrix3d::Identity()/pixelVariance_);
							es->fx = K(0, 0);
							es->fy = K(1, 1);
							es->cx = K(0, 2);
							es->cy = K(1, 2);
							es->bf = baseLine * es->fx;
							e = es;
						} else {
							// // Mono
							// g2o::EdgeSE3ProjectXYZ * em = new g2o::EdgeSE3ProjectXYZ();
							// Eigen::Vector2d obs(pt.kpt.pt.x, pt.kpt.pt.y);
							// em->setMeasurement(obs);
							// em->setInformation(Eigen::Matrix2d::Identity()/pixelVariance_);
							// em->fx = K(0, 0);
							// em->fy = K(1, 1);
							// em->cx = K(0, 2);
							// em->cy = K(1, 2);
							// e = em;
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
		optimizer.setVerbose(false);
		optimizer.initializeOptimization();
		assert(optimizer.verifyInformationMatrices());

		optimizer.optimize(iterations_/2);

		// Early stop condition
		int outliersCount = 0;
		int outliersCountFar = 0;
		optimizer.computeActiveErrors();
		double chi2 = optimizer.activeRobustChi2();
		if (std::isnan(chi2)) {
			LOG_ERROR << "Optimization generated NANs, aborting optimization!";
			return optimizedPoses;
		}

		if (chi2 > 1000000000000.0 || !std::isfinite(chi2)) {
			LOG_ERROR << "g2o: Large optimization error detected in the first time optimize, aborting optimization!";
			return optimizedPoses;
		}

		// Optimize again
		if (robustKernelDelta_ > 0.0) {
			for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
				if ((*iter)->level() == 0 && (*iter)->chi2() > (*iter)->robustKernel()->delta()) {
					(*iter)->setLevel(1);
					++outliersCount;
					double d = 0.0;

					if (dynamic_cast<g2o::EdgeStereoSE3ProjectXYZ *>(*iter) != 0) {
						d = dynamic_cast<g2o::EdgeStereoSE3ProjectXYZ *>(*iter)->measurement()[0] - dynamic_cast<g2o::EdgeStereoSE3ProjectXYZ *>(*iter)->measurement()[2];
					}

					LOG_DEBUG << "Ignoring edge " << (*iter)->vertex(0)->id()-stepVertexId << "<->" << (*iter)->vertex(1)->id() << " d: " << d << "  var: " << 1.0/((g2o::EdgeStereoSE3ProjectXYZ *)(*iter))->information()(0,0) << " kernel: " << (*iter)->robustKernel()->delta() << " chi2: " << (*iter)->chi2();

					// _outliers.insert((*iter)->vertex(0)->id() - stepVertexId);
					_outliers.emplace_back(std::make_tuple<std::size_t, std::size_t>((*iter)->vertex(0)->id() - stepVertexId, (*iter)->vertex(1)->id()));

					if (d < 5.0) {
						++outliersCountFar;
					}
				}
			}
			LOG_DEBUG << "Optimizer: find outliers: " << _outliers.size();

			if (_outliers.size() > _wordReferences.size()/2) {
				LOG_WARN << "Optimizer: large outliers detect, outliers size: " << _outliers.size();
				// return optimizedPoses;
			}

			optimizer.initializeOptimization(0);
			optimizer.optimize(iterations_/2);
		}

		// Optimize end.
		if (optimizer.activeRobustChi2() > 1000000000000.0) {
			LOG_ERROR << "g2o: Large optimization error detected in the second time optimize, aborting optimization!";
			return optimizedPoses;
		}
		// Update poses
		for (auto iter = _poses.begin(); iter != _poses.end(); ++iter) {
			if (iter->first > 0) {
				const g2o::VertexSE3Expmap * v = dynamic_cast<const g2o::VertexSE3Expmap *>(optimizer.vertex(iter->first));
				if (v) {
					Eigen::Isometry3d t(v->estimate().to_homogeneous_matrix());
					// Twc = Tcw.inverse()
					t = t.inverse();
					// remove transform camera to robot
					const GeometricCamera & cameraModel = *_cameraModels.front();
					t = t * cameraModel.getTansformImageToRobot().inverse();
					if (t.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
						LOG_WARN << "Optimized pose " << iter->first << " is null.";
						optimizedPoses.clear();
						return optimizedPoses;
					}
					optimizedPoses.emplace(iter->first, t);
				} else {
					LOG_WARN << "Vertex (pose) " << iter->first << " not found!";
				}
			}
		}

		// update point3d
		for (auto iter = _points3D.begin(); iter != _points3D.end(); ++ iter) {
			const g2o::VertexSBAPointXYZ * v;
			int id = iter->first;
			v = dynamic_cast<const g2o::VertexSBAPointXYZ *>(optimizer.vertex(stepVertexId + id));
			if (v) {
				auto [oldPose, fixSymbol] = iter->second;
				iter->second = std::make_tuple(v->estimate(), fixSymbol);
			} else {
				auto [oldPose, fixSymbol] = iter->second;
				oldPose[0] = oldPose[1] = oldPose[2] = std::numeric_limits<float>::quiet_NaN();
				iter->second = std::make_tuple(oldPose, fixSymbol);
			}
		}

	} else if (_poses.size() == 1 || iterations_ <= 0) {
		optimizedPoses = _poses;
	} else {
		LOG_ERROR << "This method should be called at least with 1 pose!";
	}

	return optimizedPoses;
}

}   // namespace