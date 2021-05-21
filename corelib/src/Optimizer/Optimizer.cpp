#include "Optimizer/Optimizer.h"
#include "Optimizer/g2o/OptimizeTypeDefine.h"
#include "Optimizer/g2o/TypeOccupiedSpace2D.h"
#include "Optimizer/ceres/LocalParameterization.h"
#include "Optimizer/ceres/OccupiedSpace2dFactor.h"
#include "Optimizer/ceres/PoseConstraintFactor.h"
#include "Optimizer/ceres/StereoObservationFactor.h"
#include "Stl.h"
#include "Math.h"
#include "Log.h"
#include "Memory.h"

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
namespace Optimizer {
Optimizer::Optimizer(const ParametersMap & _parameters) :
	framework_(Parameters::defaultOptimizerFramework()),
	solver_(Parameters::defaultOptimizerSolver()),
	trustRegion_(Parameters::defaultOptimizerTrustRegion()),
	iterations_(Parameters::defaultOptimizerIterations()),
	pixelVariance_(Parameters::defaultOptimizerPixelVariance()),
	odometryCovariance_(Parameters::defaultOptimizerOdometryCovariance()),
	laserCovariance_(Parameters::defaultOptimizerLaserCovariance()),
	robustKernelDelta_(Parameters::defaultOptimizerRobustKernelDelta()) {
	
	Parameters::parse(_parameters, Parameters::kOptimizerFramework(), framework_);
	Parameters::parse(_parameters, Parameters::kOptimizerSolver(), solver_);
	Parameters::parse(_parameters, Parameters::kOptimizerTrustRegion(), trustRegion_);
	Parameters::parse(_parameters, Parameters::kOptimizerIterations(), iterations_);
	Parameters::parse(_parameters, Parameters::kOptimizerPixelVariance(), pixelVariance_);
	Parameters::parse(_parameters, Parameters::kOptimizerOdometryCovariance(), odometryCovariance_);
	Parameters::parse(_parameters, Parameters::kOptimizerLaserCovariance(), laserCovariance_);
	Parameters::parse(_parameters, Parameters::kOptimizerRobustKernelDelta(), robustKernelDelta_);

}

std::map<std::size_t, Eigen::Isometry3d> Optimizer::localOptimize(
    std::size_t _rootId,     // fixed pose
    const std::map<std::size_t, Eigen::Isometry3d> & _poses,    // map<pose index, transform>
    const std::map<std::size_t,std::tuple<std::size_t, std::size_t, Eigen::Isometry3d>> & _links,  // map<link index, tuple<the from pose index, the to pose index, transform, infomation matrix>>
    const std::vector<std::shared_ptr<GeometricCamera>> & _cameraModels, // vector camera model left and right
    std::map<std::size_t, std::tuple<Eigen::Vector3d, bool>> & _points3D,
    const std::map<std::size_t, std::map<std::size_t, FeatureBA>> & _wordReferences,
    const std::vector<Sensor::PointCloud> & _pointClouds,
    const std::shared_ptr<const Map::Submap2D> & _submap,
    std::vector<std::tuple<std::size_t, std::size_t>> & _outliers) {

	assert(_cameraModels.size() >= 1);
	std::map<std::size_t, Eigen::Isometry3d> optimizedPoses;

	if (framework_ == G2O) {

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

			if (trustRegion_ == 0) {
				optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))));
			} else if (trustRegion_ == 1) {
				optimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))));
			}

			// Set poses to g2o
			for (auto iter = _poses.begin(); iter != _poses.end(); ++iter) {
				if (iter->first > 0) {
					const GeometricCamera & cameraModel = *_cameraModels.front();
					// Twc = Twr * Trc
					Eigen::Isometry3d cameraPose = iter->second * cameraModel.getTansformImageToRobot();

					VertexPose * vCam = new VertexPose();
					// Tcw = Twc.inverse()
					cameraPose = cameraPose.inverse();
					vCam->setEstimate(CameraPose(cameraPose.linear(), cameraPose.translation()));
					vCam->setId(iter->first);
					vCam->setFixed(_rootId >= 0 && iter->first == _rootId);
					optimizer.addVertex(vCam);
				}
			}

			// Set edges to g2o
			Eigen::Matrix<double, 6, 6> informationOdometry;
			informationOdometry.setZero();
			for (int i = 0; i < 6; ++i) {
				informationOdometry(i, i) = 1. / odometryCovariance_;
			}

			for (auto iter = _links.begin(); iter != _links.end(); ++iter) {
				auto [fromId, toId, transfrom] = iter->second;

				if (fromId > 0 && toId > 0 && uContains(_poses, fromId) && uContains(_poses, toId)) {
					if(fromId == toId) {
						// TODO
					} else {
						const GeometricCamera & cameraModels = *_cameraModels.front();
						Eigen::Isometry3d Tri = cameraModels.getTansformImageToRobot();
						//Tc1c2 = Tcr * Tr1r2 * Trc
						Eigen::Isometry3d Tc1c2 = Tri.inverse()*transfrom*Tri;

						EdgePoseConstraint * e = new EdgePoseConstraint();
						VertexPose * v1 = dynamic_cast<VertexPose *>(optimizer.vertex(fromId));
						VertexPose * v2 = dynamic_cast<VertexPose *>(optimizer.vertex(toId));
						e->setVertex(0, v1);
						e->setVertex(1, v2);
						e->setMeasurement(g2o::SE3Quat(Tc1c2.linear(), Tc1c2.translation()));
						e->setInformation(informationOdometry);

						if (!optimizer.addEdge(e)) {
							delete e;
							LOG_ERROR << "Optimizer: Failed adding constraint between " << fromId << " and " << toId << ", skipping.";
							return optimizedPoses;
						}
					}
				}
			}

			// Set 3D points to g2o
			const Eigen::Matrix3d pixelInfo = Eigen::Matrix3d::Identity() / pixelVariance_;
			const int stepVertexId = static_cast<int>(_poses.rbegin()->first + 1);
			std::list<g2o::OptimizableGraph::Edge *> visualEdges;
			for (auto iter = _wordReferences.begin(); iter != _wordReferences.end(); ++iter) {
				int id = iter->first;
				if (_points3D.find(id) != _points3D.end()) {
					// Set vertex
					auto [pointPose, fixSymbol] = _points3D.at(id);
					g2o::VertexPointXYZ* vpt3d = new g2o::VertexPointXYZ();
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
							VertexPose * vCam = dynamic_cast<VertexPose *>(optimizer.vertex(cameraId));
							if (_cameraModels.size() > 1) {
								baseLine = cameraModel.getBaseLine();
							}
							if (std::isfinite(depth) && depth > 0.0 && baseLine > 0.0) {
								// Stereo
								EdgeStereo * es = new EdgeStereo();
								float disparity = static_cast<float>(baseLine * K(0, 0) / depth);
								Eigen::Vector3d obs(pt.kpt.pt.x, pt.kpt.pt.y, pt.kpt.pt.x - disparity);
								es->setMeasurement(obs);
								es->setInformation(pixelInfo);
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
							visualEdges.push_back(e);
						}
					}
				}
			}

			// Set range points to g2o
			if (!_pointClouds.empty() && _submap != nullptr) {
				const int pointStepVertexId = static_cast<int>(_poses.rbegin()->first + 1)
						+ (_wordReferences.empty() ? 0 : static_cast<int>(_wordReferences.rbegin()->first)) + 1;
				VertexPose * latestPose = dynamic_cast<VertexPose *>(optimizer.vertex(_poses.rbegin()->first));
				std::shared_ptr<Map::MapLimits> limits = std::make_shared<Map::MapLimits>(_submap->getGrid()->limits());
				const GridArrayAdapter adapter(*_submap->getGrid());
				std::shared_ptr<ceres::BiCubicInterpolator<GridArrayAdapter>> interpolator = std::make_shared<ceres::BiCubicInterpolator<GridArrayAdapter>>(adapter);
				Eigen::Matrix<double, 1, 1> informationRangePoint; informationRangePoint << (1. / laserCovariance_);
				int index = 0;
				const GeometricCamera & cameraModel = *_cameraModels.front();
				for (auto pointCloud : _pointClouds) {
					for (auto point : pointCloud.points()) {
						// Set Vertex
						VertexPoint3D * vRangePoint = new VertexPoint3D();
						vRangePoint->setEstimate(point.position);
						vRangePoint->setId(pointStepVertexId + (++index));
						vRangePoint->setFixed(true);
						vRangePoint->setMarginalized(true);
						optimizer.addVertex(vRangePoint);

						// Set Edge
						EdgeOccupiedObservation * eo = new EdgeOccupiedObservation(interpolator, limits, cameraModel.getTansformImageToRobot());
						eo->setVertex(0, latestPose);
						eo->setVertex(1, vRangePoint);
						eo->setInformation(informationRangePoint);

						if (!optimizer.addEdge(eo)) {
							delete eo;
							LOG_ERROR << "Optimizer: Failed adding " << index << "'th observation of grid map";
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
				for (auto iter = visualEdges.begin(); iter != visualEdges.end(); ++iter) {
					if ((*iter)->level() == 0 && (*iter)->chi2() > (*iter)->robustKernel()->delta()) {
						(*iter)->setLevel(1);
						++outliersCount;
						double d = 0.0;

						if (dynamic_cast<EdgeStereo *>(*iter) != 0) {
							d = dynamic_cast<EdgeStereo *>(*iter)->measurement()[0] - dynamic_cast<EdgeStereo *>(*iter)->measurement()[2];
						}

						// LOG_INFO << "Ignoring edge " << (*iter)->vertex(0)->id()-stepVertexId << "<->" << (*iter)->vertex(1)->id() << " d: " << d << "  var: " << 1.0/((EdgeStereo *)(*iter))->information()(0,0) << " kernel: " << (*iter)->robustKernel()->delta() << " chi2: " << (*iter)->chi2();

						_outliers.emplace_back(std::make_tuple<std::size_t, std::size_t>((*iter)->vertex(0)->id() - stepVertexId, (*iter)->vertex(1)->id()));

						if (d < 5.0) {
							++outliersCountFar;
						}
					}
				}
				LOG_DEBUG << "Optimizer: find outliers: " << _outliers.size();

				if (_outliers.size() > visualEdges.size()/2) {
					LOG_WARN << "Optimizer: large outliers detect, outliers size: " << _outliers.size() << ", total edges: " << visualEdges.size();
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
					const VertexPose * v = dynamic_cast<const VertexPose *>(optimizer.vertex(iter->first));
					if (v) {
						Eigen::Isometry3d t(v->estimate().toHomogeneousMatrix());
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
				const g2o::VertexPointXYZ * v;
				int id = iter->first;
				v = dynamic_cast<const g2o::VertexPointXYZ *>(optimizer.vertex(stepVertexId + id));
				if (v) {
					auto [oldPose, fixSymbol] = iter->second;
					const g2o::Vector3 & newPose = v->estimate();
					if (uNorm(oldPose[0]-newPose[0], oldPose[1]-newPose[1], oldPose[2]-newPose[2]) < 5.0) {
						iter->second = std::forward_as_tuple(v->estimate(), fixSymbol);
					}
				} else {
					auto [oldPose, fixSymbol] = iter->second;
					oldPose[0] = oldPose[1] = oldPose[2] = std::numeric_limits<double>::quiet_NaN();
					iter->second = std::forward_as_tuple(oldPose, fixSymbol);
				}
			}

		} else if (_poses.size() == 1 || iterations_ <= 0) {
			optimizedPoses = _poses;
		} else {
			LOG_ERROR << "This method should be called at least with 1 pose!";
		}

	} else if (framework_ == CERES) {
		
		if (_poses.size() >= 2 && iterations_ > 0 && _poses.begin()->first > 0) {
			ceres::Problem problem;
			ceres::LossFunction * lossFunction = new ceres::HuberLoss(robustKernelDelta_);

			// Set poses to ceres
			double parameterPoses[_poses.size()][7];
			// auto parameterPoses = createDynamicArray2D<double>(_poses.size(), 7);
			std::map<std::size_t, std::size_t> mappingSignatureId2Index;
			std::size_t index = 0;
			const GeometricCamera & cameraModel = *_cameraModels.front();

			for (auto iter = _poses.begin(); iter != _poses.end(); ++iter) {
				if (iter->first > 0) {
					
					// Twc = Twr * Trc
					Eigen::Isometry3d cameraPose = iter->second * cameraModel.getTansformImageToRobot();
					// Tcw = Twc.inverse()
					cameraPose = cameraPose.inverse();
					Eigen::Quaterniond qcw(cameraPose.linear());
					qcw = QuaternionPositify(qcw);
					Eigen::Vector3d tcw = cameraPose.translation();

					parameterPoses[index][0] = tcw[0];
					parameterPoses[index][1] = tcw[1];
					parameterPoses[index][2] = tcw[2];
					parameterPoses[index][3] = qcw.x();
					parameterPoses[index][4] = qcw.y();
					parameterPoses[index][5] = qcw.z();
					parameterPoses[index][6] = qcw.w();

					ceres::LocalParameterization * localParameterization = new PoseLocalParameterization();
					problem.AddParameterBlock(parameterPoses[index], 7, localParameterization);
					if (_rootId >= 0 && iter->first == _rootId) {
						problem.SetParameterBlockConstant(parameterPoses[index]);
					}
					mappingSignatureId2Index.insert(std::pair(iter->first, index));
					++index;
				}
			}

			// Set pose constraints to ceres
			Eigen::Matrix<double, 6, 6> informationOdometry;
			for (int i = 0; i < 6; ++i) {
				informationOdometry(i, i) = 1. / odometryCovariance_;
			}
			const Eigen::Isometry3d Tri = cameraModel.getTansformImageToRobot();

			for (auto iter = _links.begin(); iter != _links.end(); ++iter) {
				auto [fromId, toId, transfrom] = iter->second;

				if (fromId > 0 && toId > 0 && uContains(_poses, fromId) && uContains(_poses, toId)) {
					// TODO
				} else {
					auto itIndexFrom = mappingSignatureId2Index.find(fromId);
					auto itIndexTo = mappingSignatureId2Index.find(toId);
					if ((itIndexFrom != mappingSignatureId2Index.end()) && (itIndexTo != mappingSignatureId2Index.end())) {
						// Tc1c2 = Tcr * Tr1r2 * Trc
						Eigen::Isometry3d Tc1c2 = Tri.inverse() * transfrom * Tri;
						PoseConstraintFactor * factor = new PoseConstraintFactor(informationOdometry, Tc1c2.translation(), Eigen::Quaterniond(Tc1c2.linear()));
						problem.AddResidualBlock(factor, nullptr, parameterPoses[itIndexFrom->second], parameterPoses[itIndexTo->second]);
					}
				}
			}

			// Set visual point constraints to ceres
			double parameterPoints[_points3D.size()][3];
			// auto parameterPoints = createDynamicArray2D<double>(_points3D.size(), 3);
			
			std::map<std::size_t, std::size_t> mappingPointId2Index;
			index = 0;
			const Eigen::Matrix3d pixelInfo = Eigen::Matrix3d::Identity() / pixelVariance_;
			const Eigen::Matrix3d K = cameraModel.eigenKdouble();
			// tuple<feature id, signature pose id, feature index, pose index, observation>
			std::vector<std::tuple<std::size_t, std::size_t, std::size_t, std::size_t, Eigen::Vector3d>> residualBlocks;

			for (auto iter = _wordReferences.begin(); iter != _wordReferences.end(); ++iter) {
				std::size_t id = iter->first;
				if (_points3D.find(id) != _points3D.end()) {
					// Add parameter
					auto [pointPose, fixSymbol] = _points3D.at(id);
					parameterPoints[index][0] = pointPose[0];
					parameterPoints[index][1] = pointPose[1];
					parameterPoints[index][2] = pointPose[2];
					
					ceres::LocalParameterization * localParameterization = new PointLocalParameterization();
					problem.AddParameterBlock(parameterPoints[index], 3, localParameterization);
					if (fixSymbol) {
						problem.SetParameterBlockConstant(parameterPoints[index]);
					}
					mappingPointId2Index.insert(std::pair(iter->first, index));
					++index;

					// Add residuals
					for (auto jter = iter->second.begin(); jter != iter->second.end(); ++jter) {
						
						std::size_t cameraId = jter->first;
						auto itIndexPose = mappingSignatureId2Index.find(cameraId);
						auto itIndexPoint = mappingPointId2Index.find(id);
						if (itIndexPoint != mappingPointId2Index.end() && itIndexPose != mappingSignatureId2Index.end()) {
							const FeatureBA & pt = jter->second;
							const double depth = pt.depth;
							double baseline = 0.;
							
							if (_cameraModels.size() > 1) {
								baseline = cameraModel.getBaseLine();
							}
							if (std::isfinite(depth) && depth > 0.0 && baseline > 0.0) {
								// Stereo
								float disparity = static_cast<double>(baseline * K(0, 0) / depth);
								Eigen::Vector3d obs(pt.kpt.pt.x, pt.kpt.pt.y, pt.kpt.pt.x - disparity);
								residualBlocks.emplace_back(std::forward_as_tuple(id, cameraId, itIndexPoint->second, itIndexPose->second, obs));

								StereoObservationFactor * factor = new StereoObservationFactor(K(0, 0), K(1, 1), K(0, 2), K(1, 2), baseline * K(0, 0), pixelInfo, obs);
								problem.AddResidualBlock(factor, lossFunction, parameterPoints[itIndexPoint->second], parameterPoses[itIndexPose->second]);
							} else {
								// Mono
							}

						}
					}
				}
			}
			
			// Set range points to g2o
			if (!_pointClouds.empty() && _submap != nullptr) {
				const GeometricCamera & cameraModel = *_cameraModels.front();
				auto itIndexPose = mappingSignatureId2Index.find(_poses.rbegin()->first);

				if (itIndexPose != mappingSignatureId2Index.end()) {
					for (auto pointCloud : _pointClouds) {
						auto factor = createOccupiedSpaceCostFunction2d(1. / laserCovariance_, pointCloud, (*_submap->getGrid()), cameraModel.getTansformImageToRobot());
						problem.AddResidualBlock(factor, nullptr, parameterPoses[itIndexPose->second]);
					}
				}
				
			}

			// Optimize
			ceres::Solver::Options options;
			ceres::Solver::Summary summary;

			if (solver_ == 0) {
				options.linear_solver_type = ceres::DENSE_SCHUR;
			} else if (solver_ == 1) {
				options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
			} else if (solver_ == 2) {
				options.linear_solver_type = ceres::DENSE_QR;
			}
			
			if (trustRegion_ == 0) {
				options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
			} else if (trustRegion_ == 1) {
				options.trust_region_strategy_type = ceres::DOGLEG;
			}

			options.max_num_iterations = iterations_;
			options.max_solver_time_in_seconds = 0.06;
			options.num_threads = 2;

			ceres::Solve(options, &problem, &summary);

			// Cull outliers
			if (robustKernelDelta_ > 0.0) {
				for (auto residualBlock : residualBlocks) {
					auto [featureId, sigantureId, featureIndex, signatureIndex, obs] = residualBlock;
					const Eigen::Vector3d tcw(parameterPoses[signatureIndex][0], parameterPoses[signatureIndex][1], parameterPoses[signatureIndex][2]);
					const Eigen::Quaterniond qcw(parameterPoses[signatureIndex][6], parameterPoses[signatureIndex][3], parameterPoses[signatureIndex][4], parameterPoses[signatureIndex][5]);
					const Eigen::Vector3d pw(parameterPoints[featureIndex][0], parameterPoints[featureIndex][1], parameterPoints[featureIndex][2]);

					auto error = reProjectError(tcw, qcw, pw, K, static_cast<double>(cameraModel.getBaseLine()), obs);
					if (error.dot(pixelInfo * error) > robustKernelDelta_) {
						_outliers.emplace_back(std::forward_as_tuple(featureId, sigantureId));
					}
				}
			}

			// Update states
			for (auto iter = _poses.begin(); iter != _poses.end(); ++iter) {
				if (iter->first > 0) {
					auto itIndexPose = mappingSignatureId2Index.find(iter->first);
					if (itIndexPose != mappingSignatureId2Index.end()) {
						const Eigen::Vector3d tcw(parameterPoses[itIndexPose->second][0], parameterPoses[itIndexPose->second][1], parameterPoses[itIndexPose->second][2]);
						const Eigen::Quaterniond qcw(parameterPoses[itIndexPose->second][6], parameterPoses[itIndexPose->second][3], parameterPoses[itIndexPose->second][4], parameterPoses[itIndexPose->second][5]);
						Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
						t.rotate(qcw);
						t.pretranslate(tcw);
						// Twc = Tcw.inverse()
						t = t.inverse();
						// remove transform camera to robot
						t = t * Tri.inverse();
						if (t.isApprox(Eigen::Isometry3d(Eigen::Matrix4d::Zero()))) {
							LOG_WARN << "Optimized pose " << iter->first << " is null.";
							optimizedPoses.clear();
							return optimizedPoses;
						}
						optimizedPoses.emplace(iter->first, t);
					} else {
						LOG_ERROR << "Not found pose " << iter->first << " in pose array.";
					}
				}
			}

			for (auto iter = _points3D.begin(); iter != _points3D.end(); ++iter) {
				auto itIndexPoint = mappingPointId2Index.find(iter->first);
				if (itIndexPoint != mappingPointId2Index.end()) {
					auto [oldPose, fixSymbol] = iter->second;
					const Eigen::Vector3d newPose(parameterPoints[itIndexPoint->second][0], parameterPoints[itIndexPoint->second][1], parameterPoints[itIndexPoint->second][2]);
					if (uNorm(oldPose[0]-newPose[0], oldPose[1]-newPose[1], oldPose[2]-newPose[2]) < 5.0) {
						iter->second = std::forward_as_tuple(newPose, fixSymbol);
					}
				} else {
					auto [oldPose, fixSymbol] = iter->second;
					oldPose[0] = oldPose[1] = oldPose[2] = std::numeric_limits<double>::quiet_NaN();
					iter->second = std::forward_as_tuple(oldPose, fixSymbol);
				}
			}

			// realeaseDynamicArray2D(parameterPoses, _poses.size());
			// realeaseDynamicArray2D(parameterPoints, _points3D.size());
		
		} else if (_poses.size() == 1 || iterations_ <= 0) {
			optimizedPoses = _poses;
		} else {
			LOG_ERROR << "This method should be called at least with 1 pose!";
		}
	}

	return optimizedPoses;
}

}	// Optimizer
}   // VISFS