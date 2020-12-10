#ifndef VISFS_ESTIMATOR
#define VISFS_ESTIMATOR

#include <queue>
#include <boost/thread.hpp>

#include "Signature.h"
#include "Optimizer.h"
#include "LocalMap.h"

namespace VISFS {

class Estimator {
public:
    Estimator(const ParametersMap & _parameters = ParametersMap());
    virtual ~Estimator();

    void inputSignature(const Signature & _signature);
    Signature getEstimatedSignature();
    void threadProcess();

private:
    const double COVARIANCE_EPSILON = 0.000000001;
    
    void process(Signature & _signature);
    void outputSignature(const Signature & _signature);
    Eigen::Isometry3d guessVelocity(const Eigen::Isometry3d & _t, const double _dt);
    std::vector<std::size_t> findCorrespondences(const std::map<std::size_t, cv::Point3f> & _words3dFrom, const std::map<std::size_t, cv::KeyPoint> & _words2dTo);

    std::queue<Signature> signatureThreadBuf_;
    boost::mutex mutexDataRW_;
    std::queue<Signature> processResultBuf_;
    boost::mutex mutexResultRW_;

    Optimizer * optimizer_;
    LocalMap * localMap_;

    double previousStamps_;
    Eigen::Isometry3d pose_;
    Eigen::Isometry3d previousWheelOdom_;
    Eigen::Isometry3d velocityGuess_;
    
    int sensorStrategy_;
    int minInliers_;
    int pnpIterations_;
	float pnpReprojError_;
	int pnpFlags_;
	int refineIterations_;
    double toleranceTranslation_;
    double toleranceRotation_;
    bool force3Dof_;

    Eigen::Vector3d tempWheel = Eigen::Vector3d(0.0, 0.0 ,0.0);
    Eigen::Vector3d tempVisual = Eigen::Vector3d(0.0, 0.0, 0.0);
};

}   // namespace

#endif // VISFS_ESTIMATOR