#ifndef VISFS_ESTIMATOR
#define VISFS_ESTIMATOR

#include <queue>
#include <boost/thread.hpp>

#include "Signature.h"

namespace VISFS {

class Estimator {
public:
    Estimator(){}
    void addSignature(const Signature & _signature);
    void threadProcess();

private:
    const double COVARIANCE_EPSILON = 0.000000001;
    
    void process(Signature & _signature);

    std::queue<Signature> signatureThreadBuf_;
    boost::mutex mutexDataRW_;

    Optimizer * optimizer_;

    int minInliers_;
    int pnpIterations_;
	float pnpReprojError_;
	int pnpFlags_;
	int refineIterations_;
	int bundleAdjustment_;
};

}   // namespace

#endif // VISFS_ESTIMATOR