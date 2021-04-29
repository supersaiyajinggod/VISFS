#ifndef VISFS_MONITOR
#define VISFS_MONITOR

#include <queue>
#include <thread>
#include <mutex>

#include "Signature.h"
#include "Parameters.h"

namespace VISFS {

class Monitor {
public:
    Monitor(const ParametersMap & _parameters);
    void addSignature(const Signature & _signature);
    void threadProcess();

private:
    void process(Signature & _signature);

    int sensorStrategy_;    // 0: stereo, 1: rgbd.

    std::queue<Signature> signatureBuf_;
    std::mutex mutexDataRW_;
};

}   // namespace

#endif  // VISFS_MONITOR