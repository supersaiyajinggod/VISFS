#ifndef VISFS_MAP_SUBMAPS_H_
#define VISFS_MAP_SUBMAPS_H_

#include <cmath>
#include <Eigen/Geometry>

#include "Map/ProbabilityValues.h"

namespace VISFS {
namespace Map {


/** \brief Converts the given probability to log odds.
 * \param[in] probability The probability.
 * \return The log odds of the giben probability.
 * \author eddy
 */
inline double logit(double _probability) {
    return std::log(_probability / (1.f - _probability));
}

const double kMaxLogOdds = logit(kMaxProbability);
const double kMinLogOdds = logit(kMinProbability);

inline uint8_t probabilityToLogOddsInteger(const double _probability) {
    const int value = std::lround((logit(_probability) - kMinLogOdds) * 254.f / (kMaxLogOdds - kMinLogOdds)) + 1;
    assert(1 <= value);
    assert(255 >= value);
    return value;
}

class Submap {
public:
    Submap(const Eigen::Isometry3d & _localPose) : localPose_(_localPose), rangeDataNum_(0), insertionFinish_(false) {}
    virtual ~Submap() {}

    /** \brief Get the pose of this submap in the local map frame.
     * \return The pose.
     * \author eddy
     */
    Eigen::Isometry3d localPose() const { return localPose_; }

    int getNumRangeData() const { return rangeDataNum_; }
    void setNumRangeData(const int _rangeDataNum) { rangeDataNum_ = _rangeDataNum; }

    bool getInsertionStatus() const { return insertionFinish_; }
    void setInsertionStatus(const bool _insertionFinish) { insertionFinish_ = _insertionFinish; } 

private:
    const Eigen::Isometry3d localPose_;
    int rangeDataNum_;
    bool insertionFinish_;
};

}   // namespace Map
}   // namespace VISFS

#endif  // VISFS_MAP_SUBMAPS_H_