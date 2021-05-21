#include <ceres/cubic_interpolation.h>

#include "Optimizer/ceres/OccupiedSpace2dFactor.h"
#include "Map/ProbabilityValues.h"

namespace VISFS {
namespace Optimizer {

namespace {

class OccupiedSpace2dCostFunction {
public:
    OccupiedSpace2dCostFunction(const double _info, const Sensor::PointCloud & _pointCloud,
            const Map::Grid2D & _grid, const Eigen::Isometry3d _transformImageToRobot) :
            info_(_info), pointCloud_(_pointCloud), grid_(_grid),
            transformImageToRobot_(_transformImageToRobot), transformRobotToImage_(_transformImageToRobot.inverse()) {}

    template <typename T>
    bool operator()(const T * const pose, T * residual) const {
		// _pose: t1 t2 t3 x y z w
		// _point: x y z
        const Eigen::Matrix<T, 3, 1> t(pose[0], pose[1], pose[2]);
        const Eigen::Quaternion<T> q(pose[6], pose[3], pose[4], pose[5]);
        Eigen::Transform<T, 3, 1> Tiw = Eigen::Transform<T, 3, 1>::Identity();
        Tiw.prerotate(q);
        Tiw.pretranslate(t);
        const auto Twc = Tiw.inverse() * transformRobotToImage_.cast<T>();

        const GridArrayAdapter adpter(grid_);
        ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adpter);
        const Map::MapLimits & limits = grid_.limits();

        for (std::size_t i = 0; i < pointCloud_.size(); ++i) {
            const Eigen::Vector3d & point = pointCloud_[i].position; 
            const Eigen::Matrix<T, 3, 1> Pc(static_cast<T>(point[0]), static_cast<T>(point[1]), static_cast<T>(point[2]));
            const Eigen::Matrix<T, 3, 1> Po = Twc * Pc;

            interpolator.Evaluate(
                (limits.max().x() - Po[0]) / limits.resolution() - 0.5 + static_cast<double>(kPadding),
                (limits.max().y() - Po[1]) / limits.resolution() - 0.5 + static_cast<double>(kPadding),
                &residual[i]
            );
            residual[i] = info_ * residual[i];
        }

        return true;
    }

private:
    OccupiedSpace2dCostFunction(const OccupiedSpace2dCostFunction &) = delete;
    OccupiedSpace2dCostFunction & operator=(const OccupiedSpace2dCostFunction &) = delete;

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

    private:
        int NumRows() const {
            return grid_.limits().cellLimits().numYcells + 2 * kPadding;
        }

        int NumCols() const {
            return grid_.limits().cellLimits().numXcells + 2 * kPadding;
        }

        const Map::Grid2D & grid_;
    };

    static constexpr int kPadding = INT_MAX / 4;
    const double info_;
    const Sensor::PointCloud & pointCloud_;
    const Map::Grid2D & grid_;
    const Eigen::Isometry3d transformImageToRobot_;
    const Eigen::Isometry3d transformRobotToImage_;;
};

}   // namespace

ceres::CostFunction * createOccupiedSpaceCostFunction2d(const double _info, const Sensor::PointCloud & _pointCloud,
        const Map::Grid2D & _grid, const Eigen::Isometry3d _transformImageToRobot) {
    return new ceres::AutoDiffCostFunction<OccupiedSpace2dCostFunction, ceres::DYNAMIC, 7>(
        new OccupiedSpace2dCostFunction(_info, _pointCloud, _grid, _transformImageToRobot), _pointCloud.size()
    );
}

}   // Optimizer
}   // VISFS