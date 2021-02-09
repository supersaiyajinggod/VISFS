#ifndef VISFS_MAP_2D_XYINDEX_H_
#define VISFS_MAP_2D_XYINDEX_H_

#include <iterator>
#include <Eigen/Core>

namespace VISFS {
namespace Map {

struct CellLimits {
    CellLimits() = default;

    CellLimits(int initNumXcells, int initNumYcells) : numXcells(initNumXcells), numYcells(initNumYcells) {}

    int numXcells = 0;
    int numYcells = 0;
};

class XYIndexRangeIterator : public std::iterator<std::input_iterator_tag, Eigen::Array2i> {
public:
    XYIndexRangeIterator(const Eigen::Array2i & _minIndex, const Eigen::Array2i & _maxIndex) :
        minIndex_(_minIndex),
        maxIndex_(_maxIndex),
        xyIndex_(_minIndex) {}
    
    explicit XYIndexRangeIterator(const CellLimits & _cellLimits) :
        XYIndexRangeIterator(Eigen::Array2i::Zero(), Eigen::Array2i(_cellLimits.numXcells - 1, _cellLimits.numYcells - 1)) {}

	XYIndexRangeIterator & operator++() {
		if (xyIndex_.x() < maxIndex_.x()) {
			++xyIndex_.x();
		} else {
			xyIndex_.x() = minIndex_.x();
			++xyIndex_.y();
		}
		return *this;
	}

	Eigen::Array2i & operator*() { return xyIndex_; }

	bool operator==(const XYIndexRangeIterator & _other) const {
		return (xyIndex_ == _other.xyIndex_).all();
	}

	bool operator!=(const XYIndexRangeIterator & _other) const {
		return !operator==(_other);
	}

	XYIndexRangeIterator begin() {
		return XYIndexRangeIterator(minIndex_, maxIndex_);
	}

	XYIndexRangeIterator end() {
		XYIndexRangeIterator it = begin();
		it.xyIndex_ = Eigen::Array2i(minIndex_.x(), maxIndex_.y() + 1);
		return it;
	}

private:
    Eigen::Array2i minIndex_;
    Eigen::Array2i maxIndex_;
    Eigen::Array2i xyIndex_;
};

}   // namespace Map
}   // namespace VISFS

#endif  // VISFS_MAP_2D_XYINDEX_H_