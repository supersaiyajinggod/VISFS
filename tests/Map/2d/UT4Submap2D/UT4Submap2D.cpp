#define BOOST_TEST_MAIN
#define BOOST_TEST_DYN_LINK

#include <iostream>
#include <random>
#include <set>
#include <boost/test/unit_test.hpp>

#include "Map/2d/Submap2D.h"

namespace VISFS {
namespace Map {

BOOST_AUTO_TEST_CASE(TheRightNumberOfRangeDataAreInserted) {
    constexpr int kNumRangeData = 10;
    ActiveSubmaps2D submaps(kNumRangeData, GridType::PROBABILITY_GRID, 0.05, true, 0.55, 0.49);
    std::set<std::shared_ptr<const Submap2D>> allSubmaps;

    for (int i = 0; i != 1000; ++i) {
        auto insertionSubmaps = submaps.insertRangeData({Eigen::Vector3d::Zero(), {}, {}});
        for (const auto & submap : insertionSubmaps) {
            allSubmaps.insert(submap);
        }
        if (submaps.submaps().size() > 1) {
            BOOST_CHECK_LE(kNumRangeData, submaps.submaps().front()->getNumRangeData());
        }
    }

    BOOST_CHECK_EQUAL(2, submaps.submaps().size());

    int correctNumFinishedSubmaps = 0;
    int numUnfinishedSubmaps = 0;
    for (const auto & submap : allSubmaps) {
        if (submap->getNumRangeData() == kNumRangeData * 2) {
            ++correctNumFinishedSubmaps;
        } else {
            BOOST_CHECK_EQUAL(kNumRangeData, submap->getNumRangeData());
            ++numUnfinishedSubmaps;
        }
    }

    BOOST_CHECK_EQUAL(correctNumFinishedSubmaps, allSubmaps.size() - 1);
    BOOST_CHECK_EQUAL(1, numUnfinishedSubmaps);

}

}
}