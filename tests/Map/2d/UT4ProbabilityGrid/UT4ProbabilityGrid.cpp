#define BOOST_TEST_MAIN
#define BOOST_TEST_DYN_LINK

#include <iostream>
#include <random>
#include <boost/test/unit_test.hpp>

#include "Map/2d/ProbabilityGrid.h"
#include "Map/ProbabilityValues.h" 

namespace VISFS {
namespace Map {

BOOST_AUTO_TEST_CASE(ConstructorGridType) {
    ValueConversionTables conversionTables;
    ProbabilityGrid probabilityGrid(MapLimits(1.0, Eigen::Vector2d(1.0, 1.0), CellLimits(2, 2)), &conversionTables);
    BOOST_CHECK(probabilityGrid.getGridType() == GridType::PROBABILITY_GRID);
}

BOOST_AUTO_TEST_CASE(ApplyOdds) {
    ValueConversionTables conversionTables;
    ProbabilityGrid probabilityGrid(MapLimits(1.0, Eigen::Vector2d(1.0, 1.0), CellLimits(2, 2)), &conversionTables);
    const MapLimits & limits = probabilityGrid.limits();

    BOOST_CHECK(limits.contains(Eigen::Array2i(0, 0))== true);
    BOOST_CHECK(limits.contains(Eigen::Array2i(0, 1))== true);
    BOOST_CHECK(limits.contains(Eigen::Array2i(1, 0))== true);
    BOOST_CHECK(limits.contains(Eigen::Array2i(1, 1))== true);
    BOOST_CHECK(probabilityGrid.isKnown(Eigen::Array2i(0, 0)) == false);
    BOOST_CHECK(probabilityGrid.isKnown(Eigen::Array2i(0, 1)) == false);
    BOOST_CHECK(probabilityGrid.isKnown(Eigen::Array2i(1, 0)) == false);
    BOOST_CHECK(probabilityGrid.isKnown(Eigen::Array2i(1, 1)) == false);

    probabilityGrid.setProbability(Eigen::Array2i(1, 0), 0.5);
    BOOST_CHECK_EQUAL(probabilityGrid.getProbability(Eigen::Array2i(1, 0)), 0.5);
    probabilityGrid.applyLookUpTable(Eigen::Array2i(1, 0), computeLookupTableToApplyCorrespondenceCostOdds(odds(0.9)));
    probabilityGrid.finishUpdate();
    BOOST_CHECK_GT(probabilityGrid.getProbability(Eigen::Array2i(1, 0)), 0.5);

    probabilityGrid.setProbability(Eigen::Array2i(0, 1), 0.5);
    probabilityGrid.applyLookUpTable(Eigen::Array2i(0, 1), computeLookupTableToApplyCorrespondenceCostOdds(odds(0.1)));
    probabilityGrid.finishUpdate();
    BOOST_CHECK_LT(probabilityGrid.getProbability(Eigen::Array2i(0, 1)), 0.5);

    // Tests adding odds to an unknown cell.
    probabilityGrid.applyLookUpTable(Eigen::Array2i(1, 1), computeLookupTableToApplyCorrespondenceCostOdds(odds(0.42)));
    BOOST_CHECK_CLOSE(probabilityGrid.getProbability(Eigen::Array2i(1, 1)), 0.42, 1e-2);
    // Tests that further updates are ignored if FinishUpdate() isn't called.
    probabilityGrid.applyLookUpTable(Eigen::Array2i(1, 1), computeLookupTableToApplyCorrespondenceCostOdds(odds(0.9)));
    BOOST_CHECK_CLOSE(probabilityGrid.getProbability(Eigen::Array2i(1, 1)), 0.42, 1e-2);
    probabilityGrid.finishUpdate();
    probabilityGrid.applyLookUpTable(Eigen::Array2i(1, 1), computeLookupTableToApplyCorrespondenceCostOdds(odds(0.9)));
    BOOST_CHECK_GT(probabilityGrid.getProbability(Eigen::Array2i(1, 1)), 0.42);
}

BOOST_AUTO_TEST_CASE(GetProbability) {
    ValueConversionTables conversionTables;
    ProbabilityGrid probabilityGrid(MapLimits(1.0, Eigen::Vector2d(1.0, 2.0), CellLimits(2, 2)), &conversionTables);

    const MapLimits & limits = probabilityGrid.limits();
    BOOST_CHECK_EQUAL(1.0, limits.max().x());
    BOOST_CHECK_EQUAL(2.0, limits.max().y());

    const CellLimits & cellLmitis = limits.cellLimits();
    BOOST_REQUIRE_EQUAL(2, cellLmitis.numXcells);
    BOOST_REQUIRE_EQUAL(2, cellLmitis.numYcells);

    probabilityGrid.setProbability(limits.getCellIndex(Eigen::Vector2d(-0.5, 0.5)), kMaxProbability);
    BOOST_CHECK_CLOSE(probabilityGrid.getProbability(limits.getCellIndex(Eigen::Vector2d(-0.5, 0.5))), kMaxProbability, 1e-6);
    for (const Eigen::Array2i & xyIndex : {limits.getCellIndex(Eigen::Vector2d(-0.5, 1.5)),
                                            limits.getCellIndex(Eigen::Vector2d(0.5, 0.5)),
                                            limits.getCellIndex(Eigen::Vector2d(0.5, 1.5))}) {
        BOOST_CHECK_EQUAL(limits.contains(xyIndex), true);
        BOOST_CHECK_EQUAL(probabilityGrid.isKnown(xyIndex), false);                                            
    }

}

BOOST_AUTO_TEST_CASE(GetCellIndex) {
    ValueConversionTables conversionTables;
    ProbabilityGrid probabilityGrid(MapLimits(2.0, Eigen::Vector2d(8.0, 14.0), CellLimits(14, 8)), &conversionTables);

    const MapLimits & limits = probabilityGrid.limits();
    const CellLimits & cellLimits = limits.cellLimits();

    BOOST_REQUIRE_EQUAL(14, cellLimits.numXcells);
    BOOST_REQUIRE_EQUAL(8, cellLimits.numYcells);

    BOOST_CHECK_EQUAL((Eigen::Array2i(0, 0) == limits.getCellIndex(Eigen::Vector2d(7.0, 13.0))).all(), true);
    BOOST_CHECK_EQUAL((Eigen::Array2i(13, 0) == limits.getCellIndex(Eigen::Vector2d(7.0, -13.0))).all(), true);
    BOOST_CHECK_EQUAL((Eigen::Array2i(0, 7) == limits.getCellIndex(Eigen::Vector2d(-7.0, 13.0))).all(), true);
    BOOST_CHECK_EQUAL((Eigen::Array2i(13, 7) == limits.getCellIndex(Eigen::Vector2d(-7.0, -13.0))).all(), true);
    // Check around the origin.
    BOOST_CHECK_EQUAL((Eigen::Array2i(6, 3) == limits.getCellIndex(Eigen::Vector2d(0.5, 0.5))).all(), true);
    BOOST_CHECK_EQUAL((Eigen::Array2i(6, 3) == limits.getCellIndex(Eigen::Vector2d(1.5, 1.5))).all(), true);
    BOOST_CHECK_EQUAL((Eigen::Array2i(7, 3) == limits.getCellIndex(Eigen::Vector2d(0.5, -0.5))).all(), true);
    BOOST_CHECK_EQUAL((Eigen::Array2i(6, 4) == limits.getCellIndex(Eigen::Vector2d(-0.5, 0.5))).all(), true);
    BOOST_CHECK_EQUAL((Eigen::Array2i(7, 4) == limits.getCellIndex(Eigen::Vector2d(-0.5, -0.5))).all(), true);
}

BOOST_AUTO_TEST_CASE(CorrectCropping) {
    // Creat a probability grid with random values.
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<double> valueDistribution(kMinProbability, kMaxProbability);

    ValueConversionTables conversionTables;
    ProbabilityGrid probabilityGrid(MapLimits(0.05, Eigen::Vector2d(10.0, 10.0), CellLimits(400, 400)), &conversionTables);

    for (const Eigen::Array2i & xyIndex : XYIndexRangeIterator(Eigen::Array2i(100, 100), Eigen::Array2i(299, 299))) {
        probabilityGrid.setProbability(xyIndex, valueDistribution(rng));
    }

    Eigen::Array2i offset;
    CellLimits limits;
    probabilityGrid.computeCroppedLimits(&offset, &limits);
    BOOST_CHECK_EQUAL((offset == Eigen::Array2i(100, 100)).all(), true);
    BOOST_CHECK_EQUAL(limits.numXcells, 200);
    BOOST_CHECK_EQUAL(limits.numYcells, 200);
}

BOOST_AUTO_TEST_CASE(GRID2IMAGE) {
    ValueConversionTables conversionTables;
    ProbabilityGrid probabilityGrid(MapLimits(0.05, Eigen::Vector2d(10.0, 10.0), CellLimits(400, 400)), &conversionTables);

    for (const Eigen::Array2i & xyIndex : XYIndexRangeIterator(Eigen::Array2i(100, 100), Eigen::Array2i(299, 299))) {
        probabilityGrid.setProbability(xyIndex, kMaxProbability);
    }

    cv::Mat image = probabilityGrid.grid2Image();
}



}   // Map    
}   // VISFS
