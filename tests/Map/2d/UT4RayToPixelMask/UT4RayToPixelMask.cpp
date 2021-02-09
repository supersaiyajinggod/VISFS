#define BOOST_TEST_MAIN
#define BOOST_TEST_DYN_LINK

#include <iostream>
#include <boost/test/unit_test.hpp>

#include "Map/2d/RayToPixelMask.h"
#include "Map/2d/MapLimits.h"

namespace {

bool isEqual(const Eigen::Array2i & lhs, const Eigen::Array2i & rhs) {
    return ((lhs - rhs).matrix().lpNorm<1>() == 0);
}

}

BOOST_AUTO_TEST_CASE(EQUAL) {
    BOOST_CHECK(isEqual(Eigen::Array2i(1,3), Eigen::Array2i(1,3)) == true);
}

BOOST_AUTO_TEST_CASE(SingleCell) {
    const Eigen::Array2i & begin = {1, 1};
    const Eigen::Array2i & end = {1, 1};
    const int subPixelScale = 1;

    std::vector<Eigen::Array2i> ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
    BOOST_CHECK(isEqual(ray[0], begin) == true);
}

BOOST_AUTO_TEST_CASE(AxisAlginedX) {
    const Eigen::Array2i & begin = {1, 1};
    const Eigen::Array2i & end = {3, 1};
    const int subPixelScale = 1;
    std::vector<Eigen::Array2i> ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], Eigen::Array2i({(1+i), 1})));
    }
    ray.clear();
    ray = VISFS::Map::rayToPixelMask(end, begin, subPixelScale);
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], Eigen::Array2i({(1+i), 1})));
    }
}

BOOST_AUTO_TEST_CASE(AxisAlginedY) {
    const Eigen::Array2i & begin = {1, 1};
    const Eigen::Array2i & end = {1, 3};
    const int subPixelScale = 1;
    std::vector<Eigen::Array2i> ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], Eigen::Array2i({1, 1+i})));
    }
    ray.clear();
    ray = VISFS::Map::rayToPixelMask(end, begin, subPixelScale);
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], Eigen::Array2i({1, 1+i})));
    }
}

BOOST_AUTO_TEST_CASE(Digonal) {
    Eigen::Array2i begin = {1, 1};
    Eigen::Array2i end = {3, 3};
    const int subPixelScale = 1;
    std::vector<Eigen::Array2i> rayReference = std::vector<Eigen::Array2i>{
        Eigen::Array2i({1, 1}), Eigen::Array2i({2, 2}), Eigen::Array2i({3, 3})};
    std::vector<Eigen::Array2i> ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
    BOOST_REQUIRE(ray.size() == rayReference.size());
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], rayReference[i]));
    }
    ray.clear();
    ray = VISFS::Map::rayToPixelMask(end, begin, subPixelScale);
    BOOST_REQUIRE(ray.size() == rayReference.size());
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], rayReference[i]));
    }

    begin = Eigen::Array2i({1, 3});
    end = Eigen::Array2i({3, 1});
    rayReference = std::vector<Eigen::Array2i>{
        Eigen::Array2i({1, 3}), Eigen::Array2i({2, 2}), Eigen::Array2i({3, 1})};
    ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
    BOOST_REQUIRE(ray.size() == rayReference.size());
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], rayReference[i]));
    }
    ray = VISFS::Map::rayToPixelMask(end, begin, subPixelScale);
    BOOST_REQUIRE(ray.size() == rayReference.size());
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], rayReference[i]));
    }
}

BOOST_AUTO_TEST_CASE(SteepLine) {
    Eigen::Array2i begin = {1, 1};
    Eigen::Array2i end = {2, 5};
    const int subPixelScale = 1;
    std::vector<Eigen::Array2i> rayReference = std::vector<Eigen::Array2i>{
        Eigen::Array2i({1, 1}), Eigen::Array2i({1, 2}), Eigen::Array2i({1, 3}),
        Eigen::Array2i({2, 3}), Eigen::Array2i({2, 4}), Eigen::Array2i({2, 5})};
    std::vector<Eigen::Array2i> ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
    BOOST_REQUIRE(ray.size() == rayReference.size());
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], rayReference[i]));
    }
    begin = {1, 1};
    end = {2, 4};
    rayReference = std::vector<Eigen::Array2i>{
        Eigen::Array2i({1, 1}), Eigen::Array2i({1, 2}), Eigen::Array2i({2, 3}),
        Eigen::Array2i({2, 4})};
    ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
    BOOST_REQUIRE(ray.size() == rayReference.size());
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], rayReference[i]));
    }
}

BOOST_AUTO_TEST_CASE(FlatLine) {
    Eigen::Array2i begin = {1, 1};
    Eigen::Array2i end = {5, 2};
    const int subPixelScale = 1;
    std::vector<Eigen::Array2i> rayReference = std::vector<Eigen::Array2i>{
        Eigen::Array2i({1, 1}), Eigen::Array2i({2, 1}), Eigen::Array2i({3, 1}),
        Eigen::Array2i({3, 2}), Eigen::Array2i({4, 2}), Eigen::Array2i({5, 2})};
    std::vector<Eigen::Array2i> ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
    BOOST_REQUIRE(ray.size() == rayReference.size());
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], rayReference[i]));
    }
    begin = {1, 1};
    end = {4, 2};
    rayReference = std::vector<Eigen::Array2i>{
        Eigen::Array2i({1, 1}), Eigen::Array2i({2, 1}), Eigen::Array2i({3, 2}),
        Eigen::Array2i({4, 2})};
    ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
    BOOST_REQUIRE(ray.size() == rayReference.size());
    for (int i = 0; i < ray.size(); ++i) {
        BOOST_CHECK(isEqual(ray[i], rayReference[i]));
    }
}

BOOST_AUTO_TEST_CASE(MultiScaleAxisAlignedX) {
    int subPixelScale;
    const int numCellsX = 10;
    const int numCellsY = 10;
    double resolution = 0.1;
    Eigen::Vector2d max = {1.0, 1.0};
    std::vector<Eigen::Array2i> rayReference = std::vector<Eigen::Array2i>{
        Eigen::Array2i({9, 6}), Eigen::Array2i({9, 7}), Eigen::Array2i({9, 8}), Eigen::Array2i({9, 9})};
    for (subPixelScale = 1; subPixelScale < 10000; subPixelScale *= 2) {
        double superscaledResolution  = resolution / subPixelScale;
        VISFS::Map::MapLimits superScaledLimits(superscaledResolution, max,
            VISFS::Map::CellLimits(numCellsX * subPixelScale, numCellsY * subPixelScale));
        Eigen::Array2i begin = superScaledLimits.getCellIndex(Eigen::Vector2d({0.05, 0.05}));
        Eigen::Array2i end = superScaledLimits.getCellIndex(Eigen::Vector2d({0.35, 0.05}));
        std::vector<Eigen::Array2i> ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
        BOOST_REQUIRE(ray.size() == rayReference.size());
        for (int i = 0; i < ray.size(); ++i) {
            BOOST_CHECK(isEqual(ray[i], rayReference[i]));
            // Eigen::Vector2d center = superScaledLimits.getCellCenter(ray[i]);
            // std::cout << "center: " << center.transpose() << std::endl;
        }
    }
}

BOOST_AUTO_TEST_CASE(MultiScaleSkewedLine) {
    int subPixelScale;
    const int numCellsX = 1;
    const int numCellsY = 1;
    double resolution = 0.1;
    Eigen::Vector2d max = {1.0, 1.0};
    std::vector<Eigen::Array2i> rayReference = std::vector<Eigen::Array2i>{
        Eigen::Array2i({8, 7}), Eigen::Array2i({8, 8}), Eigen::Array2i({9, 8}), Eigen::Array2i({9, 9})};
    for (subPixelScale = 1; subPixelScale < 2; subPixelScale *= 2) {
        double superscaledResolution  = resolution / subPixelScale;
        VISFS::Map::MapLimits superScaledLimits(superscaledResolution, max,
            VISFS::Map::CellLimits(numCellsX * subPixelScale, numCellsY * subPixelScale));
        Eigen::Array2i begin = superScaledLimits.getCellIndex(Eigen::Vector2d({0.01, 0.09}));
        Eigen::Array2i end = superScaledLimits.getCellIndex(Eigen::Vector2d({0.21, 0.19}));
        std::vector<Eigen::Array2i> ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
        BOOST_REQUIRE(ray.size() == rayReference.size());
        for (int i = 0; i < ray.size(); ++i) {
            BOOST_CHECK(isEqual(ray[i], rayReference[i]));
        }
    }

    std::vector<Eigen::Array2i> rayReference2 = std::vector<Eigen::Array2i>{
        Eigen::Array2i({8, 7}), Eigen::Array2i({8, 8}), Eigen::Array2i({8, 9}), Eigen::Array2i({9, 9})};
    for (subPixelScale = 20; subPixelScale < 1000; subPixelScale *= 2) {
        double superscaledResolution  = resolution / subPixelScale;
        VISFS::Map::MapLimits superScaledLimits(superscaledResolution, max,
            VISFS::Map::CellLimits(numCellsX * subPixelScale, numCellsY * subPixelScale));
        Eigen::Array2i begin = superScaledLimits.getCellIndex(Eigen::Vector2d({0.01, 0.09}));
        Eigen::Array2i end = superScaledLimits.getCellIndex(Eigen::Vector2d({0.21, 0.19}));
        std::vector<Eigen::Array2i> ray = VISFS::Map::rayToPixelMask(begin, end, subPixelScale);
        BOOST_REQUIRE(ray.size() == rayReference2.size());
        for (int i = 0; i < ray.size(); ++i) {
            BOOST_CHECK(isEqual(ray[i], rayReference2[i]));
        }
    }
}
