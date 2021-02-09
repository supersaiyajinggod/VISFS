#define BOOST_TEST_MAIN
#define BOOST_TEST_DYN_LINK

#include <iostream>
#include <boost/test/unit_test.hpp>

#include "Map/ProbabilityValues.h"

namespace VISFS {
namespace Map {

BOOST_AUTO_TEST_CASE(OddsConversions) {
    BOOST_CHECK_CLOSE(probabilityFromOdds(odds(kMinProbability)), kMinProbability, 1e-6);
    BOOST_CHECK_CLOSE(probabilityFromOdds(odds(kMaxProbability)), kMaxProbability, 1e-6);
    BOOST_CHECK_CLOSE(probabilityFromOdds(odds(0.5)), 0.5, 1e-6);
}

BOOST_AUTO_TEST_CASE(OddsConversionsCorrespondenceCost) {
    BOOST_CHECK_CLOSE(probabilityToCorrespondceCost(probabilityFromOdds(odds(correspondenceCostToProbability(kMinCorrespondenceCost)))), kMinCorrespondenceCost, 1e-6);
    BOOST_CHECK_CLOSE(probabilityToCorrespondceCost(probabilityFromOdds(odds(correspondenceCostToProbability(kMaxCorrespondenceCost)))), kMaxCorrespondenceCost, 1e-6);
    BOOST_CHECK_CLOSE(probabilityToCorrespondceCost(probabilityFromOdds(odds(correspondenceCostToProbability(0.5)))), 0.5, 1e-6);
}

BOOST_AUTO_TEST_CASE(ProbabilityValueToCorrespondenceCostValueConversions) {
    for (uint16_t i = 0; i < 32768; ++i) {
        BOOST_CHECK_EQUAL(probabilityValueToCorrespondenceCostValue(correspondenceCostValueToProbabilityValue(i)), i);
        BOOST_CHECK_EQUAL(correspondenceCostValueToProbabilityValue(probabilityValueToCorrespondenceCostValue(i)), i);
    }
}

BOOST_AUTO_TEST_CASE(ProbabilityValueToCorrespondenceCostValueConversionsWithUpdateMarker) {
    for (uint16_t i = 1; i < 32768; ++i) {
        BOOST_CHECK_EQUAL(probabilityValueToCorrespondenceCostValue(correspondenceCostValueToProbabilityValue(i + kUpdateMarker)), i + kUpdateMarker);
        BOOST_CHECK_EQUAL(correspondenceCostValueToProbabilityValue(probabilityValueToCorrespondenceCostValue(i + kUpdateMarker)), i + kUpdateMarker);
    }
}

BOOST_AUTO_TEST_CASE(ConversionLookUpTable) {
    BOOST_CHECK_CLOSE(valueToProbability(0), 1.0 - valueToCorrespondenceCost(0), 1e-6);
    for (uint16_t i = 1; i < 32768; ++i) {
        BOOST_CHECK_CLOSE(valueToProbability(i), valueToCorrespondenceCost(i), 1e-6);
    }
}

BOOST_AUTO_TEST_CASE(CellUpdate) {
    std::vector<uint16_t> probabilityTable = computeLookupTableToApplyOdds(odds(0.9));
    std::vector<uint16_t> correspondenceTable = computeLookupTableToApplyCorrespondenceCostOdds(odds(0.9));
    uint16_t cellPgPreUpdate = 0;
    uint16_t cellCgPreUpdate = 0;
    uint16_t cellPgPoseUpdate = probabilityTable[cellPgPreUpdate];
    uint16_t cellCgPoseUpdate = correspondenceTable[cellCgPreUpdate];
    double pPost = valueToProbability(cellPgPoseUpdate);
    double cPost = valueToCorrespondenceCost(cellCgPoseUpdate);
    BOOST_CHECK_CLOSE(pPost, 1.0 - cPost, 1e-6);
    int numEvaluations = 5000;
    for (int iProbability = 0; iProbability < numEvaluations; ++iProbability) {
        double p = (static_cast<double>(iProbability) / static_cast<double>(numEvaluations)) * (kMaxProbability - kMinProbability) + kMinProbability;
        cellPgPreUpdate = probabilityToValue(p);
        cellCgPreUpdate = correspondenceCostToValue(probabilityToCorrespondceCost(p));
        double pValue = (uClamp(p, kMinProbability, kMaxProbability) - kMinProbability) * (32766.0 / (kMaxProbability - kMinProbability));
        double cValue = (uClamp(probabilityToCorrespondceCost(p), kMinProbability, kMaxProbability) - kMinProbability)* (32766.0 / (kMaxProbability - kMinProbability));
        // BOOST_CHECK_CLOSE(cellPgPreUpdate, static_cast<uint16_t>(32768) - cellCgPreUpdate, 1);
        BOOST_CHECK_EQUAL(static_cast<int>(cellPgPreUpdate), 32768 - static_cast<int>(cellCgPreUpdate));
        cellPgPoseUpdate = probabilityTable[cellPgPreUpdate];
        cellCgPoseUpdate = correspondenceTable[cellCgPreUpdate];
        pPost = valueToProbability(cellPgPoseUpdate);
        cPost = valueToCorrespondenceCost(cellCgPoseUpdate);
        BOOST_CHECK_CLOSE(pPost, 1.0 - cPost, 5e-3);
    }
}

BOOST_AUTO_TEST_CASE(MultipleCellUpdate) {
    std::vector<uint16_t> probabilityTable = computeLookupTableToApplyOdds(odds(0.55));
    std::vector<uint16_t> correspondenceTable = computeLookupTableToApplyCorrespondenceCostOdds(odds(0.55));
    uint16_t cellPgPostUpdate = probabilityTable[0];
    uint16_t cellCgPostUpdate = correspondenceTable[0];
    double pPost = valueToProbability(cellPgPostUpdate);
    double cPost = valueToCorrespondenceCost(cellCgPostUpdate);
    BOOST_CHECK_CLOSE(pPost, 1.0 - cPost, 1e-6);
    int numEvaluations = 5000;
    for (int iProbability = 0; iProbability < numEvaluations; ++ iProbability) {
        double p = (static_cast<double>(iProbability) / static_cast<double>(numEvaluations)) * (kMaxProbability - kMinProbability) + kMinProbability;
        cellPgPostUpdate = probabilityToValue(p) + kUpdateMarker;
        cellCgPostUpdate = correspondenceCostToValue(probabilityToCorrespondceCost(p)) + kUpdateMarker;
        for (int iUpdate = 0; iUpdate < 20; ++ iUpdate) {
            cellPgPostUpdate = probabilityTable[cellPgPostUpdate - kUpdateMarker];
            cellCgPostUpdate = correspondenceTable[cellCgPostUpdate - kUpdateMarker];
        }
        pPost = valueToProbability(cellPgPostUpdate);
        cPost = valueToCorrespondenceCost(cellCgPostUpdate);
        BOOST_CHECK_CLOSE(pPost, 1.0 - cPost, 5e-5);
    }

}

BOOST_AUTO_TEST_CASE(EqualityLookupTableToApplyOdds) {
    std::vector<uint16_t> probabilityTable = computeLookupTableToApplyOdds(0.3);
    std::vector<uint16_t> correspondenceTable = computeLookupTableToApplyCorrespondenceCostOdds(0.3);

    for (int i = 0; i < 32768; ++i) {
        BOOST_CHECK_EQUAL(probabilityTable[i], correspondenceCostValueToProbabilityValue(correspondenceTable[probabilityValueToCorrespondenceCostValue(i)]));
        BOOST_CHECK_EQUAL(probabilityValueToCorrespondenceCostValue(probabilityTable[correspondenceCostValueToProbabilityValue(i)]), correspondenceTable[i]);
    }
}


}
}