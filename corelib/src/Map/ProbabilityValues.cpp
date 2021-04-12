#include "Map/ProbabilityValues.h"

#include <memory>

namespace VISFS {
namespace Map {

namespace {

constexpr int kValueCount = 32768;

// 0 is unknow, [1, 32767] maps to [lowerBound, upperBound]
double slowValueToBoundedDouble(const uint16_t _value, const uint16_t _unknownValue,
                const double _unknownResult, const double _lowerBound, const double _upperBound) {
    assert(_value < kValueCount);
    if (_value == _unknownValue) return _unknownResult;
    const double kScale = (_upperBound - _lowerBound) / (kValueCount - 2.0);
    return _value * kScale + (_lowerBound - kScale);
}

std::unique_ptr<std::vector<double>> preComputeValueToBoundedDouble(const uint16_t _unknownValue, const double _unknownResult,
                const double _lowerBound, const double _upperBound) {
    auto result = std::make_unique<std::vector<double>>();
    // Repeat two times, so that both values with and without the update marker
    // can be converted to a probability.
    constexpr int kRepetitionCount = 2;
    result->reserve(kRepetitionCount * kValueCount);
    for (int repeat = 0; repeat != kRepetitionCount; ++repeat) {
        for (int value = 0; value != kValueCount; ++value) {
            result->emplace_back(slowValueToBoundedDouble(value, _unknownValue, _unknownResult, _lowerBound, _upperBound));
        }
    }
    return result;
}

std::unique_ptr<std::vector<double>> preComputeValueToProbability() {
    return preComputeValueToBoundedDouble(kUnknownProbabilityValue, kMinProbability,
                kMinProbability, kMaxProbability);
}

std::unique_ptr<std::vector<double>> preComputeValueToCorrespondenceCost() {
    return preComputeValueToBoundedDouble(kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
                    kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

}

const std::vector<double> * const kValueToProbability = preComputeValueToProbability().release();

const std::vector<double> * const kValueToCorrespondenceCost = preComputeValueToCorrespondenceCost().release();

std::vector<uint16_t> computeLookupTableToApplyOdds(const double _odds) {
    std::vector<uint16_t> result;
    result.reserve(kValueCount);
    result.emplace_back(probabilityToValue(probabilityFromOdds(_odds)) + kUpdateMarker);
    for (int cell = 1; cell != kValueCount; ++cell) {
        result.emplace_back(probabilityToValue(probabilityFromOdds(_odds * odds((*kValueToProbability)[cell]))) + kUpdateMarker);
    }
    return result;
}

std::vector<uint16_t> computeLookupTableToApplyCorrespondenceCostOdds(const double _odds) {
    std::vector<uint16_t> result;
    result.reserve(kValueCount);
    result.emplace_back(correspondenceCostToValue(probabilityToCorrespondenceCost(probabilityFromOdds(_odds))) + kUpdateMarker);
    for (int cell = 1; cell != kValueCount; ++cell) {
        result.emplace_back(correspondenceCostToValue(probabilityToCorrespondenceCost(probabilityFromOdds(_odds * odds(correspondenceCostToProbability((*kValueToCorrespondenceCost)[cell]))))) + kUpdateMarker);
    }
    return result;
}

}   // namespace Map 
}   // namespace VISFS