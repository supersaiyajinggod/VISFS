#ifndef VISFS_MAP_PROBABILITY_VALUES_H_
#define VISFS_MAP_PROBABILITY_VALUES_H_

#include <cinttypes>
#include <cmath>
#include <vector>
#include <iostream>

#include "Math.h"

namespace VISFS {
namespace Map {

namespace {

inline uint16_t boundedDoubleToValue(const double _doubleValue, const double _lowerBound, const double _upperBound) {
    const int value = std::lround((uClamp(_doubleValue, _lowerBound, _upperBound)- _lowerBound) *
                                    (32766.0 / (_upperBound - _lowerBound))) + 1;
	return value;
}

}   // namespace

inline double odds(double _probability) {
    return _probability / (1.0 - _probability);
}

inline double probabilityFromOdds(const double _odds) {
    return _odds / (_odds + 1.0);
}

inline double probabilityToCorrespondceCost(const double _probability) {
    return 1.0 - _probability;
}

inline double correspondenceCostToProbability(const double _correspondenceCost) {
    return 1.0 - _correspondenceCost;
}

constexpr double kMinProbability = 0.1;
constexpr double kMaxProbability = 1.0 - kMinProbability;
constexpr double kMinCorrespondenceCost = 1.0 - kMaxProbability;
constexpr double kMaxCorrespondenceCost = 1.0 - kMinProbability;

inline double clampProbability(const double _probability) {
    return uClamp(_probability, kMinProbability, kMaxProbability);
}

inline double clampCorrespondenceCost(const double _correnspondenceCost) {
    return uClamp(_correnspondenceCost, kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

constexpr uint16_t kUnknownProbabilityValue = 0;
constexpr uint16_t kUnknownCorrespondenceValue = kUnknownProbabilityValue;
constexpr uint16_t kUpdateMarker = 1u << 15;

inline uint16_t correspondenceCostToValue(const double _correspondenceCost) {
	return boundedDoubleToValue(_correspondenceCost, kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

inline uint16_t probabilityToValue(const double _probability) {
	return boundedDoubleToValue(_probability, kMinProbability, kMaxProbability);
}

extern const std::vector<double> * const kValueToProbability;
extern const std::vector<double> * const kValueToCorrespondenceCost;

inline double valueToProbability(const uint16_t _value) {
	return (*kValueToProbability)[_value];
}

inline double valueToCorrespondenceCost(const uint16_t _value) {
	return (*kValueToCorrespondenceCost)[_value];
}

inline uint16_t probabilityValueToCorrespondenceCostValue(uint16_t _probabilityValue) {
	if (_probabilityValue == kUnknownProbabilityValue) {
		return kUnknownCorrespondenceValue;
	}
	bool updateCarry = false;
	if (_probabilityValue > kUpdateMarker) {
		_probabilityValue -= kUpdateMarker;
		updateCarry = true;
	}
	uint16_t result = correspondenceCostToValue(
		probabilityToCorrespondceCost(valueToProbability(_probabilityValue)));
	if (updateCarry) result += kUpdateMarker;
	return result;
}

inline uint16_t correspondenceCostValueToProbabilityValue(uint16_t _correspondenceCostValue) {
	if (_correspondenceCostValue == kUnknownCorrespondenceValue)
		return kUnknownProbabilityValue;
	bool updateCarry = false;
	if (_correspondenceCostValue > kUpdateMarker) {
		_correspondenceCostValue -= kUpdateMarker;
		updateCarry = true;
	}
	uint16_t result = probabilityToValue(
		correspondenceCostToProbability(valueToCorrespondenceCost(_correspondenceCostValue)));
	if (updateCarry) result += kUpdateMarker;
	return result;
}

std::vector<uint16_t> computeLookupTableToApplyOdds(double _odds);
std::vector<uint16_t> computeLookupTableToApplyCorrespondenceCostOdds(double _odds);

}   // namespace Map
}   // namespace VISFS

#endif