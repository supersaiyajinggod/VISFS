#include "Map/ValueConversionTables.h"

#include <assert.h>
#include <iostream>

namespace VISFS {
namespace Map {

namespace {

constexpr uint16_t kUpdateMarker = 1u << 15;

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
double slowValueToBoundedDouble(const uint16_t _value, const uint16_t _unkonwnValue,
                const double _unkonwnResult, const double _lowerBound, const double _upperBound) {
    assert(_value <= 32767);
    if (_value == _unkonwnValue) return _unkonwnResult;
    const double kScale = (_upperBound - _lowerBound) / 32766.0;
    return _value * kScale + (_lowerBound - kScale);
}

std::unique_ptr<std::vector<double>> preComputeValueToBoundedDouble(const uint16_t _unknownValue, const double _unkonwnResult,
            const double _lowerBound, const double _upperBound) {
    auto result = std::make_unique<std::vector<double>>();
    std::size_t numValues = std::numeric_limits<uint16_t>::max() + 1;
    result->reserve(numValues);
    for (std::size_t value = 0; value != numValues; ++value) {
        result->emplace_back(slowValueToBoundedDouble(static_cast<uint16_t>(value) & ~kUpdateMarker, _unknownValue, _unkonwnResult, _lowerBound, _upperBound));
    }
    return result;
}

}   // namespace

const std::vector<double> * ValueConversionTables::getConversionTables(double _unknownResult, double _lowerBound, double _upperBound) {
    std::tuple<double, double, double> bounds = std::make_tuple(_unknownResult, _lowerBound, _upperBound);
    auto lookupTableIterator = boundsToLookupTables_.find(bounds);
    if (lookupTableIterator == boundsToLookupTables_.end()) {
        auto insertionResult = boundsToLookupTables_.emplace(bounds, preComputeValueToBoundedDouble(0, _unknownResult, _lowerBound, _upperBound));
        return insertionResult.first->second.get();
    } else {
        return lookupTableIterator->second.get();
    }
}

}   // namespace Map    
}   // namespace VISFS