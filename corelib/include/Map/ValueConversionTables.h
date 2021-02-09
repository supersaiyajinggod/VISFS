#ifndef VISFS_MAP_VALUE_CONVERSION_TABLE_H_
#define VISFS_MAP_VALUE_CONVERSION_TABLE_H_

#include <map>
#include <vector>
#include <tuple>
#include <memory>

namespace VISFS {
namespace Map {

// Performs lazy computations of lookup tables for mapping from a uint16 value
// to a float in ['lower_bound', 'upper_bound']. The first element of the table
// is set to 'unknown_result'.

class ValueConversionTables {
public:
    const std::vector<double> * getConversionTables(double _unknownResult, double _lowerBound, double _upperBound);

private:
    std::map<const std::tuple<double, double, double>, std::unique_ptr<const std::vector<double>>> boundsToLookupTables_;

};

}   // namespace Map 
}   // namespace VISFS

#endif  // VISFS_MAP_VALUE_CONVERSION_TABLE_H_