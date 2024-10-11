#ifndef DOUBLE_COMPARE_HPP
#define DOUBLE_COMPARE_HPP

#include <array>

namespace Compare {
    
const double epsilon = 1.0e-10;
using interval = std::array<double, 2>;
bool is_equal(const double x, const double y);
bool is_greater_or_equal(const double x, const double y);
bool is_less_or_equal(const double x, const double y);
bool number_intervals_overloap(const interval& interval_1, 
                               const interval& interval_2);

} // namespace Compare

#endif // DOUBLE_COMPARE_HPP