#ifndef DOUBLE_COMPARE_HPP
#define DOUBLE_COMPARE_HPP

#include <array>

namespace Compare {
    
const double epsilon = 1.0e-9;

template <typename T>
using interval = std::array<T, 2>;

template <typename T>
bool is_equal(const T x, const T y) {
    return std::fabs(x - y) < epsilon;    
}

template <typename T>
bool is_greater_or_equal(const T x, const T y) {
    return Compare::is_equal(x, y) || (x > y);    
}

template <typename T>
bool is_less_or_equal(const T x, const T y) {
    return Compare::is_equal(x, y) || (x < y);    
}

template <typename T>
bool number_intervals_overloap(const interval<T>& interval_1, 
                               const interval<T>& interval_2) {
    if (Compare::is_less_or_equal(interval_1[0], interval_2[0]) && 
        Compare::is_less_or_equal(interval_2[0], interval_1[1]))
        return true;
        
    if (Compare::is_less_or_equal(interval_2[0], interval_1[0]) && 
        Compare::is_less_or_equal(interval_1[0], interval_2[1]))
        return true;

    return false;
}

} // namespace Compare

#endif // DOUBLE_COMPARE_HPP
