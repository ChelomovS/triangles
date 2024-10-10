#include <limits>
#include <cmath>
#include <array>

#include "double_compare.hpp"

bool Compare::is_equal(const double x, const double y) {
    return std::fabs(x - y) < epsilon;
}

bool Compare::is_greater_or_equal(const double x, const double y) {
    return Compare::is_equal(x, y) || (x > y);
}

bool Compare::is_less_or_equal(const double x, const double y) {
    return Compare::is_equal(x, y) || (x < y);
}

bool Compare::number_intervals_overloap(const interval& interval_1, const interval& interval_2) {
    if (Compare::is_less_or_equal(interval_1[0], interval_2[0]) && Compare::is_less_or_equal(interval_2[0], interval_1[1]))
        return true;
    if (Compare::is_less_or_equal(interval_2[0], interval_1[0]) && Compare::is_less_or_equal(interval_1[0], interval_2[1]))
        return true;

    return false;
}