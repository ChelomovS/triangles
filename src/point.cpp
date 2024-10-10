#include <cassert>
#include <iostream>

#include "point.hpp"

void Point::point_t::print() const {
    std::cout << x_ << "; " << y_ << "; " << z_ << std::endl;
}

bool Point::point_t::valid() const {
    return !((x_ != x_) || (y_ != y_) || (z_ != z_));
}

bool Point::point_t::equal(const Point::point_t &other) const {
    assert(valid() && other.valid());

    return (Compare::is_equal(x_, other.x_) &&
            Compare::is_equal(y_, other.y_) &&
            Compare::is_equal(z_, other.z_));
}

double Point::point_t::distance_between_points(const Point::point_t& other_point) const {
    assert(valid() && other_point.valid());

    return std::sqrt((x_ - other_point.x_) * (x_ - other_point.x_) +
                     (y_ - other_point.y_) * (y_ - other_point.y_) +
                     (z_ - other_point.z_) * (z_ - other_point.z_));
}
