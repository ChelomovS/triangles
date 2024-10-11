#ifndef POINT_HPP
#define POINT_HPP

#include <cmath>

#include "double_compare.hpp"

namespace Point {

class point_t {
    double x_ = NAN;
    double y_ = NAN;
    double z_ = NAN;

    public:
    point_t(double x = 0.0, double y = 0.0, double z = 0.0): x_{x}, y_{y}, z_{z} {}

    public:
    double get_x() const { return x_; }
    double get_y() const { return y_; }
    double get_z() const { return z_; }
    
    void   print() const;
    bool   valid() const;
    bool   equal(const point_t& other) const;
    double distance_between_points(const point_t& other_point) const;
};

} // namespace Point

#endif // POINT_HPP