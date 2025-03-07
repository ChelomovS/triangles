#ifndef POINT_HPP
#define POINT_HPP

#include <cmath>
#include <cassert>

#include "double_compare.hpp"

namespace Geom_objects {

template <typename T>
class point_t {
    private: 
    T x_, y_, z_;
    size_t number_;

    public:
    point_t(T x = NAN, T y = NAN, T z = NAN, size_t number = 0)
        : x_{x}, y_{y}, z_{z}, number_{number} {}

    public:
    T         get_x()      const { return x_; }
    T         get_y()      const { return y_; }
    T         get_z()      const { return z_; }
    size_t    get_number() const { return number_; }

    void print() const {
        std::cerr << x_ << "; " << y_ << "; " << z_ << std::endl;
    }

    bool valid() const {
        return !((x_ != x_) || (y_ != y_) || (z_ != z_));
    }

    bool equal(const point_t<T>& other) const {
        assert(valid() && other.valid());

        return (Compare::is_equal(x_, other.x_) &&
                Compare::is_equal(y_, other.y_) &&
                Compare::is_equal(z_, other.z_));
    }

    T distance_between_points(const point_t<T>& other_point) const {
        assert(valid() && other_point.valid());

        return std::sqrt((x_ - other_point.x_) * (x_ - other_point.x_) +
                         (y_ - other_point.y_) * (y_ - other_point.y_) +
                         (z_ - other_point.z_) * (z_ - other_point.z_));
    }

    point_t<T> operator+(const point_t<T>& other) const {
        return point_t<T>(x_ + other.x_, y_ + other.y_, z_ + other.z_);
    }

    point_t<T> operator*(T scalar) const {
        return point_t<T>(x_ * scalar, y_ * scalar, z_ * scalar);
    }

    T operator[] (size_t i) const {
        switch(i) {
            case 0: 
                return x_;

            case 1: 
                return y_;

            case 2: 
                return z_;

            default: 
                throw std::out_of_range("invalid axis!");
        }
    }
};

} // namespace Geom_objects

#endif // POINT_HPP