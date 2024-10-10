#ifndef VECTOR_HPP
#define VECTOR_HPP

#include "point.hpp"

namespace Vector {

class vector_t {
    double x_, y_, z_;

    public:
    vector_t(const Point::point_t& beg_point, const Point::point_t& end_point): 
        x_{end_point.get_x() - beg_point.get_x()},
        y_{end_point.get_y() - beg_point.get_y()},
        z_{end_point.get_z() - beg_point.get_z()} {};

    vector_t(const double x = 0, const double y = 0, const double z = 0):
        x_{x}, y_{y}, z_{z} {};

    public:
    double get_x() const { return x_; };
    double get_y() const { return y_; };
    double get_z() const { return z_; };
    
    Point::point_t get_vector_coord() const;
    double dot_product(const vector_t& other_vector) const;
    vector_t cross_product_vector(const vector_t& other_vector) const;
    double cross_product_module(const vector_t& other_vector) const;
    bool is_null_vector() const;
    bool vectors_are_collinear(const vector_t& other_vector) const;
    bool is_equal(const vector_t& other_vector) const;
    void print() const;
};


} // namespace Vector

#endif // VECTOR_HPP