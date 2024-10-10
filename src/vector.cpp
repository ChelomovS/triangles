#include <cmath> 
#include <iostream>

#include "vector.hpp"

Point::point_t Vector::vector_t::get_vector_coord() const {
    return Point::point_t{x_, y_, z_};
}

double Vector::vector_t::dot_product(const vector_t& other_vector) const {
    // only in orthonormal basis
    return x_ * other_vector.x_ + y_ * other_vector.y_ + z_ * other_vector.z_;
}

Vector::vector_t Vector::vector_t::cross_product_vector(const Vector::vector_t& other_vector) const {
    // only in orthonormal basis
    double x = y_ * other_vector.z_ - z_ * other_vector.y_;
    double y = z_ * other_vector.x_ - x_ * other_vector.z_;
    double z = x_ * other_vector.y_ - y_ * other_vector.x_;

    Vector::vector_t result_vector{x, y, z};

    return result_vector;
}

double Vector::vector_t::cross_product_module(const Vector::vector_t& other_vector) const {
    // only in orthonormal basis
    double x = y_ * other_vector.z_ - z_ * other_vector.y_;
    double y = z_ * other_vector.x_ - x_ * other_vector.z_;
    double z = x_ * other_vector.y_ - y_ * other_vector.x_;

    double result = x + y + z;

    return result;
}

bool Vector::vector_t::vectors_are_collinear(const Vector::vector_t& other_vector) const {
    return cross_product_vector(other_vector).is_null_vector();
}

bool Vector::vector_t::is_null_vector() const {
    return (Compare::is_equal(x_, 0) &&
            Compare::is_equal(y_, 0) &&
            Compare::is_equal(z_, 0));
}

bool Vector::vector_t::is_equal(const Vector::vector_t& other_vector) const {
    return (Compare::is_equal(x_, other_vector.x_) && 
            Compare::is_equal(y_, other_vector.y_) &&
            Compare::is_equal(z_, other_vector.z_));
}

void Vector::vector_t::print() const {
    std::cout << "coordinates of vector: ";
    std::cout << x_ << "; " << y_ << "; " << z_ << std::endl;
}