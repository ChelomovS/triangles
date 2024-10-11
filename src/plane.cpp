#include <cmath>
#include <cassert>
#include <iostream>

#include "segment.hpp"
#include "plane.hpp"
#include "double_compare.hpp"

bool Plane::plane_t::point_lies_on_plane(const Point::point_t& point) const {
    return Compare::is_equal(distance_between_point_and_plane(point), 0);
}

bool Plane::plane_t::planes_are_parallel(const Plane::plane_t& other_plane) const {
    Vector::vector_t v1_normal = normal_vector_;
    Vector::vector_t v2_normal = other_plane.normal_vector_;

    double cross_product = v1_normal.cross_product_module(v2_normal);

    return Compare::is_equal(cross_product, 0);
}

Point::point_t Plane::plane_t::intersect_plane_and_segment(const Segment::segment_t segment) const {
    double denominator = segment.get_dir_vector().dot_product(normal_vector_);

    double coeff = 0;

    if (Compare::is_equal(denominator, 0)) {
        if (Compare::is_equal(a_ * segment.get_beg_point().get_x() + 
                              b_ * segment.get_beg_point().get_y() + 
                              c_ * segment.get_beg_point().get_z() + 
                              d_, 0)) {
            return segment.get_beg_point();   
        }

        else {
            Point::point_t invalid_point{NAN, NAN, NAN};
            return invalid_point;
        }
    }

    else
    {
        coeff = -(a_ * segment.get_beg_point().get_x() + 
                  b_ * segment.get_beg_point().get_y() + 
                  c_ * segment.get_beg_point().get_z() + d_) / denominator;

        Point::point_t intersection_point {segment.get_beg_point().get_x() + coeff * segment.get_dir_vector().get_x() +
                                           segment.get_beg_point().get_y() + coeff * segment.get_dir_vector().get_y() +
                                           segment.get_beg_point().get_z() + coeff * segment.get_dir_vector().get_z()};

        return intersection_point;
    }
}

bool Plane::plane_t::planes_are_equal(const plane_t& other_plane) const {
    return (point_lies_on_plane(other_plane.point_on_plane) &&
            planes_are_parallel(other_plane));
}

Segment::segment_t Plane::plane_t::intersection_of_planes(const Plane::plane_t& other_plane) const {
    Vector::vector_t directing_vector = normal_vector_.cross_product_vector(other_plane.normal_vector_);

    double s1 = d_;
    double s2 = other_plane.d_;

    double n1_n2_dot = normal_vector_.dot_product(other_plane.normal_vector_);

    double n1_norm_sqr = normal_vector_.dot_product(normal_vector_);

    double n2_norm_sqr = other_plane.normal_vector_.dot_product(other_plane.normal_vector_);
    
    double denominator = n1_n2_dot * n1_n2_dot - n1_norm_sqr * n2_norm_sqr;

    double a = (s2 * n1_n2_dot - s1 * n2_norm_sqr) / denominator;

    double b = (s1 * n1_n2_dot - s2 * n1_norm_sqr) / denominator;

    // point_on_line_ = a * plane_.normal_vector_ + b * normal_vector_
    // normal_vector_ = (a_, b_, c_) - coordinates of normal vector

    Point::point_t point_on_line{a * a_ + b * other_plane.a_,
                                 a * b_ + b * other_plane.b_,
                                 a * c_ + b * other_plane.c_};

    Segment::segment_t result_line{point_on_line, directing_vector};
    return result_line;
}

double Plane::plane_t::distance_between_point_and_plane(const Point::point_t& point) const {
    return normal_vector_.get_x() * point.get_x() +
           normal_vector_.get_y() * point.get_y() +
           normal_vector_.get_z() * point.get_z() + d_;
}

void Plane::plane_t::print() const {
    std::cout << a_ << "x + " << b_ << "y + " << c_ << "z + " << d_ << std::endl;
    std::cout << "Normal vector of plane: ";
    normal_vector_.print();
}