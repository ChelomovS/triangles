#ifndef PLANE_HPP
#define PLANE_HPP

#include "point.hpp"
#include "vector.hpp"
#include "segment.hpp"

namespace Geom_objects {

template <typename T>
class plane_t {
    T a_, b_, c_, d_;
    vector_t<T> normal_vector_;
    point_t<T>  point_on_plane;

    public: 
    plane_t(const point_t<T>& point_1, const point_t<T>& point_2, const point_t<T>& point_3):
    a_{(point_3.get_z() - point_1.get_z()) * (point_2.get_y() - point_1.get_y()) - 
       (point_2.get_z() - point_1.get_z()) * (point_3.get_y() - point_1.get_y())},

    b_{(point_3.get_x() - point_1.get_x()) * (point_2.get_z() - point_1.get_z()) - 
       (point_3.get_z() - point_1.get_z()) * (point_2.get_x() - point_1.get_x())}, 

    c_{(point_2.get_x() - point_1.get_x()) * (point_3.get_y() - point_1.get_y()) - 
       (point_2.get_y() - point_1.get_y()) * (point_3.get_x() - point_1.get_x())}, 

    d_{-(point_1.get_x() * a_ + point_1.get_y() * b_ + point_1.get_z() * c_)},

    normal_vector_{a_, b_, c_},

    point_on_plane{point_1} {};

    public:
    T get_a() const { return a_; };
    T get_b() const { return b_; };
    T get_c() const { return c_; };
    T get_d() const { return d_; };
    vector_t<T> get_normal_vector() const { return normal_vector_; };

    point_t<T> intersect_plane_and_segment(const segment_t<T>& segment) const {
        point_t<T> invalid_point{NAN, NAN, NAN};

        T denominator = segment.get_dir_vector().dot_product(normal_vector_);
        T coeff = 0;
    
        if (Compare::is_equal(denominator, 0.0)) {
            if (Compare::is_equal(normal_vector_.dot_product(as_vector(segment.get_beg_point())) + d_, 0.0))
                return segment.get_beg_point();
            else
                return invalid_point; 
        }

        else {
            coeff = -(normal_vector_.dot_product(as_vector(segment.get_beg_point())) + d_) / denominator;

            point_t<T> intersection_point{segment.get_beg_point() + coeff * segment.get_dir_vector()};

            if (!segment.point_lies_on_segment(intersection_point))
                return invalid_point;

            return intersection_point;
        }
    }

    bool point_lies_on_plane(const point_t<T>& point) const {
        return Compare::is_equal(distance_between_point_and_plane(point), 0.0);
    }

    segment_t<T> intersection_of_planes(const plane_t<T>& other_plane) const {
        vector_t<T> directing_vector = normal_vector_.cross_product(other_plane.normal_vector_);

        T s1 = d_;
        T s2 = other_plane.d_;

        T n1_n2_dot = normal_vector_.dot_product(other_plane.normal_vector_);

        T n1_norm_sqr = normal_vector_.dot_product(normal_vector_);

        T n2_norm_sqr = other_plane.normal_vector_.dot_product(other_plane.normal_vector_);
    
        T denominator = n1_n2_dot * n1_n2_dot - n1_norm_sqr * n2_norm_sqr;

        T a = (s2 * n1_n2_dot - s1 * n2_norm_sqr) / denominator;

        T b = (s1 * n1_n2_dot - s2 * n1_norm_sqr) / denominator;

        point_t<T> point_on_line{a * a_ + b * other_plane.a_,
                                 a * b_ + b * other_plane.b_,
                                 a * c_ + b * other_plane.c_};

        segment_t<T> result_line{point_on_line, directing_vector};
        return result_line;
    }

    T distance_between_point_and_plane(const point_t<T>& point) const {
        return normal_vector_.dot_product(as_vector(point)) + d_;
    }

    bool planes_are_parallel(const plane_t<T>& other_plane) const {
        vector_t<T> v1_normal = normal_vector_;
        vector_t<T> v2_normal = other_plane.normal_vector_;

        vector_t<T> cross_product = v1_normal.cross_product(v2_normal);

        return cross_product.is_null_vector();
    }

    bool planes_are_equal(const plane_t<T>& other_plane) const {
        return point_lies_on_plane(other_plane.point_on_plane) &&
               planes_are_parallel(other_plane);
    }

    void print() const {
        std::cout << a_ << "x + " << b_ << "y + " << c_ << "z + " << d_ << std::endl;
        std::cout << "Normal vector of plane: ";
        normal_vector_.print();
    }
};

} // namespace Geom_objects

#endif // PLANE_HPP