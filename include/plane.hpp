#ifndef PLANE_HPP
#define PLANE_HPP

#include "point.hpp"
#include "vector.hpp"
#include "segment.hpp"

namespace Plane {

class plane_t {
    double a_, b_, c_, d_;
    Vector::vector_t normal_vector_;
    Point::point_t point_on_plane;

    public: 
    plane_t(const Point::point_t& point_1, const Point::point_t& point_2, 
            const Point::point_t& point_3):
    a_{(point_3.get_z() - point_1.get_z()) * (point_2.get_y() - point_1.get_y()) - 
       (point_2.get_z() - point_1.get_z()) * (point_3.get_y() - point_1.get_y())},

    b_{(point_3.get_x() - point_1.get_x()) * (point_2.get_z() - point_1.get_z()) - 
       (point_3.get_z() - point_1.get_z()) * (point_2.get_x() - point_1.get_x())}, 

    c_{(point_2.get_x() - point_1.get_x()) * (point_3.get_y() - point_1.get_y()) - 
       (point_2.get_y() - point_1.get_y()) * (point_3.get_x() - point_1.get_x())}, 

    d_{-1 * (point_1.get_x() * a_ + point_1.get_y() * b_ + point_1.get_z() * c_)},

    normal_vector_{a_, b_, c_},

    point_on_plane{point_1} {};

    public:
    double get_a() const { return a_; };
    double get_b() const { return b_; };
    double get_c() const { return c_; };
    double get_d() const { return d_; };
    Vector::vector_t get_normal_vector() const { return normal_vector_; };

    Point::point_t intersect_plane_and_segment(const Segment::segment_t segment) const;
    bool point_lies_on_plane(const Point::point_t& point) const;
    Segment::segment_t intersection_of_planes(const plane_t& other_plane) const;
    double distance_between_point_and_plane(const Point::point_t& point) const;
    bool planes_are_parallel(const plane_t& other_plane) const;
    bool planes_are_equal(const plane_t& other_plane) const;
    void print() const;
};

} // namespace Plane

#endif // PLANE_HPP