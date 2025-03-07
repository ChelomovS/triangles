#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

#include "point.hpp"
#include "plane.hpp"
#include "segment.hpp"

namespace Geom_objects {

template <typename T>
class triangle_t {
    private:
    point_t<T> a_, b_, c_;
    segment_t<T> segment_ab_, segment_bc_, segment_ca_;
    plane_t<T> plane_;
    size_t number_;

    public:
    triangle_t(const point_t<T>& a, const point_t<T>& b, const point_t<T>& c, size_t number = 0): 
    a_{a}, b_{b}, c_{c}, segment_ab_{a, b}, segment_bc_{b, c}, segment_ca_{c, a},
    plane_{a, b, c}, number_{number} {}

    public:
    point_t<T> get_a() const { return a_; }
    point_t<T> get_b() const { return b_; }
    point_t<T> get_c() const { return c_; }
    segment_t<T> get_segment_ab() const { return segment_ab_; } 
    segment_t<T> get_segment_bc() const { return segment_bc_; }
    segment_t<T> get_segment_ca() const { return segment_ca_; }
    plane_t<T> get_plane()  const { return plane_; };
    size_t     get_number() const { return number_; };

    bool triangle_intersect_segment(const segment_t<T>& segment) const {
        bool result = segment_ab_.segments_intersects(segment) ||
                      segment_ca_.segments_intersects(segment) ||
                      segment_bc_.segments_intersects(segment);

        if (result) 
            return result;

        if (plane_.point_lies_on_plane(segment.get_beg_point()) && plane_.point_lies_on_plane(segment.get_end_point()))
            return point_lies_inside_triangle(segment.get_beg_point()) || 
                   point_lies_inside_triangle(segment.get_beg_point());

        else 
            return point_lies_inside_triangle(plane_.intersect_plane_and_segment(segment));
    }

    Compare::interval<double> calculate_intersection_interval(const segment_t<T>& line,
                                                              const std::array<T, 3>& distance) const {
        vector_t<T> first_vector{a_ - line.get_beg_point()};

        vector_t<T> second_vector{b_ - line.get_beg_point()};

        vector_t<T> third_vector{c_ - line.get_beg_point()};

        T proj_0 = first_vector.dot_product(line.get_dir_vector());
        T proj_1 = second_vector.dot_product(line.get_dir_vector());
        T proj_2 = third_vector.dot_product(line.get_dir_vector());

        Compare::interval<double> interval{};

        bool segment_is_found = false;

        if ((Compare::is_greater_or_equal(distance[0] * distance[1], 0.0)) && 
            !Compare::is_equal(distance[1] - distance[2], 0.0) && 
            !Compare::is_equal(distance[0] - distance[2], 0.0)) {
            segment_is_found = true;

            interval[0] = proj_0 + (proj_2 - proj_0) * (distance[0] / (distance[0] - distance[2]));

            interval[1] = proj_1 + (proj_2 - proj_1) * (distance[1] / (distance[1] - distance[2]));
        }
        
        else if (Compare::is_greater_or_equal(distance[0] * distance[2], 0.0) && 
                !Compare::is_equal(distance[0] - distance[1], 0.0) && 
                !Compare::is_equal(distance[2] - distance[1], 0.0)) {
            segment_is_found = true;

            interval[0] = proj_0 + (proj_1 - proj_0) * (distance[0] / (distance[0] - distance[1]));

            interval[1] = proj_2 + (proj_1 - proj_2) * (distance[2] / (distance[2] - distance[1]));
        }

        else if ((Compare::is_greater_or_equal(distance[1] * distance[2], 0.0)) && 
                 !Compare::is_equal(distance[1] - distance[0], 0.0) && 
                 !Compare::is_equal(distance[2] - distance[0], 0.0)) {
            segment_is_found = true;

            interval[0] = proj_1 + (proj_0 - proj_1) * (distance[1] / (distance[1] - distance[0]));

            interval[1] = proj_2 + (proj_0 - proj_2) * (distance[2] / (distance[2] - distance[0]));
        }

        if (segment_is_found && (interval[0] > interval[1]))
            std::swap(interval[0], interval[1]);

        return interval;
    }

    bool triangles_intersects_in_3d(const triangle_t<T>& other_triangle) const {
        if (plane_.planes_are_equal(other_triangle.plane_))
            return triangles_intersects_in_2d(other_triangle);

        if (triangle_points_have_one_sign_dist_to_plane(other_triangle.plane_) ||
            other_triangle.triangle_points_have_one_sign_dist_to_plane(plane_)) 
            return false;

        segment_t<T> line_of_plane_intersection = plane_.intersection_of_planes(other_triangle.plane_);
        std::array<T, 3> dist_from_t2_points_to_t1_plane{};
        std::array<T, 3> dist_from_t1_points_to_t2_plane{};

        dist_from_t2_points_to_t1_plane[0] = plane_.distance_between_point_and_plane(other_triangle.a_);
        dist_from_t2_points_to_t1_plane[1] = plane_.distance_between_point_and_plane(other_triangle.b_);
        dist_from_t2_points_to_t1_plane[2] = plane_.distance_between_point_and_plane(other_triangle.c_);

        Compare::interval<T> first_interval = other_triangle.calculate_intersection_interval(line_of_plane_intersection, 
                                                                                             dist_from_t2_points_to_t1_plane);
            
        dist_from_t1_points_to_t2_plane[0] = other_triangle.plane_.distance_between_point_and_plane(a_);
        dist_from_t1_points_to_t2_plane[1] = other_triangle.plane_.distance_between_point_and_plane(b_);
        dist_from_t1_points_to_t2_plane[2] = other_triangle.plane_.distance_between_point_and_plane(c_);

        Compare::interval<T> second_interval = calculate_intersection_interval(line_of_plane_intersection, 
                                                                               dist_from_t1_points_to_t2_plane);

        return Compare::number_intervals_overloap(first_interval, second_interval);
    }

    bool triangles_intersects_in_2d(const triangle_t<T>& other_triangle) const {
        if (segments_of_triangles_intersects(other_triangle))
            return true;

        return point_lies_inside_triangle(other_triangle.a_) || other_triangle.point_lies_inside_triangle(a_);
    }

    bool segments_of_triangles_intersects(const triangle_t<T>& other_triangle) const {
        if (segment_ab_.segments_intersects(other_triangle.segment_ab_) ||
            segment_ab_.segments_intersects(other_triangle.segment_ca_) ||
            segment_ab_.segments_intersects(other_triangle.segment_bc_) ||

            segment_ca_.segments_intersects(other_triangle.segment_ab_) ||
            segment_ca_.segments_intersects(other_triangle.segment_ca_) ||
            segment_ca_.segments_intersects(other_triangle.segment_bc_) ||

            segment_bc_.segments_intersects(other_triangle.segment_ab_) ||
            segment_bc_.segments_intersects(other_triangle.segment_ca_) ||
            segment_bc_.segments_intersects(other_triangle.segment_bc_))
            return true;

        return false;
    }

    bool triangle_points_have_one_sign_dist_to_plane(const plane_t<T>& plane) const {
        T dist_1 = plane.distance_between_point_and_plane(a_);
        T dist_2 = plane.distance_between_point_and_plane(b_);
        T dist_3 = plane.distance_between_point_and_plane(c_);

        // If the points of a triangle have the same sign of the distance to the plane of another triangle, 
        // then such two triangles don't intersect

        if (dist_1 > 0 && dist_2 > 0 && dist_3 > 0) 
            return true;
        if (dist_1 < 0 && dist_2 < 0 && dist_3 < 0) 
            return true;

        return false;
    }

    bool point_lies_inside_triangle(const point_t<T>& point) const {
        if (!point.valid())
            return false;

        // Works only if point into plane of triangle
        if (!plane_.point_lies_on_plane(point))
            return false;

        // Point O is a point inside triangle
        vector_t<T> ao{a_, point};
        vector_t<T> bo{b_, point};
        vector_t<T> co{c_, point};

        vector_t<T> normal_vector_1 = ao.cross_product(segment_ab_.get_dir_vector());
        vector_t<T> normal_vector_2 = bo.cross_product(segment_bc_.get_dir_vector());
        vector_t<T> normal_vector_3 = co.cross_product(segment_ca_.get_dir_vector());

        if (normal_vector_1.get_z() * normal_vector_2.get_z() < 0 ||
            normal_vector_1.get_z() * normal_vector_3.get_z() < 0 ||
            normal_vector_2.get_z() * normal_vector_3.get_z() < 0) 
            return false;
        if (normal_vector_1.get_x() * normal_vector_2.get_x() < 0 ||
            normal_vector_1.get_x() * normal_vector_3.get_x() < 0 ||
            normal_vector_2.get_x() * normal_vector_3.get_x() < 0)
            return false;
        if (normal_vector_1.get_y() * normal_vector_2.get_y() < 0 ||
            normal_vector_1.get_y() * normal_vector_3.get_y() < 0 ||
            normal_vector_2.get_y() * normal_vector_3.get_y() < 0)
            return false;

        return true;
    }

    public:
    void print() const {
        std::cout << "Triangle:" << std::endl;
        std::cout << "Points of triangle:" << std::endl;
        a_.print(); 
        b_.print();
        c_.print();
        std::cout << "Segments:" << std::endl;
        segment_ab_.print();
        segment_ca_.print();
        segment_bc_.print();
        std::cout << "Plane:" << std::endl;
        plane_.print();
        std::cout << "Number of triangle:" << number_ << std::endl;
    }
};

} // namespace Geom_objects

#endif // TRIANGLE_HPP
