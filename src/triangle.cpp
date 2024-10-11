#include <iostream>

#include "triangle.hpp"
#include "plane.hpp"

void Triangle::triangle_t::get_degenerate_triangle_segment() {
    if (segment_ca_.point_lies_on_segment(b_)) {
        std::swap(b_, a_);
        std::swap(a_, c_);
        segment_ab_ = {a_, b_};
    }

    else if (segment_bc_.point_lies_on_segment(a_)) {
        std::swap(a_, b_);
        std::swap(b_, c_);        
        segment_ab_ = {a_, b_};
    }

    else if (segment_ab_.point_lies_on_segment(c_)) 
        return;
}

Triangle::Triangle_type Triangle::triangle_t::get_type_of_triangle_in_ctor() const {
    if (a_.equal(b_) && b_.equal(c_))
        return Triangle_type::point;

    if (segment_ab_.get_dir_vector().vectors_are_collinear(segment_ca_.get_dir_vector()))
        return Triangle_type::segment;

    return Triangle_type::triangle;
}

bool Triangle::triangle_t::check_degenerate_cases(const Triangle::triangle_t& other_figure) const { 
    if (triangle_type_ == Triangle::Triangle_type::point && 
        other_figure.triangle_type_ == Triangle::Triangle_type::point)
        return a_.equal(other_figure.a_);

    if (triangle_type_ == Triangle::Triangle_type::point &&
        other_figure.triangle_type_ == Triangle::Triangle_type::segment)
        return (other_figure.segment_ab_.point_lies_on_segment(a_));
    
    if (triangle_type_ == Triangle::Triangle_type::segment &&
        other_figure.triangle_type_ == Triangle::Triangle_type::point)
        return (segment_ab_.point_lies_on_segment(other_figure.a_));

    if (triangle_type_ == Triangle::Triangle_type::segment &&
        other_figure.triangle_type_ == Triangle::Triangle_type::segment)
        return (segment_ab_.segments_intersects(other_figure.segment_ab_));

    if (triangle_type_ == Triangle::Triangle_type::triangle &&
        other_figure.triangle_type_ == Triangle::Triangle_type::segment)
        return (segment_ab_.segments_intersects(other_figure.segment_ab_) ||
                segment_ca_.segments_intersects(other_figure.segment_ab_) ||
                segment_bc_.segments_intersects(other_figure.segment_ab_) ||
                point_lies_inside_triangle(plane_.intersect_plane_and_segment(other_figure.segment_ab_)));

    if (triangle_type_ == Triangle::Triangle_type::segment &&
        other_figure.triangle_type_ == Triangle::Triangle_type::triangle)
        return (other_figure.segment_ab_.segments_intersects(segment_ab_) ||
                other_figure.segment_ca_.segments_intersects(segment_ab_) ||
                other_figure.segment_bc_.segments_intersects(segment_ab_) ||
                other_figure.point_lies_inside_triangle(other_figure.plane_.intersect_plane_and_segment(segment_ab_)));

    if (triangle_type_ == Triangle::Triangle_type::point &&
        other_figure.triangle_type_ == Triangle::Triangle_type::triangle)
        return other_figure.point_lies_inside_triangle(a_);

    else 
        return point_lies_inside_triangle(other_figure.a_);
}

Compare::interval Triangle::triangle_t::calculate_intersection_interval(const Segment::segment_t& line,
                                                                        const std::array<double, 3>& distance) const {
    Vector::vector_t first_vector  = {a_.get_x() - line.get_beg_point().get_x(),
                                      a_.get_y() - line.get_beg_point().get_y(),
                                      a_.get_z() - line.get_beg_point().get_z()};

    Vector::vector_t second_vector = {b_.get_x() - line.get_beg_point().get_x(),
                                      b_.get_y() - line.get_beg_point().get_y(),
                                      b_.get_z() - line.get_beg_point().get_z()};

    Vector::vector_t third_vector  = {c_.get_x() - line.get_beg_point().get_x(),
                                      c_.get_y() - line.get_beg_point().get_y(),
                                      c_.get_z() - line.get_beg_point().get_z()};

    double proj_0 = first_vector.dot_product(line.get_dir_vector());
    double proj_1 = second_vector.dot_product(line.get_dir_vector());
    double proj_2 = third_vector.dot_product(line.get_dir_vector());

    Compare::interval interval{};

    bool segment_is_found = false;

    if ((Compare::is_greater_or_equal(distance[0] * distance[1], 0)) && 
         !Compare::is_equal(distance[1] - distance[2], 0) && 
         !Compare::is_equal(distance[0] - distance[2], 0)) {
        segment_is_found = true;

        interval[0] = proj_0 + (proj_2 - proj_0) * (distance[0] / (distance[0] - distance[2]));

        interval[1] = proj_1 + (proj_2 - proj_1) * (distance[1] / (distance[1] - distance[2]));
    }
    
    else if (Compare::is_greater_or_equal(distance[0] * distance[2], 0) && 
             !Compare::is_equal(distance[0] - distance[1], 0) && 
             !Compare::is_equal(distance[2] - distance[1], 0)) {
        segment_is_found = true;

        interval[0] = proj_0 + (proj_1 - proj_0) * (distance[0] / (distance[0] - distance[1]));

        interval[1] = proj_2 + (proj_1 - proj_2) * (distance[2] / (distance[2] - distance[1]));
    }

    else if ((Compare::is_greater_or_equal(distance[1] * distance[2], 0)) && 
              !Compare::is_equal(distance[1] - distance[0], 0) && 
              !Compare::is_equal(distance[2] - distance[0], 0)) {
        segment_is_found = true;

        interval[0] = proj_1 + (proj_0 - proj_1) * (distance[1] / (distance[1] - distance[0]));

        interval[1] = proj_2 + (proj_0 - proj_2) * (distance[2] / (distance[2] - distance[0]));
    }

    if (segment_is_found && (interval[0] > interval[1]))
        std::swap(interval[0], interval[1]);

    return interval;
}

bool Triangle::triangle_t::triangles_intersects_in_3d(const Triangle::triangle_t& other_triangle) const {
    Triangle::Triangle_type type_of_first  = triangle_type_;
    Triangle::Triangle_type type_of_second = other_triangle.triangle_type_;

    if (type_of_first  != Triangle::Triangle_type::triangle ||
        type_of_second != Triangle::Triangle_type::triangle)
        return check_degenerate_cases(other_triangle);

    if (plane_.planes_are_equal(other_triangle.plane_))
        return triangles_intersects_in_2d(other_triangle);

    if (triangle_points_have_one_sign_dist_to_plane(other_triangle.plane_) ||
        other_triangle.triangle_points_have_one_sign_dist_to_plane(plane_)) 
        return false;

    Segment::segment_t line_of_plane_intersection = plane_.intersection_of_planes(other_triangle.plane_);
    std::array<double, 3> dist_from_t2_points_to_t1_plane{};
    std::array<double, 3> dist_from_t1_points_to_t2_plane{};

    dist_from_t2_points_to_t1_plane[0] = plane_.distance_between_point_and_plane(other_triangle.a_);
    dist_from_t2_points_to_t1_plane[1] = plane_.distance_between_point_and_plane(other_triangle.b_);
    dist_from_t2_points_to_t1_plane[2] = plane_.distance_between_point_and_plane(other_triangle.c_);

    Compare::interval first_interval = other_triangle.calculate_intersection_interval(line_of_plane_intersection, 
                                                                                      dist_from_t2_points_to_t1_plane);

    dist_from_t1_points_to_t2_plane[0] = other_triangle.plane_.distance_between_point_and_plane(a_);
    dist_from_t1_points_to_t2_plane[1] = other_triangle.plane_.distance_between_point_and_plane(b_);
    dist_from_t1_points_to_t2_plane[2] = other_triangle.plane_.distance_between_point_and_plane(c_);
 
    Compare::interval second_interval = calculate_intersection_interval(line_of_plane_intersection, 
                                                                        dist_from_t1_points_to_t2_plane);


    return Compare::number_intervals_overloap(first_interval, second_interval);
}

bool Triangle::triangle_t::triangles_intersects_in_2d(const Triangle::triangle_t& other_triangle) const {
    if (segments_of_triangles_intersects(other_triangle))
        return true;

    return point_lies_inside_triangle(other_triangle.a_) || other_triangle.point_lies_inside_triangle(a_);
}

bool Triangle::triangle_t::segments_of_triangles_intersects(const Triangle::triangle_t& other_triangle) const {
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

bool Triangle::triangle_t::point_lies_inside_triangle(const Point::point_t& point) const {
    // Works only if point into plane of triangle
    if (!plane_.point_lies_on_plane(point))
        return false;

    // Point O is point inside triangle
    Vector::vector_t ao{a_, point};
    Vector::vector_t bo{b_, point};
    Vector::vector_t co{c_, point};

    double normal_vector_1_module = ao.cross_product_module(segment_ab_.get_dir_vector());
    double normal_vector_2_module = bo.cross_product_module(segment_bc_.get_dir_vector());
    double normal_vector_3_module = co.cross_product_module(segment_ca_.get_dir_vector());

    if (normal_vector_1_module * normal_vector_2_module < 0 ||
        normal_vector_1_module * normal_vector_3_module < 0 ||
        normal_vector_2_module * normal_vector_3_module < 0) 
        return false;

    return true;
}

bool Triangle::triangle_t::triangle_points_have_one_sign_dist_to_plane(const Plane::plane_t& plane) const {
    double dist_1 = plane.distance_between_point_and_plane(a_);
    double dist_2 = plane.distance_between_point_and_plane(b_);
    double dist_3 = plane.distance_between_point_and_plane(c_);

    // If the points of a triangle have the same sign of the distance to the plane of another triangle, 
    // then such two triangles don't intersect

    if (dist_1 > 0 && dist_2 > 0 && dist_3 > 0) return true;
    if (dist_1 < 0 && dist_2 < 0 && dist_3 < 0) return true;
    return false;
}

void Triangle::triangle_t::print() const {
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