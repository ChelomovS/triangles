#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

#include "point.hpp"
#include "plane.hpp"
#include "segment.hpp"

namespace Triangle {
        
enum class Triangle_type { point, segment, triangle };

class triangle_t {
    private:
    Point::point_t a_;
    Point::point_t b_;
    Point::point_t c_; 
    Segment::segment_t segment_ab_;
    Segment::segment_t segment_bc_;    
    Segment::segment_t segment_ca_;
    Plane::plane_t plane_;
    Triangle_type triangle_type_;
    size_t number_;

    public:
    triangle_t(Point::point_t a, Point::point_t b, Point::point_t c): 
    a_{a}, b_{b}, c_{c}, segment_ab_{a, b}, segment_bc_{b, c}, segment_ca_{c, a},
    plane_{a, b, c} { 
        triangle_type_ = get_type_of_triangle_in_ctor();
        if (triangle_type_ == Triangle_type::segment) {
            get_degenerate_triangle_segment();
        }
    }

    triangle_t(Point::point_t a, Point::point_t b, Point::point_t c, size_t number): 
    a_{a}, b_{b}, c_{c}, segment_ab_{a, b}, segment_bc_{b, c}, segment_ca_{c, a},
    plane_(a, b, c), number_(number) { 
        triangle_type_ = get_type_of_triangle_in_ctor();
        if (triangle_type_ == Triangle_type::segment) {
            get_degenerate_triangle_segment();
        }
    }

    private:
    void get_degenerate_triangle_segment();

    public:
    Point::point_t get_a() const { return a_; }
    Point::point_t get_b() const { return b_; }
    Point::point_t get_c() const { return c_; }
    Segment::segment_t get_segment_ab() const { return segment_ab_; } 
    Segment::segment_t get_segment_bc() const { return segment_bc_; }
    Segment::segment_t get_segment_ca() const { return segment_ca_; }
    Plane::plane_t get_plane() const { return plane_; };
    size_t get_number() const { return number_; };
    Triangle_type get_type_of_triangle() const { return triangle_type_; }

    Triangle_type get_type_of_triangle_in_ctor() const;
    bool check_degenerate_cases(const triangle_t& other_figure) const;
    Compare::interval calculate_intersection_interval(const Segment::segment_t& line,
                                                      const std::array<double, 3>& distance) const;
    bool triangles_intersects_in_3d(const triangle_t& other_triangle) const;
    bool triangles_intersects_in_2d(const triangle_t& other_triangle) const;
    bool segments_of_triangles_intersects(const triangle_t& other_triangle) const;
    bool triangle_points_have_one_sign_dist_to_plane(const Plane::plane_t& plane) const;
    bool point_lies_inside_triangle(const Point::point_t& point) const;
    void print() const;                                      
};

} // namespace Triangle


#endif // TRIANGLE_HPP