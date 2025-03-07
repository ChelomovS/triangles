#ifndef BOUNDING_BOX_HPP
#define BOUNDING_BOX_HPP

#include <array> 
#include <variant> 
#include <limits>
#include <cassert>

#include "point.hpp" 
#include "segment.hpp"
#include "triangle.hpp"
#include "double_compare.hpp"
#include "polygons.hpp"

namespace Geom_objects {

const size_t number_of_edges = 3;

template <typename T>
class AABB_t {
    private: 
    point_t<T> middle_point_;
    std::array<T, number_of_edges> box_edges_;

    bool check_axis(const vector_t<T>& axis, const point_t<T>& a, const point_t<T>& b, const point_t<T>& c) const;

    bool check_segment_intersection(const segment_t<T>& segment) const;

    public:
    AABB_t(const point_t<T>& middle_point, const std::array<T, number_of_edges>& box_edges)
        : middle_point_{middle_point}, box_edges_{box_edges} {}
    
    point_t<T> get_middle_point() const;
    std::array<T, number_of_edges> get_box_edges() const;
    T get_box_x_edge() const;
    T get_box_y_edge() const;
    T get_box_z_edge() const;

    bool is_point_inside_box(const point_t<T>& point) const;
    bool is_polygon_inside_box(const polygon_t<T>& polygon) const;
    bool is_polygon_part_inside_box(const polygon_t<T>& polygon) const;

    bool check_triangle_intersection(const triangle_t<T>& triangle) const;
    void get_min_max(point_t<T>& min_pt, point_t<T>& max_pt) const;
};

template <typename T>
point_t<T> AABB_t<T>::get_middle_point() const { 
    return middle_point_; 
}

template <typename T>
std::array<T, number_of_edges> AABB_t<T>::get_box_edges() const { 
    return box_edges_; 
}

template <typename T>
T AABB_t<T>::get_box_x_edge() const { 
    return box_edges_[0]; 
}

template <typename T>
T AABB_t<T>::get_box_y_edge() const { 
    return box_edges_[1]; 
}

template <typename T>
T AABB_t<T>::get_box_z_edge() const { 
    return box_edges_[2]; 
}

template <typename T>
bool AABB_t<T>::is_point_inside_box(const point_t<T>& point) const {
    T x_max = middle_point_.get_x() + box_edges_[0];
    T x_min = middle_point_.get_x() - box_edges_[0];
    T y_max = middle_point_.get_y() + box_edges_[1];
    T y_min = middle_point_.get_y() - box_edges_[1];
    T z_max = middle_point_.get_z() + box_edges_[2];
    T z_min = middle_point_.get_z() - box_edges_[2];

    return (point.get_x() > x_min) && (point.get_x() < x_max) &&
           (point.get_y() > y_min) && (point.get_y() < y_max) &&
           (point.get_z() > z_min) && (point.get_z() < z_max);
}

template <typename T>
bool AABB_t<T>::is_polygon_inside_box(const polygon_t<T>& polygon) const {
    switch (polygon.index()) {
        case 0: { // point_t 
            return is_point_inside_box(std::get<point_t<T>>(polygon));
        }

        case 1: { // segment_t
            auto segment = std::get<segment_t<T>>(polygon);

            return is_point_inside_box(segment.get_beg_point()) &&
                   is_point_inside_box(segment.get_end_point());
        }

        case 2: { // triangle_t
            auto triangle = std::get<triangle_t<T>>(polygon);

            return is_point_inside_box(triangle.get_a()) &&
                   is_point_inside_box(triangle.get_b()) &&
                   is_point_inside_box(triangle.get_c());
        }

        default: {
            assert(0 && "problem :(");
            return false; 
        }
    }
}

template <typename T>
bool AABB_t<T>::is_polygon_part_inside_box(const polygon_t<T>& polygon) const {
    switch (polygon.index()) {
        case 0: { // point_t
            return is_point_inside_box(std::get<point_t<T>>(polygon));
        }

        case 1: { // segment_t 
            auto segment = std::get<segment_t<T>>(polygon);

            return is_point_inside_box(segment.get_beg_point()) ||
                   is_point_inside_box(segment.get_end_point()) ||
                   check_segment_intersection(segment);
        }

        case 2: { // triangle_t
            auto triangle = std::get<triangle_t<T>>(polygon);

            if (is_point_inside_box(triangle.get_a()) ||
                is_point_inside_box(triangle.get_b()) ||
                is_point_inside_box(triangle.get_c())) 
                return true;

            return check_segment_intersection(triangle.get_segment_ab()) ||
                   check_segment_intersection(triangle.get_segment_bc()) ||
                   check_segment_intersection(triangle.get_segment_ca()) ||
                   check_triangle_intersection(triangle);
        }

        default: {
            assert(0 && "problem :(");
            return false;
        }
    }
}

template <typename T>
bool AABB_t<T>::check_triangle_intersection(const triangle_t<T>& triangle) const {
    auto a = triangle.get_a();
    auto b = triangle.get_b();
    auto c = triangle.get_c();

    for (size_t axis = 0; axis < 3; ++axis) {
        T triangle_min = std::min({a[axis], b[axis], c[axis]}); 
        T triangle_max = std::max({a[axis], b[axis], c[axis]});

        T box_min = middle_point_[axis] - box_edges_[axis];
        T box_max = middle_point_[axis] + box_edges_[axis];
        
        // if projections don't intersect,
        // there is no intersection
        if (triangle_max < box_min || triangle_min > box_max) 
            return false;
    }

    vector_t<T> normal = triangle.get_plane().get_normal_vector().get_normalized();

    if (!check_axis(normal, a, b, c)) 
        return false;

    vector_t<T> aabb_edges[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    vector_t<T> triangle_edges[3] = {b - a, c - b, a - c};

    for (auto aabb_edge : aabb_edges) {
        for (auto triangle_edge : triangle_edges) {
            vector_t<T> axis = aabb_edge.cross_product(triangle_edge);
            if (axis.is_null_vector()) 
                continue;

            if (!check_axis(axis.get_normalized(), a, b, c)) 
                return false;
        }
    }

    return true;
}

template <typename T>
void AABB_t<T>::get_min_max(point_t<T>& min_pt, point_t<T>& max_pt) const {
    min_pt = {
        middle_point_.get_x() - box_edges_[0],
        middle_point_.get_y() - box_edges_[1],
        middle_point_.get_z() - box_edges_[2]};

    max_pt = {
        middle_point_.get_x() + box_edges_[0],
        middle_point_.get_y() + box_edges_[1],
        middle_point_.get_z() + box_edges_[2]};
}

template <typename T>
bool AABB_t<T>::check_axis(const vector_t<T>& axis,
                           const point_t<T>& a,
                           const point_t<T>& b,
                           const point_t<T>& c) const {
    // triangle projection
    T triangle_proj_a = axis.dot_product(as_vector(a));
    T triangle_proj_b = axis.dot_product(as_vector(b));
    T triangle_proj_c = axis.dot_product(as_vector(c));

    T triangle_min = std::min({triangle_proj_a, triangle_proj_b, triangle_proj_c});
    T triangle_max = std::max({triangle_proj_a, triangle_proj_b, triangle_proj_c});

    // box projection
    point_t<T> aabb_min, aabb_max;
    get_min_max(aabb_min, aabb_max);
    
    T b_proj_min = std::numeric_limits<T>::max();
    T b_proj_max = std::numeric_limits<T>::lowest();
    
    for (size_t i = 0; i < 8; ++i) {
        point_t<T> p = {
            (i & 1) ? aabb_max.get_x() : aabb_min.get_x(),
            (i & 2) ? aabb_max.get_y() : aabb_min.get_y(),
            (i & 4) ? aabb_max.get_z() : aabb_min.get_z()
        };
        
        T projection = p.get_x() * axis.get_x() + p.get_y() * axis.get_y() + p.get_z() * axis.get_z();
        b_proj_min = std::min(b_proj_min, projection);
        b_proj_max = std::max(b_proj_max, projection);
    }

    return !(triangle_max < b_proj_min || triangle_min > b_proj_max);
}

template <typename T>
bool AABB_t<T>::check_segment_intersection(const segment_t<T>& segment) const {
    point_t<T> aabb_min, aabb_max;
    get_min_max(aabb_min, aabb_max);
    
    T t_min = 0.0;
    T t_max = 1.0;
    
    vector_t<T> dir_vector = segment.get_dir_vector();

    vector_t<T> inv_dir = vector_t<T>(
        Compare::is_equal(dir_vector.get_x(), 0.0) ? std::numeric_limits<T>::infinity() : 1.0 / dir_vector.get_x(),
        Compare::is_equal(dir_vector.get_y(), 0.0) ? std::numeric_limits<T>::infinity() : 1.0 / dir_vector.get_y(),
        Compare::is_equal(dir_vector.get_z(), 0.0) ? std::numeric_limits<T>::infinity() : 1.0 / dir_vector.get_z()
    );

    for (size_t axis = 0; axis < 3; ++axis) {
        T t1 = (aabb_min[axis] - segment.get_beg_point()[axis]) * inv_dir[axis];
        T t2 = (aabb_max[axis] - segment.get_beg_point()[axis]) * inv_dir[axis];
        
        if (inv_dir[axis] < 0.0) 
            std::swap(t1, t2);
        
        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);
        
        if (t_min > t_max) 
            return false;
    }
    
    return (Compare::is_greater_or_equal(t_min, 0.0) && Compare::is_less_or_equal(t_min, 1.0)) ||
           (Compare::is_greater_or_equal(t_max, 0.0) && Compare::is_less_or_equal(t_max, 1.0)) ||
           (Compare::is_greater_or_equal(t_max, 0.0) && Compare::is_less_or_equal(t_min, 1.0));
}

} // namespace Geom_objects

#endif // BOUNDING_BOX_HPP 
