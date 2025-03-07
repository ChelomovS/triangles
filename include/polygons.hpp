#ifndef POLYGONS_HPP
#define POLYGONS_HPP

#include <variant>

#include "point.hpp"
#include "segment.hpp"
#include "triangle.hpp"

namespace Geom_objects {

template <typename T>
using polygon_t = std::variant<point_t<T>, segment_t<T>, triangle_t<T>>;

template <typename T>
size_t get_number(const polygon_t<T>& polygon) {
    return std::visit([](const auto& obj) -> size_t { return obj.get_number(); }, polygon);
}

template <typename T>
segment_t<T> get_segment_from_collinear_points(const segment_t<T>& segment_ab,
                                               const segment_t<T>& segment_ca,
                                               const segment_t<T>& segment_bc) {
    point_t<T> a = segment_ab.get_beg_point();
    point_t<T> b = segment_bc.get_beg_point();

    if (segment_ca.point_lies_on_segment(b))
        return segment_ca;

    else if (segment_bc.point_lies_on_segment(a))
        return segment_bc;

    return segment_ab;
}

template <typename T>
polygon_t<T> make_geometric_primitive(const point_t<T>& a,
                                      const point_t<T>& b,
                                      const point_t<T>& c) {
    if (a.equal(b) && b.equal(c)) {
        return a;
    } 

    segment_t<T> segment_ab(a, b, a.get_number());
    segment_t<T> segment_ca(c, a, c.get_number());
    segment_t<T> segment_bc(b, c, b.get_number());

    if (segment_ab.get_dir_vector().vectors_are_collinear(segment_ca.get_dir_vector()) ||
        segment_bc.get_dir_vector().vectors_are_collinear(segment_ca.get_dir_vector()))
        return get_segment_from_collinear_points(segment_ab, segment_ca, segment_bc);
    
    return triangle_t{a, b, c, a.get_number()};
}  

template <typename T>
bool check_figures_intersection(const polygon_t<T>& first, const polygon_t<T>& second) {
    switch (first.index()) {
        case 0: { // point_t
            auto point_1 = std::get<point_t<T>>(first);
            switch (second.index()) {
                case 0: { // point_t
                    auto point_2 = std::get<point_t<T>>(second);
                    return point_1.equal(point_2);
                }
                case 1: { // segment_t
                    auto segment = std::get<segment_t<T>>(second);
                    return segment.point_lies_on_segment(point_1);
                }
                case 2: { // triangle_t
                    auto triangle = std::get<triangle_t<T>>(second);
                    return triangle.point_lies_inside_triangle(point_1);
                }
            }
            break;
        }

        case 1: { // segment_t
            auto segment_1 = std::get<segment_t<T>>(first);
            switch (second.index()) {
                case 0: { // point_t
                    auto point = std::get<point_t<T>>(second);
                    return segment_1.point_lies_on_segment(point);
                }
                case 1: { // segment_t
                    auto segment_2 = std::get<segment_t<T>>(second);
                    return segment_1.segments_intersects(segment_2) || segment_1.segments_overloap(segment_2);
                }
                case 2: { // triangle_t
                    auto triangle = std::get<triangle_t<T>>(second);
                    return triangle.triangle_intersect_segment(segment_1);
                }
            }
            break;
        }
        case 2: { // triangle_t
            auto triangle_1 = std::get<triangle_t<T>>(first);
            switch (second.index()) {
                case 0: { // point_t
                    auto point = std::get<point_t<T>>(second);
                    return triangle_1.point_lies_inside_triangle(point);
                }
                case 1: { // segment_t
                    auto segment = std::get<segment_t<T>>(second);
                    return triangle_1.triangle_intersect_segment(segment);
                }
                case 2: { // triangle_t
                    auto triangle_2 = std::get<triangle_t<T>>(second);
                    return triangle_2.triangles_intersects_in_3d(triangle_1);
                }
            }
            break;
        }
    }
    return false;
}

} // namespace Geom_objects

#endif // POLYGONS_HPP 
