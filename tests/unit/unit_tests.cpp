#include <gtest/gtest.h>

#include "polygons.hpp"
#include "point.hpp"
#include "vector.hpp"
#include "segment.hpp"
#include "plane.hpp"
#include "triangle.hpp"

TEST(POINT_FUNCTIONS, point_is_not_valid) {
    Geom_objects::point_t<double> p{NAN, NAN, NAN};
    ASSERT_EQ(p.valid(), false);
}

TEST(POINT_FUNCTIONS, point_is_valid) {
    Geom_objects::point_t<double> p{1.0, 2.1, 3.8};
    ASSERT_EQ(p.valid(), true);
}

TEST(POINT_FUNCTIONS, distance_between_points_1) {
    Geom_objects::point_t<double> p1{0.0, 0.0, 3.0};
    Geom_objects::point_t<double> p2{0.0, 0.0, 4.0};
    double distance_between_points = p1.distance_between_points(p2);
    ASSERT_DOUBLE_EQ(distance_between_points, 1.0);
}

TEST(POINT_FUNCTIONS, distance_between_points_2) {
    Geom_objects::point_t<double> p1{0.0, 3.0, 0.0};
    Geom_objects::point_t<double> p2{0.0, 0.0, 4.0};
    double distance_between_points = p1.distance_between_points(p2);
    ASSERT_DOUBLE_EQ(distance_between_points, 5.0);
}

TEST(VECTOR_FUNCTIONS, dot_product_1) {
    Geom_objects::vector_t<double> v1{-5.0, 70.0, 80.0};
    Geom_objects::vector_t<double> v2{-90.0, 100.0, 100.0};
    double dot_product = v1.dot_product(v2);
    ASSERT_DOUBLE_EQ(dot_product, 15450.0);
}

TEST(VECTOR_FUNCTIONS, dot_product_2) {
    Geom_objects::vector_t<double> v1{100.0, 100.0, 100.0};
    Geom_objects::vector_t<double> v2{100.0, -100.0, 0.0};
    double dot_product = v1.dot_product(v2);
    ASSERT_DOUBLE_EQ(dot_product, 0.0);
}

TEST(VECTOR_FUNCTIONS, cross_product_vector_1) {
    Geom_objects::vector_t<double> v1{-90.0, 100.0, 105.0};
    Geom_objects::vector_t<double> v2{90.0, 1.0, 4.0};
    Geom_objects::vector_t<double> cross_product = v1.cross_product(v2);
    ASSERT_EQ(cross_product.is_equal({295.0, 9810.0, -9090.0}), true);
}

TEST(VECTOR_FUNCTIONS, vectors_are_collinear_1) {
    Geom_objects::vector_t<double> v1{-4.0, 5.0, 0.0};
    Geom_objects::vector_t<double> v2{9.0, 9.0, 9.0};
    ASSERT_EQ(v1.vectors_are_collinear(v2), false);
}

TEST(VECTOR_FUNCTIONS, vectors_are_collinear_2) {
    Geom_objects::vector_t<double> v1{15.0, 15.0, 15.0};
    Geom_objects::vector_t<double> v2{-1.0, -1.0, -1.0};
    ASSERT_EQ(v1.vectors_are_collinear(v2), true);
}

TEST(SEGMENT_FUNCTIONS, point_lies_on_segment_1) {
    Geom_objects::point_t<double> point{0.0, 0.0, 0.5};
    Geom_objects::point_t<double> beg_segment{0.0, 0.0, 1.0};
    Geom_objects::point_t<double> end_segment{0.0, 0.0, 0.0};
    Geom_objects::segment_t segment{beg_segment, end_segment};
    ASSERT_EQ(segment.point_lies_on_segment(point), true);
}

TEST(SEGMENT_FUNCTIONS, point_lies_on_segment_2) {
    Geom_objects::point_t<double> point{0.5, -0.5, 1.0};
    Geom_objects::point_t<double> beg_segment{0.0, 0.0, 2.0};
    Geom_objects::point_t<double> end_segment{1.0, -1.0, 0.0};
    Geom_objects::segment_t segment{beg_segment, end_segment};
    ASSERT_EQ(segment.point_lies_on_segment(point), true);
}

TEST(SEGMENT_FUNCTIONS, segment_length) {
    Geom_objects::point_t<double> beg_segment{0.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_segment{13.0, 18.0, 6.0};
    Geom_objects::segment_t<double> segment{beg_segment, end_segment};
    ASSERT_DOUBLE_EQ(segment.get_length(), 23.0);
}

TEST(SEGMENT_FUNCTIONS, segments_intersection_1) {
    Geom_objects::point_t<double> beg_segment_1{3.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_segment_1{0.0, 0.0, 2.0};   
    Geom_objects::point_t<double> beg_segment_2{2.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_segment_2{0.0, 0.0, 3.0};
    Geom_objects::segment_t<double> segment_1{beg_segment_1, end_segment_1};
    Geom_objects::segment_t<double> segment_2{beg_segment_2, end_segment_2};
    ASSERT_EQ(segment_1.segments_intersects(segment_2), true);
}
 
TEST(SEGMENT_FUNCTIONS, segments_intersection_2) {
    Geom_objects::point_t<double> beg_segment_1{2.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_segment_1{0.0, 0.0, 3.0};   
    Geom_objects::point_t<double> beg_segment_2{2.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_segment_2{0.0, 0.0, 3.0};
    Geom_objects::segment_t<double> segment_1{beg_segment_1, end_segment_1};
    Geom_objects::segment_t<double> segment_2{beg_segment_2, end_segment_2};
    ASSERT_EQ(segment_1.segments_intersects(segment_2), true);
}

TEST(SEGMENT_FUNCTIONS, segments_intersection_3) {
    Geom_objects::point_t<double> beg_segment_1{-1.0, -1.0, 0.0};
    Geom_objects::point_t<double> end_segment_1{0.0, 0.0, 3.0};   
    Geom_objects::point_t<double> beg_segment_2{-3.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_segment_2{-1.0, -1.0, 0.0};
    Geom_objects::segment_t<double> segment_1{beg_segment_1, end_segment_1};
    Geom_objects::segment_t<double> segment_2{beg_segment_2, end_segment_2};
    ASSERT_EQ(segment_1.segments_intersects(segment_2), true);
}

TEST(SEGMENT_FUNCTIONS, segment_and_line_intersection_1) {
    Geom_objects::point_t<double> beg_line_1{3.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_line_1{0.0, 0.0, 2.0};   
    Geom_objects::point_t<double> beg_line_2{2.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_line_2{0.0, 0.0, 3.0};
    Geom_objects::segment_t<double> line_1{beg_line_1, end_line_1};
    Geom_objects::segment_t<double> line_2{beg_line_2, end_line_2};
    ASSERT_EQ(line_1.line_and_line_intersects(line_2).equal({1.2, 0.0, 1.2}), true);
}

TEST(SEGMENT_FUNCTIONS, line_and_line_intersection_2) {
    Geom_objects::point_t<double> beg_line_1{-6.0, 0.0, 0.0}; 
    Geom_objects::point_t<double> end_line_1{0.0, -6.0, 0.0};
    Geom_objects::point_t<double> beg_line_2{0.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_line_2{-5.0, -5.0, 0.0};
    Geom_objects::segment_t<double> line_1{beg_line_1, end_line_1};
    Geom_objects::segment_t<double> line_2{beg_line_2, end_line_2};
    ASSERT_EQ(line_1.line_and_line_intersects(line_2).equal({-3.0, -3.0, 0.0}), true);
}

TEST(SEGMENT_FUNCTIONS, point_lies_on_line_1) {
    Geom_objects::point_t<double> beg_point{0.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_point{1.0, 1.0, 1.0};
    Geom_objects::point_t<double> point{-100.0, -100.0, -100.0};
    Geom_objects::segment_t<double> line{beg_point, end_point};
    ASSERT_EQ(line.point_lies_on_line(point), true);
}

TEST(SEGMENT_FUNCTIONS, point_lies_on_line_2) {
    Geom_objects::point_t<double> beg_point{0.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_point{1.0, 1.0, 1.0};
    Geom_objects::point_t<double> point{0.5, 0.5, 0.5};
    Geom_objects::segment_t<double> line{beg_point, end_point};
    ASSERT_EQ(line.point_lies_on_line(point), true);
}

TEST(SEGMENT_FUNCTIONS, lines_are_concident_1) {
    Geom_objects::point_t<double> beg_point_1{1.0, 1.0, 1.0};
    Geom_objects::point_t<double> end_point_1{2.0, 2.0, 2.0};
    Geom_objects::point_t<double> beg_point_2{50.0, 50.0, 50.0};
    Geom_objects::point_t<double> end_point_2{100.0, 100.0, 100.0};
    Geom_objects::segment_t<double> line_1{beg_point_1, end_point_1};
    Geom_objects::segment_t<double> line_2{beg_point_2, end_point_2};
    ASSERT_EQ(line_1.lines_are_coincident(line_2), true);
}

TEST(SEGMENT_FUNCTIONS, lines_are_concident_2) {
    Geom_objects::point_t<double> beg_point_1{1.0, 1.5, 1.0};
    Geom_objects::point_t<double> end_point_1{2.0, 2.0, 2.0};
    Geom_objects::point_t<double> beg_point_2{50.0, 50.0, 50.0};
    Geom_objects::point_t<double> end_point_2{100.0, 100.0, 100.0};
    Geom_objects::segment_t<double> line_1{beg_point_1, end_point_1};
    Geom_objects::segment_t<double> line_2{beg_point_2, end_point_2};
    ASSERT_EQ(line_1.lines_are_coincident(line_2), false);
}

TEST(SEGMENT_FUNCTIONS, segments_overlope_1) {
    Geom_objects::point_t<double> beg_point_1{20.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_point_1{100.0, 0.0, 0.0};
    Geom_objects::point_t<double> beg_point_2{50.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_point_2{-50.0, 0.0, 0.0};
    Geom_objects::segment_t<double> segment_1{beg_point_1, end_point_1};
    Geom_objects::segment_t<double> segment_2{beg_point_2, end_point_2};
    ASSERT_EQ(segment_1.segments_overloap(segment_2), true);
}

TEST(SEGMENT_FUNCTIONS, segments_overlope_2) {
    Geom_objects::point_t<double> beg_point_1{0.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_point_1{50.0, 50.0, 50.0};
    Geom_objects::point_t<double> beg_point_2{-100.0, -100.0, -100.0};
    Geom_objects::point_t<double> end_point_2{100.0, 100.0, 100.0};
    Geom_objects::segment_t<double> segment_1{beg_point_1, end_point_1};
    Geom_objects::segment_t<double> segment_2{beg_point_2, end_point_2};
    ASSERT_EQ(segment_1.segments_overloap(segment_2), true);
}

TEST(SEGMENT_FUNCTIONS, segments_dont_overlope_1) {
    Geom_objects::point_t<double> beg_point_1{0.0, 0.0, -50.0};
    Geom_objects::point_t<double> end_point_1{0.0, 0.0, 50.0};
    Geom_objects::point_t<double> beg_point_2{50.0, 100.0, 0.0};
    Geom_objects::point_t<double> end_point_2{-50.0, 200.0, 0.0};
    Geom_objects::segment_t<double> segment_1{beg_point_1, end_point_1};
    Geom_objects::segment_t<double> segment_2{beg_point_2, end_point_2};
    ASSERT_EQ(segment_1.segments_overloap(segment_2), false);
}

TEST(SEGMENT_FUNCTIONS, segments_dont_overlope_2) {
    Geom_objects::point_t<double> beg_point_1{0.0, 0.0, 1.0};
    Geom_objects::point_t<double> end_point_1{0.0, 0.0, 2.0};
    Geom_objects::point_t<double> beg_point_2{0.0, 0.0, 0.0};
    Geom_objects::point_t<double> end_point_2{0.0, 0.0, 0.9};
    Geom_objects::segment_t<double> segment_1{beg_point_1, end_point_1};
    Geom_objects::segment_t<double> segment_2{beg_point_2, end_point_2};
    ASSERT_EQ(segment_1.segments_overloap(segment_2), false);
}

TEST(PLANE_FUNCTIONS, point_lies_on_plane_1) {
    Geom_objects::point_t<double> point{0.0, 0.0, 0.0};
    Geom_objects::plane_t<double> plane{{0.0, 1.0, 1.0}, {-2.5, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    ASSERT_EQ(plane.point_lies_on_plane(point), true);
}

TEST(PLANE_FUNCTIONS, point_lies_on_plane_2) {
    Geom_objects::point_t<double> point{0.1, 0.1, 0.1};
    Geom_objects::plane_t<double> plane{{0.0, 1.0, 1.0}, {-2.5, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    ASSERT_EQ(plane.point_lies_on_plane(point), true);
}

TEST(TRIANGLE_FUNCTIONS, triangles_intersectection_in_2d_1) {
    Geom_objects::triangle_t<double> triangle_1{{-4.0, 0.0, 0.0}, {0.0, -4.0, 0.0}, {-2.0, 0.0, 0.0}};
    Geom_objects::triangle_t<double> triangle_2{{2.0, 0.0, 0.0},  {3.0, 0.0, 0.0},  {0.0, 2.0, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_2d(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, triangles_intersectection_in_2d_2) {
    Geom_objects::triangle_t<double> triangle_1{{-4.0, 0.0, 0.0}, {0.0, -4.0, 0.0}, {-2.0, 0.0, 0.0}};
    Geom_objects::triangle_t<double> triangle_2{{3.0, 0.0, 0.0},  {-5.0, 0.0, 0.0}, {0.0, 2.0, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_2d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangles_intersectection_in_2d_3) {
    Geom_objects::triangle_t<double> triangle_1{{-4.0, 0.0, 0.0}, {0.0, -4.0, 0.0},  {-2.0, 0.0, 0.0}};
    Geom_objects::triangle_t<double> triangle_2{{3.0, 0.0, 0.0},  {-4.0, -5.0, 0.0}, {0.0, 2.0, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_2d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangles_intersectection_in_2d_4) {
    Geom_objects::triangle_t<double> triangle_1{{0.0, 0.0, 0.0}, {0.0, 0.0, 4.0}, {0.0, 5.0, 0.0}};
    Geom_objects::triangle_t<double> triangle_2{{0.0, 1.0, 0.0}, {0.0, 1.0, 1.0}, {0.0, 3.0, 1.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_2d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, segments_of_triangles_dont_intersect) {
    Geom_objects::triangle_t<double> triangle_1{{-4.0, 0.0, 0.0}, {0.0, -4.0, 0.0}, {-2.0, 0.0, 0.0}};
    Geom_objects::triangle_t<double> triangle_2{{2.0, 0.0, 0.0},  {3.0, 0.0, 0.0},  {0.0, 2.0, 0.0}};
    ASSERT_EQ(triangle_1.segments_of_triangles_intersects(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, point_outside_triangle_1) {
    Geom_objects::point_t<double> p{0.0, 0.0, 0.0};
    Geom_objects::triangle_t<double> triangle{{0.0, 0.0, 2.0}, {1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}};
    ASSERT_EQ(triangle.point_lies_inside_triangle(p), false);
}

TEST(TRIANGLE_FUNCTIONS, point_outside_triangle_2) {
    Geom_objects::point_t<double> p{1.2, 0.0, 0.0};
    Geom_objects::triangle_t<double> triangle{{0.0, 0.0, 0.2}, {1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}};
    ASSERT_EQ(triangle.point_lies_inside_triangle(p), false);
}

TEST(TRIANGLE_FUNCTIONS, point_inside_triangle) {
    Geom_objects::triangle_t<double> triangle{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    Geom_objects::point_t<double> point{0.5, 0.0, 0.0};
    ASSERT_EQ(triangle.point_lies_inside_triangle(point), true);
}

TEST(TRIANGLE_FUNCTIONS, figures_intersection_1) {
    Geom_objects::point_t point_1_of_polygon_1 = {0.0, 1.0, 0.0};
    Geom_objects::point_t point_2_of_polygon_1 = {0.0, 1.0, 0.0};
    Geom_objects::point_t point_3_of_polygon_1 = {0.0, 1.0, 0.0};

    Geom_objects::point_t point_1_of_polygon_2 = {0.0, 1.0, 0.0};
    Geom_objects::point_t point_2_of_polygon_2 = {0.0, 1.0, 0.0};
    Geom_objects::point_t point_3_of_polygon_2 = {0.0, 1.0, 0.0};
    
    Geom_objects::polygon_t<double> polygon_1 = Geom_objects::make_geometric_primitive(point_1_of_polygon_1, 
                                                                                       point_2_of_polygon_1, 
                                                                                       point_3_of_polygon_1);

    Geom_objects::polygon_t<double> polygon_2 = Geom_objects::make_geometric_primitive(point_1_of_polygon_2, 
                                                                                       point_2_of_polygon_2, 
                                                                                       point_3_of_polygon_2); 

    ASSERT_EQ(Geom_objects::check_figures_intersection(polygon_1, polygon_2), true);
}

TEST(TRIANGLE_FUNCTIONS, figures_intersection_2) {
    Geom_objects::point_t point_1_of_polygon_1 = {0.0, 0.0, 0.0};
    Geom_objects::point_t point_2_of_polygon_1 = {1.0, 0.0, 0.0};
    Geom_objects::point_t point_3_of_polygon_1 = {0.0, 1.0, 0.0};

    Geom_objects::point_t point_1_of_polygon_2 = {0.5, -0.5, 0.0};
    Geom_objects::point_t point_2_of_polygon_2 = {0.5, 0.5, 1.0};
    Geom_objects::point_t point_3_of_polygon_2 = {0.5, 0.5, -1.0};
    
    Geom_objects::polygon_t<double> polygon_1 = Geom_objects::make_geometric_primitive(point_1_of_polygon_1, 
                                                                                       point_2_of_polygon_1, 
                                                                                       point_3_of_polygon_1);

    Geom_objects::polygon_t<double> polygon_2 = Geom_objects::make_geometric_primitive(point_1_of_polygon_2, 
                                                                                       point_2_of_polygon_2, 
                                                                                       point_3_of_polygon_2); 

    ASSERT_EQ(Geom_objects::check_figures_intersection(polygon_1, polygon_2), true);
}

TEST(TRIANGLE_FUNCTIONS, figures_intersection_3) {
    Geom_objects::point_t point_1_of_polygon_1 = {1.0, 1.0, 1.0};
    Geom_objects::point_t point_2_of_polygon_1 = {2.0, 1.0, 1.0};
    Geom_objects::point_t point_3_of_polygon_1 = {1.0, 2.0, 1.0};

    Geom_objects::point_t point_1_of_polygon_2 = {1.5, 0.5, 0.0};
    Geom_objects::point_t point_2_of_polygon_2 = {1.5, 1.5, 2.0};
    Geom_objects::point_t point_3_of_polygon_2 = {1.5, 1.5, -2.0};
    
    Geom_objects::polygon_t<double> polygon_1 = Geom_objects::make_geometric_primitive(point_1_of_polygon_1, 
                                                                                       point_2_of_polygon_1, 
                                                                                       point_3_of_polygon_1);

    Geom_objects::polygon_t<double> polygon_2 = Geom_objects::make_geometric_primitive(point_1_of_polygon_2, 
                                                                                       point_2_of_polygon_2, 
                                                                                       point_3_of_polygon_2); 

    ASSERT_EQ(Geom_objects::check_figures_intersection(polygon_1, polygon_2), true);
}

TEST(TRIANGLE_FUNCTIONS, figures_intersection_4) {
    Geom_objects::point_t point_1_of_polygon_1 = {1.0, 1.0, 0.0};
    Geom_objects::point_t point_2_of_polygon_1 = {3.0, 1.0, 0.0};
    Geom_objects::point_t point_3_of_polygon_1 = {1.0, 3.0, 0.0};

    Geom_objects::point_t point_1_of_polygon_2 = {0.0, 0.0, 0.0};
    Geom_objects::point_t point_2_of_polygon_2 = {1.0, 0.0, 0.0};
    Geom_objects::point_t point_3_of_polygon_2 = {0.0, 1.0, 0.0};
    
    Geom_objects::polygon_t<double> polygon_1 = Geom_objects::make_geometric_primitive(point_1_of_polygon_1, 
                                                                                       point_2_of_polygon_1, 
                                                                                       point_3_of_polygon_1);

    Geom_objects::polygon_t<double> polygon_2 = Geom_objects::make_geometric_primitive(point_1_of_polygon_2, 
                                                                                       point_2_of_polygon_2, 
                                                                                       point_3_of_polygon_2); 

    ASSERT_EQ(Geom_objects::check_figures_intersection(polygon_1, polygon_2), false);
}

TEST(TRIANGLE_FUNCTIONS, figures_intersection_5) {
    Geom_objects::point_t point_1_of_polygon_1 = {1.0, 1.0, 2.0};
    Geom_objects::point_t point_2_of_polygon_1 = {1.0, 1.0, 2.0};
    Geom_objects::point_t point_3_of_polygon_1 = {2.0, 2.0, 2.0};

    Geom_objects::point_t point_1_of_polygon_2 = {1.0, 1.0, 1.0};
    Geom_objects::point_t point_2_of_polygon_2 = {1.0, 5.0, 1.0};
    Geom_objects::point_t point_3_of_polygon_2 = {5.0, 1.0, 1.0};
    
    Geom_objects::polygon_t<double> polygon_1 = Geom_objects::make_geometric_primitive(point_1_of_polygon_1, 
                                                                                       point_2_of_polygon_1, 
                                                                                       point_3_of_polygon_1);

    Geom_objects::polygon_t<double> polygon_2 = Geom_objects::make_geometric_primitive(point_1_of_polygon_2, 
                                                                                       point_2_of_polygon_2, 
                                                                                       point_3_of_polygon_2); 

    ASSERT_EQ(Geom_objects::check_figures_intersection(polygon_1, polygon_2), false);
}

TEST(TRIANGLE_FUNCTIONS, figures_intersection_6) {
    Geom_objects::point_t point_1_of_polygon_1 = {1.0, 1.0, 1.0};
    Geom_objects::point_t point_2_of_polygon_1 = {2.0, 2.0, 2.0};
    Geom_objects::point_t point_3_of_polygon_1 = {3.0, 3.0, 3.0};

    Geom_objects::point_t point_1_of_polygon_2 = {1.0, 1.0, 1.0};
    Geom_objects::point_t point_2_of_polygon_2 = {1.0, 5.0, 1.0};
    Geom_objects::point_t point_3_of_polygon_2 = {5.0, 1.0, 1.0};
    
    Geom_objects::polygon_t<double> polygon_1 = Geom_objects::make_geometric_primitive(point_1_of_polygon_1, 
                                                                                       point_2_of_polygon_1, 
                                                                                       point_3_of_polygon_1);

    Geom_objects::polygon_t<double> polygon_2 = Geom_objects::make_geometric_primitive(point_1_of_polygon_2, 
                                                                                       point_2_of_polygon_2, 
                                                                                       point_3_of_polygon_2); 

    ASSERT_EQ(Geom_objects::check_figures_intersection(polygon_1, polygon_2), true);
}

TEST(TRIANGLE_FUNCTIONS, figures_intersection_7) {
    Geom_objects::point_t point_1_of_polygon_1 = {1.0, 1.0, 1.0};
    Geom_objects::point_t point_2_of_polygon_1 = {2.0, 2.0, 2.0};
    Geom_objects::point_t point_3_of_polygon_1 = {3.0, 3.0, 3.0};

    Geom_objects::point_t point_1_of_polygon_2 = {1.0, 1.0, 2.0};
    Geom_objects::point_t point_2_of_polygon_2 = {1.0, 1.0, 2.0};
    Geom_objects::point_t point_3_of_polygon_2 = {2.0, 2.0, 2.0};
    
    Geom_objects::polygon_t<double> polygon_1 = Geom_objects::make_geometric_primitive(point_1_of_polygon_1, 
                                                                                       point_2_of_polygon_1, 
                                                                                       point_3_of_polygon_1);

    Geom_objects::polygon_t<double> polygon_2 = Geom_objects::make_geometric_primitive(point_1_of_polygon_2, 
                                                                                       point_2_of_polygon_2, 
                                                                                       point_3_of_polygon_2); 

    ASSERT_EQ(Geom_objects::check_figures_intersection(polygon_1, polygon_2), true);
}

TEST(TRIANGLE_FUNCTIONS, figures_intersection_8) {
    Geom_objects::point_t point_1_of_polygon_1 = {1.0, 0.0, 0.0};
    Geom_objects::point_t point_2_of_polygon_1 = {0.0, 1.0, 0.0};
    Geom_objects::point_t point_3_of_polygon_1 = {0.0, 0.0, 1.0};

    Geom_objects::point_t point_1_of_polygon_2 = {8.0, 8.0, 8.0};
    Geom_objects::point_t point_2_of_polygon_2 = {8.0, 8.0, 8.0};
    Geom_objects::point_t point_3_of_polygon_2 = {-10.0, 8.0, 8.0};
    
    Geom_objects::polygon_t<double> polygon_1 = Geom_objects::make_geometric_primitive(point_1_of_polygon_1, 
                                                                                       point_2_of_polygon_1, 
                                                                                       point_3_of_polygon_1);

    Geom_objects::polygon_t<double> polygon_2 = Geom_objects::make_geometric_primitive(point_1_of_polygon_2, 
                                                                                       point_2_of_polygon_2, 
                                                                                       point_3_of_polygon_2); 

    ASSERT_EQ(Geom_objects::check_figures_intersection(polygon_1, polygon_2), false);
}

TEST(TRIANGLE_FUNCTIONS, figures_intersection_9) {
    Geom_objects::point_t point_1_of_polygon_1 = {0.25, 0.25, 2.0};
    Geom_objects::point_t point_2_of_polygon_1 = {0.25, 0.25, -2.0};
    Geom_objects::point_t point_3_of_polygon_1 = {0.25, 0.25, 1.0};

    Geom_objects::point_t point_1_of_polygon_2 = {0.25, 0.25, 1.0};
    Geom_objects::point_t point_2_of_polygon_2 = {0.25, 0.25, -1.0};
    Geom_objects::point_t point_3_of_polygon_2 = {0.25, 0.25, 0.0};
    
    Geom_objects::polygon_t<double> polygon_1 = Geom_objects::make_geometric_primitive(point_1_of_polygon_1, 
                                                                                       point_2_of_polygon_1, 
                                                                                       point_3_of_polygon_1);

    Geom_objects::polygon_t<double> polygon_2 = Geom_objects::make_geometric_primitive(point_1_of_polygon_2, 
                                                                                       point_2_of_polygon_2, 
                                                                                       point_3_of_polygon_2); 

    ASSERT_EQ(Geom_objects::check_figures_intersection(polygon_1, polygon_2), true);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
