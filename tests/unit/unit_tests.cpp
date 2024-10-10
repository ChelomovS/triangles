#include <gtest/gtest.h>

#include "point.hpp"
#include "vector.hpp"
#include "segment.hpp"
#include "plane.hpp"
#include "triangle.hpp"

TEST(POINT_FUNCTIONS, point_is_not_valid) {
    Point::point_t p{NAN, NAN, NAN};
    ASSERT_EQ(p.valid(), false);
}

TEST(POINT_FUNCTIONS, point_is_valid) {
    Point::point_t p{1.0, 2.1, 3.8};
    ASSERT_EQ(p.valid(), true);
}

TEST(POINT_FUNCTIONS, distance_between_points_1) {
    Point::point_t p1{0.0, 0.0, 3.0};
    Point::point_t p2{0.0, 0.0, 4.0};
    double distance_between_points = p1.distance_between_points(p2);
    ASSERT_DOUBLE_EQ(distance_between_points, 1.0);
}

TEST(POINT_FUNCTIONS, distance_between_points_2) {
    Point::point_t p1{0.0, 3.0, 0.0};
    Point::point_t p2{0.0, 0.0, 4.0};
    double distance_between_points = p1.distance_between_points(p2);
    ASSERT_DOUBLE_EQ(distance_between_points, 5.0);
}

TEST(VECTOR_FUNCTIONS, dot_product_1) {
    Vector::vector_t v1{-5.0, 70.0, 80.0};
    Vector::vector_t v2{-90.0, 100.0, 100.0};
    double dot_product = v1.dot_product(v2);
    ASSERT_DOUBLE_EQ(dot_product, 15450.0);
}

TEST(VECTOR_FUNCTIONS, dot_product_2) {
    Vector::vector_t v1{100.0, 100.0, 100.0};
    Vector::vector_t v2{100.0, -100.0, 0.0};
    double dot_product = v1.dot_product(v2);
    ASSERT_DOUBLE_EQ(dot_product, 0.0);
}

TEST(VECTOR_FUNCTIONS, cross_product_vector_1) {
    Vector::vector_t v1{-90.0, 100.0, 105.0};
    Vector::vector_t v2{90.0, 1.0, 4.0};
    Vector::vector_t cross_product = v1.cross_product_vector(v2);
    ASSERT_EQ(cross_product.is_equal({295.0, 9810.0, -9090.0}), true);
}

TEST(VECTOR_FUNCTIONS, vectors_are_collinear_1) {
    Vector::vector_t v1{-4.0, 5.0, 0.0};
    Vector::vector_t v2{9.0, 9.0, 9.0};
    ASSERT_EQ(v1.vectors_are_collinear(v2), false);
}

TEST(VECTOR_FUNCTIONS, vectors_are_collinear_2) {
    Vector::vector_t v1{15.0, 15.0, 15.0};
    Vector::vector_t v2{-1.0, -1.0, -1.0};
    ASSERT_EQ(v1.vectors_are_collinear(v2), true);
}

TEST(SEGMENT_FUNCTIONS, point_lies_on_segment_1) {
    Point::point_t point{0.0, 0.0, 0.5};
    Point::point_t beg_segment{0.0, 0.0, 1.0};
    Point::point_t end_segment{0.0, 0.0, 0.0};
    Segment::segment_t segment{beg_segment, end_segment};
    ASSERT_EQ(segment.point_lies_on_segment(point), true);
}

TEST(SEGMENT_FUNCTIONS, point_lies_on_segment_2) {
    Point::point_t point{0.5, -0.5, 1.0};
    Point::point_t beg_segment{0.0, 0.0, 2.0};
    Point::point_t end_segment{1.0, -1.0, 0.0};
    Segment::segment_t segment{beg_segment, end_segment};
    ASSERT_EQ(segment.point_lies_on_segment(point), true);
}

TEST(SEGMENT_FUNCTIONS, segment_length) {
    Point::point_t beg_segment{0.0, 0.0, 0.0};
    Point::point_t end_segment{13.0, 18.0, 6.0};
    Segment::segment_t segment{beg_segment, end_segment};
    ASSERT_DOUBLE_EQ(segment.get_length(), 23.0);
}

TEST(SEGMENT_FUNCTIONS, segments_intersection_1) {
    Point::point_t beg_segment_1{3.0, 0.0, 0.0};
    Point::point_t end_segment_1{0.0, 0.0, 2.0};   
    Point::point_t beg_segment_2{2.0, 0.0, 0.0};
    Point::point_t end_segment_2{0.0, 0.0, 3.0};
    Segment::segment_t segment_1{beg_segment_1, end_segment_1};
    Segment::segment_t segment_2{beg_segment_2, end_segment_2};
    ASSERT_EQ(segment_1.segments_intersects(segment_2), true);
}
 
TEST(SEGMENT_FUNCTIONS, segments_intersection_2) {
    Point::point_t beg_segment_1{2.0, 0.0, 0.0};
    Point::point_t end_segment_1{0.0, 0.0, 3.0};   
    Point::point_t beg_segment_2{2.0, 0.0, 0.0};
    Point::point_t end_segment_2{0.0, 0.0, 3.0};
    Segment::segment_t segment_1{beg_segment_1, end_segment_1};
    Segment::segment_t segment_2{beg_segment_2, end_segment_2};
    ASSERT_EQ(segment_1.segments_intersects(segment_2), true);
}

TEST(SEGMENT_FUNCTIONS, segments_intersection_3) {
    Point::point_t beg_segment_1{-1.0, -1.0, 0.0};
    Point::point_t end_segment_1{0.0, 0.0, 3.0};   
    Point::point_t beg_segment_2{-3.0, 0.0, 0.0};
    Point::point_t end_segment_2{-1.0, -1.0, 0.0};
    Segment::segment_t segment_1{beg_segment_1, end_segment_1};
    Segment::segment_t segment_2{beg_segment_2, end_segment_2};
    ASSERT_EQ(segment_1.segments_intersects(segment_2), true);
}

TEST(SEGMENT_FUNCTIONS, segment_and_line_intersection_1) {
    Point::point_t beg_line_1{3.0, 0.0, 0.0};
    Point::point_t end_line_1{0.0, 0.0, 2.0};   
    Point::point_t beg_line_2{2.0, 0.0, 0.0};
    Point::point_t end_line_2{0.0, 0.0, 3.0};
    Segment::segment_t line_1{beg_line_1, end_line_1};
    Segment::segment_t line_2{beg_line_2, end_line_2};
    ASSERT_EQ(line_1.line_and_line_intersects(line_2).equal({1.2, 0.0, 1.2}), true);
}

TEST(SEGMENT_FUNCTIONS, line_and_line_intersection_2) {
    Point::point_t beg_line_1{-6.0, 0.0, 0.0}; 
    Point::point_t end_line_1{0.0, -6.0, 0.0};
    Point::point_t beg_line_2{0.0, 0.0, 0.0};
    Point::point_t end_line_2{-5.0, -5.0, 0.0};
    Segment::segment_t line_1{beg_line_1, end_line_1};
    Segment::segment_t line_2{beg_line_2, end_line_2};
    ASSERT_EQ(line_1.line_and_line_intersects(line_2).equal({-3.0, -3.0, 0.0}), true);
}

TEST(SEGMENT_FUNCTIONS, point_lies_on_line_1) {
    Point::point_t beg_point{0.0, 0.0, 0.0};
    Point::point_t end_point{1.0, 1.0, 1.0};
    Point::point_t point{-100.0, -100.0, -100.0};
    Segment::segment_t line{beg_point, end_point};
    ASSERT_EQ(line.point_lies_on_line(point), true);
}

TEST(SEGMENT_FUNCTIONS, point_lies_on_line_2) {
    Point::point_t beg_point{0.0, 0.0, 0.0};
    Point::point_t end_point{1.0, 1.0, 1.0};
    Point::point_t point{0.5, 0.5, 0.5};
    Segment::segment_t line{beg_point, end_point};
    ASSERT_EQ(line.point_lies_on_line(point), true);
}

TEST(SEGMENT_FUNCTIONS, lines_are_concident_1) {
    Point::point_t beg_point_1{1.0, 1.0, 1.0};
    Point::point_t end_point_1{2.0, 2.0, 2.0};
    Point::point_t beg_point_2{50.0, 50.0, 50.0};
    Point::point_t end_point_2{100.0, 100.0, 100.0};
    Segment::segment_t line_1{beg_point_1, end_point_1};
    Segment::segment_t line_2{beg_point_2, end_point_2};
    ASSERT_EQ(line_1.lines_are_coincident(line_2), true);
}

TEST(SEGMENT_FUNCTIONS, lines_are_concident_2) {
    Point::point_t beg_point_1{1.0, 1.5, 1.0};
    Point::point_t end_point_1{2.0, 2.0, 2.0};
    Point::point_t beg_point_2{50.0, 50.0, 50.0};
    Point::point_t end_point_2{100.0, 100.0, 100.0};
    Segment::segment_t line_1{beg_point_1, end_point_1};
    Segment::segment_t line_2{beg_point_2, end_point_2};
    ASSERT_EQ(line_1.lines_are_coincident(line_2), false);
}

TEST(SEGMENT_FUNCTIONS, segments_overlope_1) {
    Point::point_t beg_point_1{20.0, 0.0, 0.0};
    Point::point_t end_point_1{100.0, 0.0, 0.0};
    Point::point_t beg_point_2{50.0, 0.0, 0.0};
    Point::point_t end_point_2{-50.0, 0.0, 0.0};
    Segment::segment_t segment_1{beg_point_1, end_point_1};
    Segment::segment_t segment_2{beg_point_2, end_point_2};
    ASSERT_EQ(segment_1.segments_overloap(segment_2), true);
}

TEST(SEGMENT_FUNCTIONS, segments_overlope_2) {
    Point::point_t beg_point_1{0.0, 0.0, 0.0};
    Point::point_t end_point_1{50.0, 50.0, 50.0};
    Point::point_t beg_point_2{-100.0, -100.0, -100.0};
    Point::point_t end_point_2{100.0, 100.0, 100.0};
    Segment::segment_t segment_1{beg_point_1, end_point_1};
    Segment::segment_t segment_2{beg_point_2, end_point_2};
    ASSERT_EQ(segment_1.segments_overloap(segment_2), true);
}

TEST(SEGMENT_FUNCTIONS, segments_dont_overlope_1) {
    Point::point_t beg_point_1{0.0, 0.0, -50.0};
    Point::point_t end_point_1{0.0, 0.0, 50.0};
    Point::point_t beg_point_2{50.0, 100.0, 0.0};
    Point::point_t end_point_2{-50.0, 200.0, 0.0};
    Segment::segment_t segment_1{beg_point_1, end_point_1};
    Segment::segment_t segment_2{beg_point_2, end_point_2};
    ASSERT_EQ(segment_1.segments_overloap(segment_2), false);
}

TEST(SEGMENT_FUNCTIONS, segments_dont_overlope_2) {
    Point::point_t beg_point_1{0.0, 0.0, 1.0};
    Point::point_t end_point_1{0.0, 0.0, 2.0};
    Point::point_t beg_point_2{0.0, 0.0, 0.0};
    Point::point_t end_point_2{0.0, 0.0, 0.9};
    Segment::segment_t segment_1{beg_point_1, end_point_1};
    Segment::segment_t segment_2{beg_point_2, end_point_2};
    ASSERT_EQ(segment_1.segments_overloap(segment_2), false);
}

TEST(PLANE_FUNCTIONS, point_lies_on_plane_1) {
    Point::point_t point{0.0, 0.0, 0.0};
    Plane::plane_t plane{{0.0, 1.0, 1.0}, {-2.5, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    ASSERT_EQ(plane.point_lies_on_plane(point), true);
}

TEST(PLANE_FUNCTIONS, point_lies_on_plane_2) {
    Point::point_t point{0.1, 0.1, 0.1};
    Plane::plane_t plane{{0.0, 1.0, 1.0}, {-2.5, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    ASSERT_EQ(plane.point_lies_on_plane(point), true);
}

TEST(TRIANGLE_FUNCTIONS, get_type_of_triangle_1) {
    Point::point_t point_1{0.0, 0.0, 0.0};
    Point::point_t point_2{0.0, 0.0, 0.0};
    Point::point_t point_3{0.0, 0.0, 0.0};
    Triangle::triangle_t triangle{point_1, point_2, point_3};
    Triangle::Triangle_type type = triangle.get_type_of_triangle();
    ASSERT_EQ(type == Triangle::Triangle_type::point, true);
}

TEST(TRIANGLE_FUNCTIONS, get_type_of_triangle_2) {
    Point::point_t point_1{1.0, 0.0, 0.0};
    Point::point_t point_2{0.0, 0.0, 0.0};
    Point::point_t point_3{0.0, 0.0, 0.0};
    Triangle::triangle_t triangle{point_1, point_2, point_3};
    Triangle::Triangle_type type = triangle.get_type_of_triangle();
    ASSERT_EQ(type == Triangle::Triangle_type::segment, true);
}

TEST(TRIANGLE_FUNCTIONS, get_type_of_triangle_3) {
    Point::point_t point_1{1.0, 0.0, 0.0};
    Point::point_t point_2{0.0, 1.0, 0.0};
    Point::point_t point_3{0.0, 0.0, 1.0};
    Triangle::triangle_t triangle{point_1, point_2, point_3};
    Triangle::Triangle_type type = triangle.get_type_of_triangle();
    ASSERT_EQ(type == Triangle::Triangle_type::triangle, true);
}

TEST(TRIANGLE_FUNCTIONS, triangles_intersectection_in_2d_1) {
    Triangle::triangle_t triangle_1{{-4.0, 0.0, 0.0}, {0.0, -4.0, 0.0}, {-2.0, 0.0, 0.0}};
    Triangle::triangle_t triangle_2{{2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, {0.0, 2.0, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_2d(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, triangles_intersectection_in_2d_2) {
    Triangle::triangle_t triangle_1{{-4.0, 0.0, 0.0}, {0.0, -4.0, 0.0}, {-2.0, 0.0, 0.0}};
    Triangle::triangle_t triangle_2{{3.0, 0.0, 0.0}, {-5.0, 0.0, 0.0}, {0.0, 2.0, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_2d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangles_intersectection_in_2d_3) {
    Triangle::triangle_t triangle_1{{-4.0, 0.0, 0.0}, {0.0, -4.0, 0.0}, {-2.0, 0.0, 0.0}};
    Triangle::triangle_t triangle_2{{3.0, 0.0, 0.0}, {-4.0, -5.0, 0.0}, {0.0, 2.0, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_2d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangles_intersectection_in_2d_4) {
    Triangle::triangle_t triangle_1{{0.0, 0.0, 0.0}, {0.0, 0.0, 4.0}, {0.0, 5.0, 0.0}};
    Triangle::triangle_t triangle_2{{0.0, 1.0, 0.0}, {0.0, 1.0, 1.0}, {0.0, 3.0, 1.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_2d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, segments_of_triangles_intersects_1) {
    Triangle::triangle_t triangle_1{{0.0, 0.0, 0.0}, {5.0, 5.0, 5.0}, {5.0, 5.0, 0.0}};
    Triangle::triangle_t triangle_2{{0.0, 0.0, 6.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 6.0}};
    ASSERT_EQ(triangle_1.segments_of_triangles_intersects(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, segments_of_triangles_intersects_2) {
    Triangle::triangle_t triangle_1{{2.0, 0.0, 0.0}, {0.0, -3.0, 0.0}, {-2.0, 0.0, 0.0}};
    Triangle::triangle_t triangle_2{{0.0, 0.0, 4.0}, {0.0, 0.0, -3.0}, {0.0, -3.0, 0.0}};
    ASSERT_EQ(triangle_1.segments_of_triangles_intersects(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, segments_of_triangles_dont_intersect) {
    Triangle::triangle_t triangle_1{{-4.0, 0.0, 0.0}, {0.0, -4.0, 0.0}, {-2.0, 0.0, 0.0}};
    Triangle::triangle_t triangle_2{{2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, {0.0, 2.0, 0.0}};
    ASSERT_EQ(triangle_1.segments_of_triangles_intersects(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, point_outside_triangle_2) {
    Point::point_t p{0.0, 0.0, 0.0};
    Triangle::triangle_t triangle{{0.0, 0.0, 2.0}, {1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}};
    ASSERT_EQ(triangle.point_lies_inside_triangle(p), false);
}

TEST(TRIANGLE_FUNCTIONS, point_outside_triangle_3) {
    Point::point_t p{1.2, 0.0, 0.0};
    Triangle::triangle_t triangle{{0.0, 0.0, 0.2}, {1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}};
    ASSERT_EQ(triangle.point_lies_inside_triangle(p), false);
}

TEST(TRIANGLE_FUNCTIONS, point_inside_triangle) {
    Triangle::triangle_t triangle{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    Point::point_t point{0.5, 0.0, 0.0};
    ASSERT_EQ(triangle.point_lies_inside_triangle(point), true);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_1) {
    Triangle::triangle_t triangle_1{{0.0, 1.0, 0.0}, {0.0, 0.0, 2.0}, {0.0, -1.0, 0.0}};
    Triangle::triangle_t triangle_2{{0.0, 2.0, 0.0}, {-3.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_2) {
    Triangle::triangle_t triangle_1{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    Triangle::triangle_t triangle_2{{0.5, -0.5, 0.0}, {0.5, 0.5, 1.0}, {0.5, 0.5, -1.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_3) {
    Triangle::triangle_t triangle_1{{1.0, 1.0, 1.0}, {2.0, 1.0, 1.0}, {1.0, 2.0, 1.0}};
    Triangle::triangle_t triangle_2{{1.5, 0.5, 0.0}, {1.5, 1.5, 2.0}, {1.5, 1.5, -2.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_4) {
    Triangle::triangle_t triangle_1{{0.0, -1.0, -1.0}, {2.0, -1.0, -1.0}, {0.0, -1.0, 2.0}};
    Triangle::triangle_t triangle_2{{1.0, 1.0, 0.0}, {3.0, 1.0, 0.0}, {1.0, 1.0, 2.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_5) {
    Triangle::triangle_t triangle_1{{0.0, -1.0, -1.0}, {2.0, -1.0, -1.0}, {0.0, -1.0, 2.0}};
    Triangle::triangle_t triangle_2{{2.0, 2.0, 2.0}, {1.0, 1.0, 2.0}, {2.0, 1.0, 2.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_6) {
    Triangle::triangle_t triangle_1{{1.0, 1.0, 0.0}, {3.0, 1.0, 0.0}, {1.0, 1.0, 2.0}};
    Triangle::triangle_t triangle_2{{2.0, 2.0, 2.0}, {1.0, 1.0, 2.0}, {2.0, 1.0, 2.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_7) {
    Triangle::triangle_t triangle_1{{1.0, 1.0, 0.0}, {3.0, 1.0, 0.0}, {1.0, 3.0, 0.0}};
    Triangle::triangle_t triangle_2{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_8) {
    Triangle::triangle_t triangle_1{{1.0, 0.5, 0.0}, {1.0, 0.5, 0.0}, {1.0, 0.5, 1.0}};
    Triangle::triangle_t triangle_2{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_9) {
    Triangle::triangle_t triangle_1{{1.0, 0.5, 0.0}, {1.0, 0.5, 0.0}, {1.0, 0.5, 1.0}};
    Triangle::triangle_t triangle_2{{1.0, 1.0, 0.0}, {3.0, 1.0, 0.0}, {1.0, 3.0, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_10) {
    Triangle::triangle_t triangle_1{{1.0, 0.5, 0.0}, {1.0, 0.5, 0.0}, {1.0, 0.5, 1.0}};
    Triangle::triangle_t triangle_2{{1.0, 1.0, 0.0}, {3.0, 1.0, 0.0}, {1.0, 3.0, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_11) {
    Triangle::triangle_t triangle_1{{1.0, 1.0, 2.0}, {1.0, 1.0, 2.0}, {2.0, 2.0, 2.0}};
    Triangle::triangle_t triangle_2{{1.0, 1.0, 1.0}, {1.0, 5.0, 1.0}, {5.0, 1.0, 1.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_12) {
    Triangle::triangle_t triangle_1{{1.0, 1.0, 1.0}, {2.0, 2.0, 2.0}, {3.0, 3.0, 3.0}};
    Triangle::triangle_t triangle_2{{1.0, 1.0, 1.0}, {1.0, 5.0, 1.0}, {5.0, 1.0, 1.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_13) {
    Triangle::triangle_t triangle_1{{1.0, 1.0, 1.0}, {2.0, 2.0, 2.0}, {3.0, 3.0, 3.0}};
    Triangle::triangle_t triangle_2{{1.0, 1.0, 2.0}, {1.0, 1.0, 2.0}, {2.0, 2.0, 2.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_14) {
    Triangle::triangle_t triangle_1{{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {0.0, 0.0, 0.0}};
    Triangle::triangle_t triangle_2{{1.0, 1.0, 2.0}, {1.0, 1.0, 2.0}, {2.0, 2.0, 2.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_15) {
    Triangle::triangle_t triangle_1{{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {0.0, 0.0, 0.0}};
    Triangle::triangle_t triangle_2{{1.0, 1.0, 1.0}, {5.0, 1.0, 1.0}, {3.0, 4.0, 1.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_16) {
    Triangle::triangle_t triangle_1{{1.0, 1.0, 1.0}, {1.0, 5.0, 1.0}, {5.0, 1.0, 1.0}};
    Triangle::triangle_t triangle_2{{1.0, 1.0, 1.0}, {5.0, 1.0, 1.0}, {3.0, 4.0, 1.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_17) {
    Triangle::triangle_t triangle_1{{1.0, 1.0, 1.0}, {1.0, 5.0, 1.0}, {5.0, 1.0, 1.0}};
    Triangle::triangle_t triangle_2{{1.0, 1.0, 1.0}, {5.0, 1.0, 1.0}, {3.0, 4.0, 1.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_18) {
    Triangle::triangle_t triangle_1{{81.8857, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 78.4631, 0.0}};
    Triangle::triangle_t triangle_2{{73.2058, 12.201, 0.0}, {155.092, 12.201, 0.0}, {73.2058, 90.6641, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), false);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_19) {
    Triangle::triangle_t triangle_1{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    Triangle::triangle_t triangle_2{{0.25, 0.25, 1.0}, {0.25, 0.25, -1.0}, {0.25, 0.25, 0.0}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), true);
}

TEST(TRIANGLE_FUNCTIONS, triangle_intersection_20) {
    Triangle::triangle_t triangle_1{{-4.4, 3.7, -2.1}, {-0.6, 2.8, -0.3}, {-1.6, -1.2, -1.0}};
    Triangle::triangle_t triangle_2{{0.2, -1.1, -1.7}, {2.6, -3.2, -3.9}, {-4.7, -1.7, 0.3}};
    ASSERT_EQ(triangle_1.triangles_intersects_in_3d(triangle_2), false);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}