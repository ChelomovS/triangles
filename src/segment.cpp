#include <iostream>

#include "segment.hpp"
#include "point.hpp"

double Segment::segment_t::get_length() const {
    return beg_point_.distance_between_points(end_point_);
}

bool Segment::segment_t::point_lies_on_segment(const Point::point_t& point) const {
    double distance_1     = point.distance_between_points(beg_point_);
    double distance_2     = point.distance_between_points(end_point_);
    double segment_length = get_length();

    return Compare::is_equal(distance_1 + distance_2, segment_length);
}

bool Segment::segment_t::point_lies_on_line(const Point::point_t& point) const {
    Vector::vector_t vector_between_points = {point.get_x() - beg_point_.get_x(), 
                                              point.get_y() - beg_point_.get_y(), 
                                              point.get_z() - beg_point_.get_z()};

    return directing_vector_.vectors_are_collinear(vector_between_points);
}

bool Segment::segment_t::segments_intersects(const Segment::segment_t& other_segment) const {
    if (lines_are_coincident(other_segment) && segments_overloap(other_segment))
        return true;

    Point::point_t point_of_intersection = line_and_line_intersects(other_segment);

    if (!point_of_intersection.valid()) 
        return false;

    if (point_lies_on_segment(point_of_intersection) &&
        other_segment.point_lies_on_segment(point_of_intersection))
        return true;

    return false;
}

bool Segment::segment_t::lines_are_coincident(const Segment::segment_t& other_line) const {
    return point_lies_on_line(other_line.beg_point_) && point_lies_on_line(other_line.end_point_);
}

bool Segment::segment_t::segments_overloap(const Segment::segment_t& other_segment) const {
    if (!directing_vector_.vectors_are_collinear(other_segment.directing_vector_))
        return false;

    if (other_segment.point_lies_on_segment(beg_point_) ||
        other_segment.point_lies_on_segment(end_point_) ||
        point_lies_on_segment(other_segment.beg_point_) ||
        point_lies_on_segment(other_segment.end_point_))
        return true;

    return false;
}

Point::point_t Segment::segment_t::line_and_line_intersects(const Segment::segment_t& other_line) const {
    // Directing vectors coordinates
    double dx1 = directing_vector_.get_x();
    double dy1 = directing_vector_.get_y();
    double dz1 = directing_vector_.get_z();

    double dx2 = other_line.directing_vector_.get_x();
    double dy2 = other_line.directing_vector_.get_y();
    double dz2 = other_line.directing_vector_.get_z();

    double a = dx1 * dx1 + dy1 * dy1 + dz1 * dz1;
    double b = dx1 * dx2 + dy1 * dy2 + dz1 * dz2;
    double c = dx2 * dx2 + dy2 * dy2 + dz2 * dz2;

    double d = dx1 * (beg_point_.get_x() - other_line.beg_point_.get_x()) 
             + dy1 * (beg_point_.get_y() - other_line.beg_point_.get_y()) 
             + dz1 * (beg_point_.get_z() - other_line.beg_point_.get_z());

    double e = dx2 * (beg_point_.get_x() - other_line.beg_point_.get_x()) 
             + dy2 * (beg_point_.get_y() - other_line.beg_point_.get_y()) 
             + dz2 * (beg_point_.get_z() - other_line.beg_point_.get_z());

    double denominator = a * c - b * b;

    if (Compare::is_equal(denominator, 0)) {
        Point::point_t invalid_point{NAN, NAN, NAN};
        return invalid_point;
    }

    double coeff = (b * e - c * d) / denominator;
    Point::point_t intersection_point{beg_point_.get_x() + coeff * dx1, 
                                      beg_point_.get_y() + coeff * dy1,
                                      beg_point_.get_z() + coeff * dz1};
    return intersection_point;
}
 
void Segment::segment_t::print() const {
    std::cout << "Begin point: ";
    beg_point_.print();
    std::cout << "End point: ";
    end_point_.print();
    std::cout << "Directing vector: ";
    directing_vector_.print();
}
