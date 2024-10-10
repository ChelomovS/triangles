#ifndef SEGMENT_HPP
#define SEGMENT_HPP

#include "point.hpp"
#include "vector.hpp"

namespace Segment {
    
class segment_t {
    Point::point_t   beg_point_;
    Point::point_t   end_point_;
    Vector::vector_t directing_vector_;

    public:
    segment_t(const Point::point_t& beg_point, const Point::point_t& end_point,
                                        const Vector::vector_t& directing_vector):
    beg_point_{beg_point}, end_point_{end_point}, directing_vector_{directing_vector} {}


    segment_t(const Point::point_t& beg_point, const Point::point_t& end_point):
    beg_point_{beg_point}, end_point_{end_point}, 
    directing_vector_{end_point_.get_x() - beg_point_.get_x(),
                      end_point_.get_y() - beg_point_.get_y(), 
                      end_point_.get_z() - beg_point_.get_z()} {};

    segment_t(const Point::point_t& point, const Vector::vector_t& directing_vector):
    beg_point_{point}, end_point_{beg_point_.get_x() + directing_vector.get_x(), 
                                  beg_point_.get_y() + directing_vector.get_y(),
                                  beg_point_.get_z() + directing_vector.get_z()},
    directing_vector_{directing_vector} {};

    public:    
    Point::point_t   get_beg_point()  const { return beg_point_; }
    Point::point_t   get_end_point()  const { return end_point_; }
    Vector::vector_t get_dir_vector() const { return directing_vector_; }

    double get_length() const;
    bool point_lies_on_segment(const Point::point_t& point) const;
    bool point_lies_on_line(const Point::point_t& point) const;
    bool segments_intersects(const segment_t& other_segment) const;
    bool lines_are_coincident(const Segment::segment_t& other_line) const;
    Point::point_t line_and_line_intersects(const segment_t& other_line) const;
    bool segments_overloap(const segment_t& other_segment) const;

    void print() const;
};

} // namespace Segment

#endif // SEGMENT_HPP