#ifndef SEGMENT_HPP
#define SEGMENT_HPP

#include "point.hpp"
#include "vector.hpp"
namespace Geom_objects {

template <typename T>
class segment_t {
    point_t<T>   beg_point_;
    point_t<T>   end_point_;
    vector_t<T>  directing_vector_;
    size_t       number_;

    public:
    segment_t(const point_t<T>& beg_point, const point_t<T>& end_point, size_t number = 0)
        : beg_point_{beg_point}, end_point_{end_point}, 
          directing_vector_{end_point - beg_point}, number_{number} {}

    segment_t(const point_t<T>& beg_point, const vector_t<T>& directing_vector, size_t number = 0)
        : beg_point_{beg_point}, 
          end_point_{beg_point_ + directing_vector}, 
          directing_vector_{directing_vector}, number_{number} {}

    public:
    point_t<T>  get_beg_point()  const { return beg_point_; }
    point_t<T>  get_end_point()  const { return end_point_; }
    vector_t<T> get_dir_vector() const { return directing_vector_; }
    size_t      get_number()     const { return number_; }

    T get_length() const {
        return beg_point_.distance_between_points(end_point_);
    }

    bool point_lies_on_segment(const point_t<T>& point) const {
        T distance_1     = point.distance_between_points(beg_point_);
        T distance_2     = point.distance_between_points(end_point_);
        T segment_length = get_length();

        return Compare::is_equal(distance_1 + distance_2, segment_length);
    }

    bool point_lies_on_line(const point_t<T>& point) const {
        vector_t<T> vector_between_points{point - beg_point_};

        return directing_vector_.vectors_are_collinear(vector_between_points);
    }

    bool segments_intersects(const segment_t<T>& other_segment) const {
        if (lines_are_coincident(other_segment) && segments_overloap(other_segment))
            return true;

        point_t<T> point_of_intersection = line_and_line_intersects(other_segment);

        if (!point_of_intersection.valid())
            return false;

        if (point_lies_on_segment(point_of_intersection) &&
            other_segment.point_lies_on_segment(point_of_intersection))
            return true;

        return false;
    }

    bool lines_are_coincident(const segment_t<T>& other_line) const {
        return point_lies_on_line(other_line.beg_point_) && point_lies_on_line(other_line.end_point_);
    }

    point_t<T> line_and_line_intersects(const segment_t<T>& other_line) const {
        T dx1 = directing_vector_.get_x();
        T dy1 = directing_vector_.get_y();
        T dz1 = directing_vector_.get_z();

        T dx2 = other_line.directing_vector_.get_x();
        T dy2 = other_line.directing_vector_.get_y();
        T dz2 = other_line.directing_vector_.get_z();

        T a = dx1 * dx1 + dy1 * dy1 + dz1 * dz1;
        T b = dx1 * dx2 + dy1 * dy2 + dz1 * dz2;
        T c = dx2 * dx2 + dy2 * dy2 + dz2 * dz2;

        T d = dx1 * (beg_point_.get_x() - other_line.beg_point_.get_x()) 
            + dy1 * (beg_point_.get_y() - other_line.beg_point_.get_y()) 
            + dz1 * (beg_point_.get_z() - other_line.beg_point_.get_z());

        T e = dx2 * (beg_point_.get_x() - other_line.beg_point_.get_x()) 
            + dy2 * (beg_point_.get_y() - other_line.beg_point_.get_y()) 
            + dz2 * (beg_point_.get_z() - other_line.beg_point_.get_z());

        T denominator = a * c - b * b;

        if (Compare::is_equal(denominator, 0.0)) {
            point_t<T> invalid_point{NAN, NAN, NAN};
            return invalid_point;
        }

        T coeff = (b * e - c * d) / denominator;

        point_t<T> intersection_point{beg_point_.get_x() + coeff * dx1, 
                                      beg_point_.get_y() + coeff * dy1,
                                      beg_point_.get_z() + coeff * dz1};
        return intersection_point;
    }

    bool segments_overloap(const segment_t<T>& other_segment) const {
        if (!directing_vector_.vectors_are_collinear(other_segment.directing_vector_))
            return false;

        if (other_segment.point_lies_on_segment(beg_point_) ||
            other_segment.point_lies_on_segment(end_point_) ||
            point_lies_on_segment(other_segment.beg_point_) ||
            point_lies_on_segment(other_segment.end_point_))
            return true;

        return false;
    }

    void print() const {
        std::cout << "Begin point: ";
        beg_point_.print();
        std::cout << "End point: ";
        end_point_.print();
        std::cout << "Directing vector: ";
        directing_vector_.print();
    }
};

} // namespace Geom_objects

#endif // SEGMENT_HPP