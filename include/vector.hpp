#ifndef VECTOR_HPP
#define VECTOR_HPP

#include "point.hpp"

namespace Geom_objects {

template <typename T>
class vector_t {
    private:
    T x_, y_, z_;

    public:
    vector_t(const point_t<T>& beg_point, const point_t<T>& end_point): 
        x_{end_point.get_x() - beg_point.get_x()},
        y_{end_point.get_y() - beg_point.get_y()},
        z_{end_point.get_z() - beg_point.get_z()} {};

    vector_t(T x = NAN, T y = NAN, T z = NAN):
        x_{x}, y_{y}, z_{z} {};

    public:
    T get_x() const { return x_; };
    T get_y() const { return y_; };
    T get_z() const { return z_; };
    
    point_t<T> get_vector_coord() const {
        return point_t<T>{x_, y_, z_};
    }

    T dot_product(const vector_t<T>& other_vector) const {
        return x_ * other_vector.x_ + y_ * other_vector.y_ + z_ * other_vector.z_;
    }

    vector_t<T> cross_product(const vector_t<T>& other_vector) const {
        T x = y_ * other_vector.z_ - z_ * other_vector.y_;
        T y = z_ * other_vector.x_ - x_ * other_vector.z_;
        T z = x_ * other_vector.y_ - y_ * other_vector.x_;

        Geom_objects::vector_t<T> result_vector{x, y, z};

        return result_vector;
    }

    T get_length() const {
        return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
    }

    vector_t<T> get_normalized() const {
        T len = get_length();
        return vector_t<T>(x_ / len, y_ / len, z_ / len);
    }
    
    bool is_null_vector() const {
        return (Compare::is_equal(x_, 0.0) &&
                Compare::is_equal(y_, 0.0) &&
                Compare::is_equal(z_, 0.0));
    }

    bool vectors_are_collinear(const vector_t<T>& other_vector) const {
        return cross_product(other_vector).is_null_vector();
    }

    bool is_equal(const vector_t<T>& other_vector) const {
        return (Compare::is_equal(x_, other_vector.x_) && 
                Compare::is_equal(y_, other_vector.y_) &&
                Compare::is_equal(z_, other_vector.z_));
    }

    void print() const {
        std::cout << "coordinates of vector: ";
        std::cout << x_ << "; " << y_ << "; " << z_ << std::endl;
    }

    T operator[](size_t i) const {
        switch(i) {
            case 0: 
                return x_;

            case 1: 
                return y_;

            case 2:
                return z_;

            default: 
                throw std::out_of_range("invalid axis!");
        }
    }   
};

template <typename T> 
vector_t<T> as_vector(const point_t<T>& point) {
    point_t<T> zero_point{0.0, 0.0, 0.0};
    vector_t<T> result{zero_point, point};
    return result;
}

template <typename T>
vector_t<T> operator-(const point_t<T>& end, const point_t<T>& beg) {
    vector_t<T> result{end.get_x() - beg.get_x(), 
                       end.get_y() - beg.get_y(), 
                       end.get_z() - beg.get_z()};
    return result;
}

template <typename T>
vector_t<T> operator+(const vector_t<T>& first, const vector_t<T>& second) {
    vector_t<T> result{first.get_x() + second.get_x(),
                       first.get_y() + second.get_y(),
                       first.get_z() + second.get_z()};
    return result;
}

template <typename T>
point_t<T> operator+(const point_t<T>& point, const vector_t<T>& vector) {
    point_t<T> result{point.get_x() + vector.get_x(), 
                      point.get_y() + vector.get_y(),
                      point.get_z() + vector.get_z()};
    return result;
}

template <typename T> 
vector_t<T> operator*(const T& coeff, const vector_t<T>& vector) {
    vector_t<T> result{coeff * vector.get_x(),
                       coeff * vector.get_y(), 
                       coeff * vector.get_z()};
    return result;
}

} // namespace Geom_objects

#endif // VECTOR_HPP
