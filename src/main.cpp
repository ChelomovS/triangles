#include <iostream>
#include <list>
#include <set>

#include "octree.hpp"
#include "triangle.hpp"
#include "point.hpp"


int main() {
    size_t number_of_triangles = 0;

    std::cin >> number_of_triangles;
    if (!std::cin.good()) {
        std::cerr << "Error input" << std::endl;
        return 1;
    }

    std::list<Triangle::triangle_t> triangles{};

    Point::point_t point_1, point_2, point_3;
    double x_coordinate, y_coordinate, z_coordinate;

    double max_x_coordinate = 0;
    double max_y_coordinate = 0;
    double max_z_coordinate = 0;

    for (size_t triangle_counter = 0; triangle_counter < number_of_triangles; ++triangle_counter) {
        std::cin >> x_coordinate >> y_coordinate >> z_coordinate;

        point_1 = {x_coordinate, y_coordinate, z_coordinate};

        std::cin >> x_coordinate >> y_coordinate >> z_coordinate;

        point_2 = {x_coordinate, y_coordinate, z_coordinate};

        std::cin >> x_coordinate >> y_coordinate >> z_coordinate;

        point_3 = {x_coordinate, y_coordinate, z_coordinate};
    
        Triangle::triangle_t new_triangle{point_1, point_2, point_3, triangle_counter};
        triangles.push_back(new_triangle);

        max_x_coordinate = std::max({max_x_coordinate, point_1.get_x(), point_2.get_x(), point_3.get_x()});
        max_y_coordinate = std::max({max_y_coordinate, point_1.get_y(), point_2.get_y(), point_3.get_y()});
        max_z_coordinate = std::max({max_z_coordinate, point_1.get_z(), point_2.get_z(), point_3.get_z()});
    }

    Point::point_t bounding_box{std::abs(max_x_coordinate), std::abs(max_y_coordinate), 
                                std::abs(max_z_coordinate)};

    Octree::octree_t octree{triangles.begin(), triangles.end(), bounding_box};

    std::set<size_t> result{};

    octree.inresect_triangles_inside_node(result);

    for (size_t number : result) 
       std::cout << number << std::endl;

    return 0;
}