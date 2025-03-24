#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <array>

#include "polygons.hpp"
#include "bounding_box.hpp"
#include "octree.hpp"

int main() {
    size_t number_of_polygons = 0;

    std::cin >> number_of_polygons;
    if (!std::cin.good()) {
        std::cerr << "Error input" << std::endl;
        return -1;
    }

    std::list<Geom_objects::polygon_t<double>> polygons{};

    Geom_objects::point_t<double> point_1, point_2, point_3;

    double x_coordinate, y_coordinate, z_coordinate;

    double max_x_coordinate = 0;
    double max_y_coordinate = 0;
    double max_z_coordinate = 0;

    for (size_t polygon_counter = 0; polygon_counter < number_of_polygons; ++polygon_counter) {
        if (!(std::cin >> x_coordinate >> y_coordinate >> z_coordinate)) {
            std::cerr << "Error reading point 1 for triangle " << polygon_counter << std::endl;
            return -1;
        }
        point_1 = {x_coordinate, y_coordinate, z_coordinate, polygon_counter};

        if (!(std::cin >> x_coordinate >> y_coordinate >> z_coordinate)) {
            std::cerr << "Error reading point 2 for triangle " << polygon_counter << std::endl;
            return -1;
        }
        point_2 = {x_coordinate, y_coordinate, z_coordinate, polygon_counter};

        if (!(std::cin >> x_coordinate >> y_coordinate >> z_coordinate)) {
            std::cerr << "Error reading point 3 for triangle " << polygon_counter << std::endl;
            return -1;
        }
        point_3 = {x_coordinate, y_coordinate, z_coordinate, polygon_counter};

        polygons.push_back(Geom_objects::make_geometric_primitive(point_1, point_2, point_3));

        max_x_coordinate = std::max({max_x_coordinate, point_1.get_x(), point_2.get_x(), point_3.get_x()});
        max_y_coordinate = std::max({max_y_coordinate, point_1.get_y(), point_2.get_y(), point_3.get_y()});
        max_z_coordinate = std::max({max_z_coordinate, point_1.get_z(), point_2.get_z(), point_3.get_z()});
    }   

    std::array<double, Geom_objects::AABB_t<double>::number_of_edges> box_edges{std::abs(max_x_coordinate),
                                                                                std::abs(max_y_coordinate), 
                                                                                std::abs(max_z_coordinate)};

    Geom_objects::point_t<double> middle_of_space{0.0, 0.0, 0.0};
    Geom_objects::AABB_t<double> bounding_box{middle_of_space, box_edges};

    Octree::octree_t<double> octree{polygons.begin(), polygons.end(), bounding_box};

    std::set<size_t> result{};
    
    octree.get_number_of_intersections(result);

    for (size_t number : result) 
        std::cout << number << std::endl;

    return 0;
}