#include <stack>
#include <set>
#include <iostream>

#include "octree.hpp"
#include "double_compare.hpp"
#include "triangle.hpp"

bool Octree::is_triangle_inside_box(const Triangle::triangle_t& triangle, const Point::point_t& middle_point,
                                    const std::array<double, Octree::number_of_edges>& box_edges) {
    return is_point_inside_box(triangle.get_a(), middle_point, box_edges) &&
           is_point_inside_box(triangle.get_b(), middle_point, box_edges) &&
           is_point_inside_box(triangle.get_c(), middle_point, box_edges);
}

bool Octree::is_point_inside_box(const Point::point_t& point, const Point::point_t& middle_point,
                                 const std::array<double, Octree::number_of_edges>& box_edges) {
    double x_max = middle_point.get_x() + box_edges[0];
    double x_min = middle_point.get_x() - box_edges[0];
    double y_max = middle_point.get_y() + box_edges[1];
    double y_min = middle_point.get_y() - box_edges[1];
    double z_max = middle_point.get_z() + box_edges[2];
    double z_min = middle_point.get_z() - box_edges[2];

    return Compare::is_greater_or_equal(point.get_x(), x_min) && Compare::is_less_or_equal(point.get_x(), x_max) &&
           Compare::is_greater_or_equal(point.get_y(), y_min) && Compare::is_less_or_equal(point.get_y(), y_max) &&
           Compare::is_greater_or_equal(point.get_z(), z_min) && Compare::is_less_or_equal(point.get_z(), z_max);
}

void Octree::octree_t::inresect_triangles_inside_node(std::set<size_t>& result) {
    if (root_ == nullptr) 
        return;

    // Used a stack to avoid recursion
    std::stack<const Octree::octree_node_t*> node_stack;
    node_stack.push(root_);

    while (!node_stack.empty()) {
        auto current_node = node_stack.top();
        node_stack.pop();

        auto begin_of_triangle_list = current_node->triangles_in_space_.begin();
        auto end_of_triangle_list   = current_node->triangles_in_space_.end();

        for (auto iter_1 = begin_of_triangle_list; iter_1 != end_of_triangle_list; ++iter_1) {
            for (auto iter_2 = iter_1; iter_2 != end_of_triangle_list; ++iter_2) {
                if (iter_1->get_number() == iter_2->get_number())
                    continue;

                if (iter_1->triangles_intersects_in_3d(*iter_2)) {
                    result.insert(iter_1->get_number());
                    result.insert(iter_2->get_number());
                }
            }

            intersect_triangles_with_children(result, *iter_1, current_node);
        }

        for (size_t number_of_child = 0; number_of_child < Octree::number_of_children; ++number_of_child) {
            if (current_node->valid_children_[number_of_child]) {
                node_stack.push(current_node->children_[number_of_child]);
            }
        }
    }
}

void Octree::intersect_triangles_with_children(std::set<size_t>& result, Triangle::triangle_t triangle, 
                                               const Octree::octree_node_t* node) {
    if (node == nullptr)
        return;
    if (node->is_leaf)
        return;

    // Used a stack to avoid recursion
    std::stack<const Octree::octree_node_t*> node_stack;
    node_stack.push(node);

    while (!node_stack.empty()) {
        auto current_node = node_stack.top();
        node_stack.pop();

        for (size_t number_of_child = 0; number_of_child < Octree::number_of_children; ++number_of_child) {
            if (current_node->valid_children_[number_of_child]) {
                auto begin_of_child_triangle_list = 
                                        current_node->children_[number_of_child]->triangles_in_space_.begin();
                auto end_of_child_triangle_list = 
                                        current_node->children_[number_of_child]->triangles_in_space_.end();

                for (auto list_iter = begin_of_child_triangle_list; list_iter != end_of_child_triangle_list; 
                     ++list_iter) {
                    if (list_iter->triangles_intersects_in_3d(triangle)) {
                        result.insert(list_iter->get_number());
                        result.insert(triangle.get_number());
                    }
                }
                node_stack.push(current_node->children_[number_of_child]);
            }
        }
    }
}

void Octree::octree_t::subdivide(Octree::octree_node_t* root) {
    if (root == nullptr)
        return;

    // Used a stack to avoid recursion
    std::stack<Octree::octree_node_t*> node_stack;
    node_stack.push(root);

    while (!node_stack.empty()) {
        auto current_node = node_stack.top();
        node_stack.pop();

        size_t begin_size = current_node->triangles_in_space_.size();
        if (begin_size <= Octree::min_size)
            continue;

        double middle_of_x_edge = current_node->box_edges_[0] / 2;
        double middle_of_y_edge = current_node->box_edges_[1] / 2;
        double middle_of_z_edge = current_node->box_edges_[2] / 2;

        double middle_point_x, middle_point_y, middle_point_z;

        std::array<double, Octree::number_of_edges> new_box_edges{middle_of_x_edge, 
                                                                  middle_of_y_edge,
                                                                  middle_of_z_edge};

        for (size_t number_of_child = 0; number_of_child < Octree::number_of_children; ++number_of_child) {
            if (number_of_child % 2 == 0) 
                middle_point_x = current_node->middle_point_.get_x() + middle_of_x_edge;
            else
                middle_point_x = current_node->middle_point_.get_x() - middle_of_x_edge;

            if (number_of_child % 4 == 0)
                middle_point_y = current_node->middle_point_.get_y() + middle_of_y_edge;
            else
                middle_point_y = current_node->middle_point_.get_y() - middle_of_y_edge;

            if (number_of_child % 8 == 0)
                middle_point_z = current_node->middle_point_.get_z() + middle_of_z_edge;
            else
                middle_point_z = current_node->middle_point_.get_z() - middle_of_z_edge;

            Point::point_t new_middle_point{middle_point_x, middle_point_y, middle_point_z};

            current_node->children_[number_of_child] = new Octree::octree_node_t{new_middle_point, 
                                                                                 new_box_edges, 
                                                                                 current_node};
        }

        auto begin_of_triangle_list = current_node->triangles_in_space_.begin();
        auto end_of_triangle_list   = current_node->triangles_in_space_.end();

        for (auto list_iter = begin_of_triangle_list; list_iter != end_of_triangle_list; ++list_iter) {
            for (size_t number_of_child = 0; number_of_child < Octree::number_of_children; ++number_of_child) {
                if (Octree::is_triangle_inside_box(*list_iter, current_node->children_[number_of_child]->middle_point_, 
                                                   new_box_edges)) {
                    current_node->valid_children_[number_of_child] = true;                                          
                    current_node->children_[number_of_child]->triangles_in_space_.push_back(*list_iter);
                    list_iter = current_node->triangles_in_space_.erase(list_iter);
                    break;
                }
            }
        }

        if (begin_size == current_node->triangles_in_space_.size())
            current_node->is_leaf = true;

        for (size_t number_of_child = 0; number_of_child < Octree::number_of_children; ++number_of_child) {
            if (current_node->children_[number_of_child]->triangles_in_space_.size() >= Octree::min_size) {
                current_node->valid_children_[number_of_child] = true;
                node_stack.push(current_node->children_[number_of_child]);
            } else {
                current_node->valid_children_[number_of_child] = false;
            }
        }
    }
}

void Octree::octree_t::delete_tree(Octree::octree_node_t* root) {
    if (root == nullptr) 
        return;

    // Used a stack to avoid recursion
    std::stack<Octree::octree_node_t*> node_stack;
    node_stack.push(root);

    while (!node_stack.empty()) {
        auto node = node_stack.top();
        node_stack.pop();

        for (size_t number_of_child = 0; number_of_child < Octree::number_of_children; ++number_of_child) {
            if (node->children_[number_of_child] != nullptr) {
                node_stack.push(node->children_[number_of_child]);
            }
        }

        delete node;
    }
}