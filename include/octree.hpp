#ifndef OCTREE_HPP
#define OCTREE_HPP

#include <iterator>
#include <vector>
#include <list>
#include <cassert>
#include <stack>
#include <array>

#include "bounding_box.hpp"
#include "point.hpp"
#include "triangle.hpp"

namespace Octree {

const size_t number_of_children = 8;
const size_t min_size           = 50;

template <typename T>
class octree_node_t {
    public:
    Geom_objects::AABB_t<T> bounding_box_; 
    size_t depth = 0; 
    std::vector<Geom_objects::polygon_t<T>> polygons_in_space_;
    const octree_node_t<T>* parent_;
    std::array<octree_node_t<T>*, number_of_children> children_ = {nullptr, nullptr, nullptr, nullptr, 
                                                                   nullptr, nullptr, nullptr, nullptr};
    std::array<bool, number_of_children> valid_children_ = {false, false, false, false,
                                                            false, false, false, false};
    bool is_leaf_ = true;

    public:
    octree_node_t(const Geom_objects::AABB_t<T>& bounding_box, const octree_node_t<T>* parent_node):
    bounding_box_{bounding_box}, parent_{parent_node} {};
};

template <typename T>
class octree_t {
    private:
    octree_node_t<T>* root_ = nullptr;
    
    public:
    template <typename PolygonsIterator>
    octree_t(PolygonsIterator begin, PolygonsIterator end, const Geom_objects::AABB_t<T>& bounding_box) {
        if (begin == end)
            return;

        root_ = new octree_node_t<T>{bounding_box, nullptr};

        for (auto polygon_iter = begin; polygon_iter != end; ++polygon_iter) {
            root_->polygons_in_space_.emplace_back(*polygon_iter);
        }

        subdivide(root_); 
    }

    octree_node_t<T>* get_root() const { return root_; };

    void subdivide(octree_node_t<T>* root) {
        if (root == nullptr)
            return;

        if (root->polygons_in_space_.size() < min_size)
            return;

        std::stack<octree_node_t<T>*> node_stack;
        node_stack.push(root);
        while (!node_stack.empty()) {
            auto current_node = node_stack.top();
            node_stack.pop();
            size_t begin_size = current_node->polygons_in_space_.size();

            std::array<T, Geom_objects::number_of_edges> halfs_of_edges{current_node->bounding_box_.get_box_x_edge() / 2,
                                                                        current_node->bounding_box_.get_box_y_edge() / 2,
                                                                        current_node->bounding_box_.get_box_z_edge() / 2};
 
            T middle_point_x, middle_point_y, middle_point_z;

            for (size_t number_of_child = 0; number_of_child < number_of_children; ++number_of_child) {
                middle_point_x = current_node->bounding_box_.get_middle_point().get_x() + 
                                ((number_of_child & 1) ? -halfs_of_edges[0] : halfs_of_edges[0]);

                middle_point_y = current_node->bounding_box_.get_middle_point().get_y() + 
                                ((number_of_child & 2) ? -halfs_of_edges[1] : halfs_of_edges[1]);

                middle_point_z = current_node->bounding_box_.get_middle_point().get_z() + 
                                ((number_of_child & 4) ? -halfs_of_edges[2] : halfs_of_edges[2]);

                Geom_objects::point_t<T> new_middle_point{middle_point_x, middle_point_y, middle_point_z};
                Geom_objects::AABB_t<T> new_bounding_box{new_middle_point, halfs_of_edges};

                current_node->children_[number_of_child] = new octree_node_t<T>{new_bounding_box, 
                                                                                current_node};
            }

            std::vector<Geom_objects::polygon_t<T>> polygons_to_move;
            polygons_to_move.swap(current_node->polygons_in_space_);

            for (auto polygon : polygons_to_move) {
                bool moved = false;
                for (size_t number_of_child = 0; number_of_child < number_of_children; ++number_of_child) {
                    if (current_node->children_[number_of_child]->bounding_box_.is_polygon_inside_box(polygon)) {
                        current_node->children_[number_of_child]->polygons_in_space_.push_back(polygon);
                        current_node->valid_children_[number_of_child] = true;
                        moved = true;
                    }
                }
                if (!moved) {
                    current_node->polygons_in_space_.push_back(polygon);
                }
            }

            if (begin_size - current_node->polygons_in_space_.size()) 
                current_node->is_leaf_ = false;

            for (size_t number_of_child = 0; number_of_child < number_of_children; ++number_of_child) {
                if (current_node->valid_children_[number_of_child])
                    node_stack.push(current_node->children_[number_of_child]);
            }
        }
    }
        
    void intersect_polygons_inside_node(octree_node_t<T>* current_node, std::set<size_t>& result) {
        if (current_node == nullptr) 
            return;

        // Used a stack to avoid recursion
        std::stack<octree_node_t<T>*> node_stack;
        node_stack.push(current_node);

        while (!node_stack.empty()) {
            octree_node_t<T>* node = node_stack.top();
            node_stack.pop();

            auto& polygons = node->polygons_in_space_;
            for (auto iter_1 = polygons.begin(); iter_1 != polygons.end(); ++iter_1) {
                for (auto iter_2 = std::next(iter_1); iter_2 != polygons.end(); ++iter_2) {
                    if (check_figures_intersection(*iter_1, *iter_2)) {
                        result.insert(get_number(*iter_1));
                        result.insert(get_number(*iter_2));
                    }
                }
                intersect_polygons_with_children(result, *iter_1, node);
            }

            for (size_t number_of_child = 0; number_of_child < number_of_children; ++number_of_child) {
                if (node->valid_children_[number_of_child]) {
                    node_stack.push(node->children_[number_of_child]);
                }
            }
        }
    }

    void delete_tree(octree_node_t<T>* root) {
        if (root == nullptr) 
            return;

        // Used a stack to avoid recursion
        std::stack<octree_node_t<T>*> node_stack;
        node_stack.push(root);

        while (!node_stack.empty()) {
            auto node = node_stack.top();
            node_stack.pop();

            for (size_t number_of_child = 0; number_of_child < number_of_children; ++number_of_child) {
                if (node->children_[number_of_child] != nullptr) {
                    node_stack.push(node->children_[number_of_child]);
                }
            }
            delete node;
        }
    }

    ~octree_t() {
        delete_tree(root_);
    }                                          
};

template <typename T>
void intersect_polygons_with_children(std::set<size_t>& result, const Geom_objects::polygon_t<T>& polygone, 
                                      const octree_node_t<T>* current_node) {
    if (current_node == nullptr)
        return;

    // Used a stack to avoid recursion
    std::stack<const octree_node_t<T>*> node_stack;
    node_stack.push(current_node);

    while (!node_stack.empty()) {
        const octree_node_t<T>* node = node_stack.top();
        node_stack.pop();

        if (node->is_leaf_)
            continue;

        for (size_t number_of_child = 0; number_of_child < number_of_children; ++number_of_child) {
            if (!node->valid_children_[number_of_child])
                continue;

            const octree_node_t<T>* child = node->children_[number_of_child];
            if (!child->bounding_box_.is_polygon_part_inside_box(polygone))
                continue;

            auto child_polygons = child->polygons_in_space_;
            for (auto child_polygon : child_polygons) {
                if (check_figures_intersection(polygone, child_polygon)) {
                    result.insert(get_number(child_polygon));
                    result.insert(get_number(polygone));
                }
            }

            node_stack.push(child);
        }
    }
}

} // namespace Octree

#endif // OCTREE_HPP