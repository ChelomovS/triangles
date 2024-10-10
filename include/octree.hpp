#ifndef OCTREE_HPP
#define OCTREE_HPP

#include <iterator>
#include <vector>
#include <list>
#include <cassert>

#include "point.hpp"
#include "triangle.hpp"

namespace Octree {

const size_t number_of_children = 8;
const size_t number_of_edges    = 3;
const size_t min_size           = 1;

class octree_node_t {
    public:
    std::list<Triangle::triangle_t> triangles_in_space_;
    const octree_node_t* parent_;
    std::array<octree_node_t*, number_of_children> children_ = {nullptr, nullptr, nullptr, nullptr, 
                                                                nullptr, nullptr, nullptr, nullptr};

    std::array<bool, number_of_children> valid_children_ = {false, false, false, false,
                                                            false, false, false, false};

    std::array<double, number_of_edges> box_edges_;
    const Point::point_t middle_point_;
    bool is_leaf = false;

    public:
    octree_node_t(const Point::point_t& middle_point, const std::array<double, number_of_edges>& box_edges,
                  const octree_node_t* parent_node):
    parent_{parent_node}, box_edges_{box_edges}, middle_point_{middle_point} {};
};

class octree_t {
    private:
    octree_node_t* root_ = nullptr;

    public:
    template <typename TrianglesIterator>
    octree_t(TrianglesIterator begin, TrianglesIterator end, const Point::point_t& bounding_box) {
        assert(begin != end);

        const std::array<double, number_of_edges> box_edges = {bounding_box.get_x(), bounding_box.get_y(), 
                                                               bounding_box.get_z()};

        const Point::point_t middle_point{0.0, 0.0, 0.0};

        root_ = new octree_node_t{middle_point, box_edges, nullptr};

        for (auto triangle_iter = begin; triangle_iter != end; ++triangle_iter) {
            root_->triangles_in_space_.push_back(*triangle_iter);
        }
        subdivide(root_);
    }
    const octree_node_t* get_root() const { return root_; };
    void subdivide(octree_node_t* root);  
    void inresect_triangles_inside_node(std::set<size_t>& result);
    void delete_tree(octree_node_t* root);

    ~octree_t() {
        delete_tree(root_);
    }                                          
};

void intersect_triangles_with_children(std::set<size_t>& result, Triangle::triangle_t triangle, 
                                       const octree_node_t* node);
    
bool is_triangle_inside_box(const Triangle::triangle_t& triangle, const Point::point_t& middle_point,
                            const std::array<double, Octree::number_of_edges>& box_edges);
    
bool is_point_inside_box(const Point::point_t& point, const Point::point_t& middle_point,
                         const std::array<double, Octree::number_of_edges>& box_edges);

} // namespace Octree

#endif // OCTREE_HPP