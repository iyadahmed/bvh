#include <iostream>

#include "bvh.hpp"
#include "ray_intersection.hpp"
#include "subdivision.hpp"
#include "utils.hpp"

namespace BVH
{

    AABBTree::AABBTree(const std::vector<Triangle> &tris, float aabb_expansion) : tris(tris)
    {
        preallocated_nodes = new Node[2 * tris.size()];

        root = new_node(this->tris.begin(), this->tris.end());
        subdivide((Node *)root, aabb_expansion);
        assert(count_leaf_triangles((Node *)root) == tris.size());
    }

    AABBTree::~AABBTree()
    {
        delete[] preallocated_nodes;
    }

    Node *AABBTree::new_node(std::vector<Triangle>::iterator begin, std::vector<Triangle>::iterator end)
    {
        assert(num_used_nodes < (2 * tris.size()));
        Node *node = preallocated_nodes + (num_used_nodes++);
        node->begin = begin;
        node->end = end;
        return node;
    }

    bool AABBTree::does_intersect_ray(Vector4 origin, Vector4 direction, float *t_out) const
    {
        Ray ray(origin, direction);
        intersect_ray_bvh(ray, (Node *)root);
        *t_out = ray.get_t();
        return ray.get_t() < std::numeric_limits<float>::max();
    }

    void AABBTree::print_stats() const
    {
        std::cout << "Num. BVH triangles = " << tris.size() << std::endl;
        std::cout << "Num. BVH leaf nodes = " << count_leaf_nodes((Node *)root) << std::endl;
    }

}
