#include <iostream>

#include "bvh.hpp"
#include "utils.hpp"
#include "subdivision.hpp"
#include "ray_intersection.hpp"

namespace BVH {

    BVH::BVH(const std::vector<Triangle> &tris) : tris(tris) {
        preallocated_nodes = new Node[2 * tris.size()];

        root = new_node(this->tris.begin(), this->tris.end());
        subdivide((Node *) root);
        assert(count_leaf_triangles((Node *) root) == tris.size());
    }

    BVH::~BVH() {
        delete[] preallocated_nodes;
    }

    Node* BVH::new_node(std::vector<Triangle>::iterator begin, std::vector<Triangle>::iterator end) {
        assert(num_used_nodes < (2 * tris.size()));
        Node* node = preallocated_nodes + (num_used_nodes++);
        node->begin = begin;
        node->end = end;
        return node;
    }

    bool BVH::does_intersect_ray(Vector4 origin, Vector4 direction, float *t_out) const {
        Ray ray(origin, direction);
        intersect_ray_bvh(ray, (Node *) root);
        *t_out = ray.get_t();
        return ray.get_t() < std::numeric_limits<float>::max();
    }

    void BVH::print_stats() const {
        int num_leaf_nodes = count_leaf_nodes((Node *) root);
        int quality = (num_leaf_nodes * 100) / tris.size();
        std::cout << "Number of BVH triangles = " << tris.size() << std::endl;
        std::cout
                << "Number of reachable triangles (should be equal to the number of BVH triangles, otherwise it means triangles were lost when building the BVH somehow) = "
                << count_leaf_triangles((Node *) root) << std::endl;
        std::cout << "BVH Tree quality (n. leaf nodes / n. triangles) = " << quality << "%" << std::endl;
    }

}
