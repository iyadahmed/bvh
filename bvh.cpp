#include "bvh.hpp"
#include "bvh_internal.hpp"

namespace BVH {

    BVH::BVH(const std::vector<Triangle> &tris)
            : tris(tris) {
        root = new Node(this->tris.begin(), this->tris.end());
        subdivide((Node *) root);
        assert(count_leaf_triangles((Node *) root) == tris.size());
    }

    BVH::~BVH() {
        if (root) {
            free_tree((Node *) root);
        }
    }

    bool BVH::does_intersect_ray(Vector4 origin, Vector4 direction, float *t_out) const {
        Ray ray(origin, direction);
        intersect_ray_bvh(ray, (Node *) root);
        *t_out = ray.t;
        return ray.t < std::numeric_limits<float>::max();
    }

}
