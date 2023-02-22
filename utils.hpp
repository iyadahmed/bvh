#pragma once

#include "bvh.hpp"

namespace BVH
{

    int count_nodes(Node *node)
    {
        if (node == nullptr)
            return 0;

        return 1 + count_nodes(node->left) + count_nodes(node->right);
    }

    // https://stackoverflow.com/a/9181223/8094047
    void free_tree(Node *node)
    {
        if (node == nullptr)
            return;

        free_tree(node->left);
        free_tree(node->right);

        delete node;
    }

    int count_leaf_triangles(Node *node)
    {
        if (node == nullptr)
        {
            return 0;
        }
        else if (node->is_leaf())
        {
            return std::abs(std::distance(node->begin, node->end));
        }
        else
        {
            return count_leaf_triangles(node->left) + count_leaf_triangles(node->right);
        }
    }

    int count_leaf_nodes(Node *node)
    {
        if (node == nullptr)
        {
            return 0;
        }
        else if (node->is_leaf())
        {
            return 1;
        }
        else
        {
            return count_leaf_nodes(node->left) + count_leaf_nodes(node->right);
        }
    }

    bool is_point_above_plane(const Vector4 &point, const Vector4 &plane_normal, const Vector4 &plane_point)
    {
        return plane_normal.dot3(point - plane_point) > 0;
    }

}
