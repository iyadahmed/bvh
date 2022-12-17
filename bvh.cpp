#include <algorithm> // for std::partition
#include <assert.h>
#include <iostream>
#include <limits>
#include <stdio.h>
#include <vector>

#include <tiny_stl/tiny_stl.hpp>

#include "vec4.hpp"

struct Triangle {
    Vector4 vertices[3];

    Vector4 calc_centroid() const
    {
        return (vertices[0] + vertices[1] + vertices[2]) / 3;
    }
};

struct Node {
    std::vector<Triangle>::iterator begin, end;
    Node *left = nullptr, *right = nullptr;

    Node(std::vector<Triangle>::iterator begin, std::vector<Triangle>::iterator end)
    {
        this->begin = begin;
        this->end = end;
        left = nullptr;
        right = nullptr;
    }

    bool is_leaf() const
    {
        return (left == nullptr) && (right == nullptr);
    }
};

static int count_nodes(Node* node)
{
    if (node == nullptr)
        return 0;

    return 1 + count_nodes(node->left) + count_nodes(node->right);
}

// https://stackoverflow.com/a/9181223/8094047
static void free_tree(Node* node)
{
    if (node == nullptr)
        return;

    free_tree(node->left);
    free_tree(node->right);

    delete node;
}

static int count_leaf_triangles(Node* node)
{
    if (node == nullptr) {
        return 0;
    } else if (node->is_leaf()) {
        return std::abs(std::distance(node->begin, node->end));
    } else {
        return count_leaf_triangles(node->left) + count_leaf_triangles(node->right);
    }
}

static void bvh(Node* parent)
{
    auto begin = parent->begin;
    auto end = parent->end;

    if (begin == end)
        return;

    Vector4 upper = Vector4(-1 * std::numeric_limits<float>::infinity());
    Vector4 lower = Vector4(std::numeric_limits<float>::infinity());

    for (std::vector<Triangle>::iterator it = begin; it != end; ++it) {
        for (int i = 0; i < 3; i++) {
            upper = upper.max(it->vertices[i]);
            lower = lower.min(it->vertices[i]);
        }
    }

    Vector4 dims = upper - lower;

    int split_axis = 0;

    if (dims[1] > dims[0]) {
        split_axis = 1;
    }

    if (dims[2] > dims[split_axis]) {
        split_axis = 2;
    }

    float split_pos = lower[split_axis] + dims[split_axis] * 0.5f;

    auto middle = std::partition(begin, end, [split_axis, split_pos](const Triangle& t) {
        return t.calc_centroid()[split_axis] < split_pos;
    });

    if ((middle == begin) || (middle == end)) {
        return;
    }

    Node* left = new Node(begin, middle);
    Node* right = new Node(middle, end);

    parent->left = left;
    parent->right = right;

    bvh(left);
    bvh(right);
}

int main(int argc, char* argv[])
{
    if (argc != 2) {
        puts("Expected arguments: mesh.stl");
        return 1;
    }

    auto reader = STL_Mesh_IO::create_reader(argv[1]);

    std::vector<Triangle> tris;

    STL_Mesh_IO::Triangle t;
    while (reader->read_next_triangle(&t)) {
        tris.push_back({ t.vertices[0], t.vertices[1], t.vertices[2] });
    }

    std::cout << "Number of triangles: " << tris.size() << std::endl;

    Node* root = new Node(tris.begin(), tris.end());

    bvh(root);

    assert(count_leaf_triangles(root) == tris.size());

    free_tree(root);

    return 0;
}