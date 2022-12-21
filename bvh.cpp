#include <algorithm> // for std::partition, std::min/max
#include <assert.h>
#include <iostream>
#include <limits>
#include <stdio.h>
#include <vector>

#include <SDL.h>
#include <tiny_stl/tiny_stl.hpp>

#include "vec4.hpp"

struct AABB {
    Vector4 upper, lower;
};

struct Ray {
    Vector4 O, D;
};

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
    AABB aabb;

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

    assert(begin < end);

    Vector4& upper = parent->aabb.upper;
    Vector4& lower = parent->aabb.lower;

    upper = Vector4(-1 * std::numeric_limits<float>::infinity());
    lower = Vector4(std::numeric_limits<float>::infinity());

    // TODO: calculate variance and split triangles along axis of greatest variance
    Vector4 mean(0.0f);
    Vector4 mean_of_squares(0.0f);
    for (std::vector<Triangle>::iterator it = begin; it != end; ++it) {
        for (int i = 0; i < 3; i++) {
            upper = upper.max(it->vertices[i]);
            lower = lower.min(it->vertices[i]);
        }
    }
    Vector4 variance = mean_of_squares - mean * mean;

    // Expand bounding box by an epsilon;
    // fixes an issue where rays that are tangent to the bounding box miss,
    // hopefully this does not strike back and need extra margins in future,
    // P.S.: this is probably related to numeric precision of instrinsics and order of floating-point operations
    upper = upper + Vector4(std::numeric_limits<float>::epsilon());
    lower = lower - Vector4(std::numeric_limits<float>::epsilon());

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

static bool intersect_ray_aabb(const Ray& ray, const AABB& aabb)
{
    Vector4 t_upper = (aabb.upper - ray.O) / ray.D;
    Vector4 t_lower = (aabb.lower - ray.O) / ray.D;
    Vector4 t_min_v = t_upper.min(t_lower);
    Vector4 t_max_v = t_upper.max(t_lower);

    float t_min = t_min_v[0];
    float t_max = t_max_v[0];

    for (int i = 1; i < 3; i++) {
        t_min = std::max(t_min, t_min_v[i]);
        t_max = std::min(t_max, t_max_v[i]);
    }

    return t_max >= t_min && t_min < std::numeric_limits<float>::max() && t_max > 0;
}

static bool intersect_ray_triangle(const Ray& ray, const Triangle& tri, float* t_out)
{
    const Vector4 edge1 = tri.vertices[1] - tri.vertices[0];
    const Vector4 edge2 = tri.vertices[2] - tri.vertices[0];
    const Vector4 h = ray.D.cross3(edge2);
    const float a = edge1.dot3(h);
    if (a > -0.0001f && a < 0.0001f)
        return false; // ray parallel to triangle
    const float f = 1 / a;
    const Vector4 s = ray.O - tri.vertices[0];
    const float u = f * s.dot3(h);
    if (u < 0 || u > 1)
        return false;
    const Vector4 q = s.cross3(edge1);
    const float v = f * ray.D.dot3(q);
    if (v < 0 || u + v > 1)
        return false;
    const float t = f * edge2.dot3(q);
    *t_out = t;
    return t > 0.0001f;
}

static void intersect_ray_bvh(const Ray& ray, Node* node, float* t_min)
{
    if (!intersect_ray_aabb(ray, node->aabb)) {
        return;
    }

    if (node->is_leaf()) {
        for (std::vector<Triangle>::iterator it = node->begin; it != node->end; ++it) {
            float temp = 0.0f;
            if (intersect_ray_triangle(ray, *it, &temp)) {
                *t_min = std::min(*t_min, temp);
            }
        }
    } else {
        intersect_ray_bvh(ray, node->left, t_min);
        intersect_ray_bvh(ray, node->right, t_min);
    }
}

// RNG - Marsaglia's xor32
static unsigned int seed = 0x12345678;
unsigned int random_uint()
{
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed;
}
float random_float() { return random_uint() * 2.3283064365387e-10f; }
Vector4 random_vec3()
{
    return { random_float(), random_float(), random_float() };
}

int main(int argc, char* argv[])
{
    if (argc != 2) {
        puts("Expected arguments: mesh.tri");
        return 1;
    }

    const char* filepath = argv[1];

    std::vector<Triangle> tris;

    // Loads an STL mesh
    // auto reader = STL_Mesh_IO::create_reader(filepath);
    // STL_Mesh_IO::Triangle t;
    // while (reader->read_next_triangle(&t)) {
    //     tris.push_back({ t.vertices[0], t.vertices[1], t.vertices[2] });
    // }
    // std::cout << "Number of triangles: " << tris.size() << std::endl;

    // Generates random triangles
    // int N = 100;
    // for (int i = 0; i < N; i++) {
    //     Vector4 r0 = random_vec3();
    //     Vector4 r1 = random_vec3();
    //     Vector4 r2 = random_vec3();
    //     Triangle t {};
    //     t.vertices[0] = r0 * 9 - Vector4(5);
    //     t.vertices[1] = t.vertices[0] + r1;
    //     t.vertices[2] = t.vertices[0] + r2;
    //     tris.push_back(t);
    // }

    // Loads .tri mesh file
    FILE* file = fopen(filepath, "r");
    if (file == NULL) {
        throw std::runtime_error("Failed to open file");
    }
    float a, b, c, d, e, f, g, h, i;
    while (fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
               &a, &b, &c, &d, &e, &f, &g, &h, &i)
        == 9) {
        Vector4 v1(a, b, c);
        Vector4 v2(d, e, f);
        Vector4 v3(g, h, i);
        tris.push_back({ v1, v2, v3 });
    }
    fclose(file);

    Node* root = new Node(tris.begin(), tris.end());
    bvh(root);
    assert(count_leaf_triangles(root) == tris.size());

    // Raytrace
    Vector4 cam_pos(-1.5f, -0.2f, -2.5);
    Vector4 p0(-1, 1, 2), p1(1, 1, 2), p2(-1, -1, 2);
    Ray ray;

    // Render to image
    // int image_width = 640;
    // int image_height = 640;

    // FILE* image = fopen("raytrace.ppm", "wb");
    // fputs("P3", image); // "P3" means this is an RGB PPM color image in ASCII
    // fprintf(image, "%d %d 255\n", image_width, image_height);

    // for (int y = 0; y < image_height; y++) {
    //     for (int x = 0; x < image_width; x++) {
    //         ray.O = cam_pos;
    //         Vector4 pixel_pos = ray.O + p0 + (p1 - p0) * (x / (float)image_width) + (p2 - p0) * (y / (float)image_height);
    //         ray.D = (pixel_pos - ray.O).normalized3();

    //        float t_min = std::numeric_limits<float>::max();
    //        intersect_ray_bvh(ray, root, &t_min);

    //        if (t_min < std::numeric_limits<float>::max()) {
    //            unsigned char c = 500 - (int)(t_min * 42);
    //            fprintf(image, "%u %u %u ", c, c, c);
    //        } else {
    //            fprintf(image, "0 0 0 ");
    //        }
    //    }
    //}

    // fclose(image);

    SDL_Event event;
    SDL_Renderer* renderer;
    SDL_Window* window;

    // FIXME: support non square aspect ratio
    constexpr int WINDOW_WIDTH = 640;
    constexpr int WINDOW_HEIGHT = 640;

    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window, &renderer);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    for (int y = 0; y < WINDOW_HEIGHT; y++) {
        for (int x = 0; x < WINDOW_WIDTH; x++) {
            ray.O = cam_pos;
            Vector4 pixel_pos = ray.O + p0 + (p1 - p0) * (x / (float)WINDOW_WIDTH) + (p2 - p0) * (y / (float)WINDOW_HEIGHT);
            ray.D = (pixel_pos - ray.O).normalized3();

            float t_min = std::numeric_limits<float>::max();
            intersect_ray_bvh(ray, root, &t_min);

            if (t_min < std::numeric_limits<float>::max()) {
                unsigned char c = 500 - (int)(t_min * 42);
                SDL_SetRenderDrawColor(renderer, c, c, c, 255);
                SDL_RenderDrawPoint(renderer, x, y);
            } else {
                // TODO: draw background, sky, or do nothing
            }
        }
    }

    free_tree(root);

    SDL_RenderPresent(renderer);
    while (1) {
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT) {
            break;
        }
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}