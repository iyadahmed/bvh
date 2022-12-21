#include <cassert>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <limits>
#include <vector>

#include <SDL.h>
#include <tiny_stl/tiny_stl.hpp>

#include "bvh.hpp"
#include "vec4.hpp"

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
    subdivide(root);
    assert(count_leaf_triangles(root) == tris.size());

    // Raytrace
    Vector4 cam_pos(-1.5f, -0.2f, 2.5);
    Vector4 p0(-1, 1, -2), p1(1, 1, -2), p2(-1, -1, -2);

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

    auto t1 = std::chrono::high_resolution_clock::now();
    for (int y = 0; y < WINDOW_HEIGHT; y++) {
        for (int x = 0; x < WINDOW_WIDTH; x++) {
            Vector4 pixel_pos = cam_pos + p0 + (p1 - p0) * (x / (float)WINDOW_WIDTH) + (p2 - p0) * (y / (float)WINDOW_HEIGHT);
            Ray ray(cam_pos, (pixel_pos - cam_pos).normalized3());

            intersect_ray_bvh(ray, root);

            if (ray.t < std::numeric_limits<float>::max()) {
                unsigned char c = 500 - (int)(ray.t * 42);
                SDL_SetRenderDrawColor(renderer, c, c, c, 255);
                SDL_RenderDrawPoint(renderer, x, y);
            } else {
                // TODO: draw background, sky, or do nothing
            }
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Rendering took: " << (t2 - t1).count() / 1'000'000 << " milli seconds" << std::endl;

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