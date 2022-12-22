#include <chrono>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <SDL.h>

#include "bvh.hpp"
#include "vec4.hpp"

static void render(SDL_Renderer* renderer, const BVH::BVH& bvh, Vector4 cam_pos, Vector4 p0, Vector4 p1, Vector4 p2)
{
    int width, height;
    SDL_GetRendererOutputSize(renderer, &width, &height);

    auto t1 = std::chrono::high_resolution_clock::now();
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Vector4 pixel_pos = cam_pos + p0 + (p1 - p0) * (x / (float)width) + (p2 - p0) * (y / (float)height);

            Vector4 ray_origin = cam_pos;
            Vector4 ray_direction = (pixel_pos - cam_pos).normalized3();

            float t = 0.0f;
            if (bvh.does_intersect_ray(cam_pos, ray_direction, &t)) {
                unsigned char c = 500 - (int)(t * 42);
                SDL_SetRenderDrawColor(renderer, c, c, c, 255);
                SDL_RenderDrawPoint(renderer, x, y);
            } else {
                // TODO: draw background, sky, or do nothing
            }
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Rendering took: " << (t2 - t1).count() / 1'000'000 << " milli seconds" << std::endl;
}

int main(int argc, char* argv[])
{
    if (argc != 2) {
        puts("Expected arguments: mesh.tri");
        return 1;
    }

    const char* filepath = argv[1];

    // Loads .tri mesh file
    std::vector<BVH::Triangle> tris;
    {
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
    }

    BVH::BVH bvh(tris);

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

    Vector4 cam_pos(-1.5f, -0.2f, 2.5);
    Vector4 p0(-1, 1, -2), p1(1, 1, -2), p2(-1, -1, -2);
    render(renderer, bvh, cam_pos, p0, p1, p2);

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