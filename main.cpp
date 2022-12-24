#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <SDL.h>

#include "bvh.hpp"
#include "vec4.hpp"
#include "camera.hpp"

Camera cam;

static void render(SDL_Renderer* renderer, const BVH::BVH& bvh)
{
    int width, height;
    SDL_GetRendererOutputSize(renderer, &width, &height);

    float fov = cam.get_fov();
    float d = std::tan(fov / 2);
    Vector4 cam_pos = cam.get_pos();
    Vector4 up;
    Vector4 right;
    Vector4 forward;
    cam.calc_vectors(&up, &right, &forward);

    float aspect_ratio;
    if (width > height) {
        aspect_ratio = width / (float) height;
    } else {
        aspect_ratio = height / (float)width;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {

            float xn = x / (float)width;
            xn = 2 * xn - 1;
            xn *= aspect_ratio;
            float yn = y / (float)height;
            yn = 1 - 2 * yn;

            Vector4 pixel_pos = cam_pos + forward + right * d * xn + up * d * yn;

            Vector4 ray_origin = cam_pos;
            Vector4 ray_direction = (pixel_pos - cam_pos).normalized3();

            float t = 0.0f;
            if (bvh.does_intersect_ray(ray_origin, ray_direction, &t)) {
                // Map t from [0, inf[ to [0, 1[
                // https://math.stackexchange.com/a/3200751/691043
                float tr = std::atan(t) / (3.14/2);
                unsigned char c = (tr * tr) * 255;

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

    constexpr int WINDOW_WIDTH = 640;
    constexpr int WINDOW_HEIGHT = 640;
    constexpr float ROTATION_SPEED = .001;
    constexpr float MOVEMENT_SPEED = .1;

    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window, &renderer);
    SDL_SetRelativeMouseMode(SDL_TRUE);

    bool is_running = true;
    int32_t dx, dy;
    while (true) {
        while (SDL_PollEvent(&event) != 0) {
            Vector4 up;
            Vector4 right;
            Vector4 forward;
            cam.calc_vectors(&up, &right, &forward);

            switch (event.type) {
            case SDL_QUIT:
                is_running = false;
                break;
            case SDL_MOUSEMOTION:
                dx = event.motion.xrel;
                dy = event.motion.yrel;
                cam.rotate(dx * ROTATION_SPEED, dy * ROTATION_SPEED);
                break;
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE) { is_running = false; }
                else if (event.key.keysym.sym == SDLK_w) {
                    cam.move(forward * MOVEMENT_SPEED);
                }
                else if (event.key.keysym.sym == SDLK_s) {
                    cam.move(forward * MOVEMENT_SPEED * -1);
                }
                else if (event.key.keysym.sym == SDLK_a) {
                    cam.move(right * MOVEMENT_SPEED * -1);
                }
                else if (event.key.keysym.sym == SDLK_d) {
                    cam.move(right * MOVEMENT_SPEED);
                }
                break;
            }
        }

        if (!is_running)
            break;

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        render(renderer, bvh);
        SDL_RenderPresent(renderer);
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}