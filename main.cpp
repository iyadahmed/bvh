#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include <SDL.h>

#include "bvh.hpp"
#include "vec4.hpp"
#include "camera.hpp"
#include "main.hpp"

constexpr int WINDOW_WIDTH = 640;
constexpr int WINDOW_HEIGHT = 640;
constexpr float ROTATION_SPEED = .001;
constexpr float MOVEMENT_SPEED = .1;

Camera cam({0, -2, 0}, {0, 0, 0});

struct Color {
    unsigned char a, b, g, r;
};

static void render(Color *pixels, const BVH::BVH &bvh, int width, int height) {

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
        aspect_ratio = height / (float) width;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
    for (int i = 0; i < width * height; i++) {
        int x = i % width;
        int y = i / width;
        float xn = x / (float) width;
        xn = 2 * xn - 1;
        xn *= aspect_ratio;
        float yn = y / (float) height;
        yn = 1 - 2 * yn;

        Vector4 pixel_pos = cam_pos + forward + right * d * xn + up * d * yn;

        Vector4 ray_origin = cam_pos;
        Vector4 ray_direction = (pixel_pos - cam_pos).normalized3();

        float t = 0.0f;
        if (bvh.does_intersect_ray(ray_origin, ray_direction, &t)) {
            // Map t from [0, inf[ to [0, 1[
            // https://math.stackexchange.com/a/3200751/691043
            float tr = std::atan(t) / (3.14 / 2);
            unsigned char c = (tr * tr) * 255;
            pixels[x + y * width] = {255, c, c, c};
        } else {
            pixels[x + y * width] = {255, 0, 0, 0};
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Rendering took: " << (t2 - t1).count() / 1'000'000 << " milli seconds" << std::endl;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        puts("Expected arguments: mesh.[stl|tri]");
        return 1;
    }

    const char *filepath = argv[1];
    std::vector<BVH::Triangle> tris = load_bvh_tris_from_mesh_file(filepath);
    std::cout << "Loaded " << tris.size() << " triangles from " << filepath << std::endl;
    BVH::BVH bvh(tris);
    bvh.print_stats();

    SDL_Event event;
    SDL_Renderer *renderer;
    SDL_Window *window;

    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window, &renderer);
    SDL_SetRelativeMouseMode(SDL_TRUE);

    bool is_running = true;
    int32_t dx, dy;
    auto *pixels = static_cast<Color *>(malloc(WINDOW_WIDTH * WINDOW_HEIGHT * 4));

    SDL_Texture *buffer = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING,
                                            WINDOW_WIDTH, WINDOW_HEIGHT);

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
                    } else if (event.key.keysym.sym == SDLK_s) {
                        cam.move(forward * MOVEMENT_SPEED * -1);
                    } else if (event.key.keysym.sym == SDLK_a) {
                        cam.move(right * MOVEMENT_SPEED * -1);
                    } else if (event.key.keysym.sym == SDLK_d) {
                        cam.move(right * MOVEMENT_SPEED);
                    }
                    break;
            }
        }

        if (!is_running)
            break;

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        render(pixels, bvh, WINDOW_WIDTH, WINDOW_HEIGHT);
        SDL_UpdateTexture(buffer, nullptr, pixels, WINDOW_WIDTH * 4);
        SDL_RenderCopy(renderer, buffer, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    free(pixels);

    return 0;
}