#pragma once

#include <vector>
#include <cstdio>
#include <string>
#include <stdexcept>

#include "bvh.hpp"
#include "tiny_stl/tiny_stl.hpp"

std::vector<BVH::Triangle> bvh_tris_from_tri_file(const char *filepath) {
    std::vector<BVH::Triangle> tris;
    FILE *file = fopen(filepath, "r");
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
        tris.push_back({v1, v2, v3});
    }
    fclose(file);
    return tris;
}

std::vector<BVH::Triangle> bvh_tris_from_stl_file(const char *filepath) {
    auto reader = STL_Mesh_IO::create_reader(filepath);
    std::vector<BVH::Triangle> tris;
    STL_Mesh_IO::Triangle t;
    BVH::Triangle bt;
    while (reader->read_next_triangle(&t)) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                bt.vertices[i][j] = t.vertices[i][j];
            }
        }
        tris.push_back(bt);
    }
    return tris;
}

bool ends_with(const std::string &str, const std::string &suffix) {
    if (str.length() < suffix.length()) {
        return false;
    }

    size_t j = 0;
    for (size_t i = str.length() - suffix.length(); i < str.length(); i++) {
        if (str[i] != suffix[j]) {
            return false;
        }
        j++;
    }

    return true;
}

std::vector<BVH::Triangle> load_bvh_from_mesh_file(const std::string &filepath) {
    if (ends_with(filepath, ".stl")) {
        return bvh_tris_from_stl_file(filepath.c_str());
    } else if (ends_with(filepath, ".tri")) {
        return bvh_tris_from_tri_file(filepath.c_str());
    } else {
        throw std::runtime_error("Unrecognized file extension");
    }
}