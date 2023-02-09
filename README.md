## BVH

Implementation of Bounding Volume Hierarchy algorithm in C++ using variance metric for splitting, SIMD and std::partition, includes a simple ray tracing demo in SDL2

Stanford_Bunny.stl license:
By Makerbot - https://thingiverse.com/thing:88208/files, CC BY 3.0, https://commons.wikimedia.org/w/index.php?curid=86235456

## How to run
note: CMake configure step needs an internet connection for now
1. clone the repo: `git clone https://github.com/iyadahmed/bvh.git`
2. build and run:
```
cd bvh
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_SYSTEM_SDL=OFF
cmake --build . --config Release
./raytrace ../Stanford_Bunny.stl
```
