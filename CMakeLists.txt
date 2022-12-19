cmake_minimum_required(VERSION 3.16)

project(bvh)

add_subdirectory(tiny_stl)
add_executable(bvh "bvh.cpp")
target_link_libraries(bvh PUBLIC tiny_stl)

if(MSVC)
  target_link_options(bvh PRIVATE $<$<CONFIG:RELWITHDEBINFO>:/PROFILE>)
endif()