cmake_minimum_required(VERSION 3.16)

project(bvh)

include(FetchContent) # Needs to be included first
Set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
	sdl
	GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
	GIT_TAG release-2.26.1
	GIT_SHALLOW    ON
	GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(sdl)

add_subdirectory(tiny_stl)
add_executable(bvh "bvh.cpp" "vec4.hpp")
target_link_libraries(bvh PUBLIC tiny_stl SDL2::SDL2 SDL2::SDL2main)

if(MSVC)
  target_link_options(bvh PRIVATE $<$<CONFIG:RELWITHDEBINFO>:/PROFILE>)
endif()