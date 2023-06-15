cmake_minimum_required(VERSION 3.0)
project(dzy CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_subdirectory(deps/vecmath)

set(PA1_SOURCES
        src/image.cpp
        src/main.cpp
        src/scene.cpp
        src/texture.cpp)

set(PA1_INCLUDES
        include/camera.hpp
        include/group.hpp
        include/hit.hpp
        include/image.hpp
        include/light.hpp
        include/material.hpp
        include/mesh.hpp
        include/object3d.hpp
        include/plane.hpp
        include/ray.hpp
        include/scene.hpp
        include/sphere.hpp
        include/transform.hpp
        include/triangle.hpp
        include/texture.h
        include/hit_kdtree.hpp
        include/object_kdtree.hpp
        include/render.hpp
        )

add_executable(${PROJECT_NAME} ${PA1_SOURCES} ${PA1_INCLUDES})
target_link_libraries(${PROJECT_NAME} PRIVATE vecmath)
target_include_directories(${PROJECT_NAME} PRIVATE include)
add_compile_options("$<$<CONFIG:DEBUG>:$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -fopenmp>")
add_compile_options("$<$<CONFIG:RELEASE>:$ENV{CXXFLAGS} -O3 -Wall -fopenmp>")
