cmake_minimum_required(VERSION 3.14)
project(cmucam-viewer VERSION 0.1 LANGUAGES C CXX)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(SDL2 REQUIRED)

add_subdirectory(dependencies)
add_subdirectory(src)
