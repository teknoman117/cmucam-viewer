cmake_minimum_required(VERSION 3.14)

# Create library for ImGui
add_library(imgui STATIC
    ${CMAKE_CURRENT_LIST_DIR}/imgui/imgui.cpp
    ${CMAKE_CURRENT_LIST_DIR}/imgui/imgui_demo.cpp
    ${CMAKE_CURRENT_LIST_DIR}/imgui/imgui_draw.cpp
    ${CMAKE_CURRENT_LIST_DIR}/imgui/imgui_tables.cpp
    ${CMAKE_CURRENT_LIST_DIR}/imgui/imgui_widgets.cpp
    ${CMAKE_CURRENT_LIST_DIR}/imgui/backends/imgui_impl_sdl2.cpp
    ${CMAKE_CURRENT_LIST_DIR}/imgui/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/imgui>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/imgui/backends>
)

target_link_libraries(imgui PRIVATE
    OpenGL::OpenGL
    SDL2::SDL2
)

target_compile_features(imgui PUBLIC
    c_std_17
    cxx_std_17
)