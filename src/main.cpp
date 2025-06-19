/*
 * Copyright (c) 2022 Nathan Lewis <github@nrlewis.dev>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of mosquitto nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <GL/glew.h>
#include <GL/gl.h>

#include <SDL.h>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"

#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include "cmucam.h"
#include "shaders.h"

enum class CMUcamState {
    Idle,
    Frame,
    Mean,
    Tracking,
};

const char *cmucam_path = "/dev/ttyUSB0";
unsigned int window_scale = 1;

void show_help(const char* executable_path) {
    fprintf(stderr, "Usage: %s [FLAGS] [OPTIONS] \n", executable_path);
    fprintf(stderr, "Provides a frame viewer for a CMUcam\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "FLAGS:\n");
    fprintf(stderr, "\t-h, --help\t\tPrints this help information\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "OPTIONS:\n");
    fprintf(stderr, "\t-d, --device <path>\t\tSet the path to the CMUcam device [default: /dev/ttyUSB0]\n");
    fprintf(stderr, "\t-s, --scale <scale>\t\tSet the window size to a multiple of the actual size [default: 1]\n");
    fprintf(stderr, "\n");
}

int process_options(int argc, char** argv) {
    const char* executable_path = argc > 0 ? argv[0] : "";

    static struct option options[] = {
        {"device", required_argument, 0, 'd'},
        {"scale", required_argument, 0, 's'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    int rc = 0;
    int index = 0;
    while ((rc = getopt_long(argc, argv, "d:s:h", options, &index)) != -1) {
        switch (rc) {
            case 'd':
                cmucam_path = optarg;
                break;
            case 's':
                if (sscanf(optarg, "%u", &window_scale) == EOF) {
                    fprintf(stderr, "invalid argument for scale: \"%s\"", optarg);
                    show_help(executable_path);
                    return -EINVAL;
                }
                break;

            case '?':
            case 'h':
                show_help(executable_path);
                return -EINVAL;
        }
    }
    return 0;
}

int main(int argc, char** argv) {
    int rc = process_options(argc, argv);
    if (rc < 0) {
        return rc;
    }

    rc = cmucam_initialize();
    if (rc < 0) {
        return rc;
    }

    int cmucam = cmucam_open(cmucam_path);
    if (cmucam < 0) {
        fprintf(stderr, "Failed to open CMUcam: %d\n", cmucam);
        return EXIT_FAILURE;
    }

    // Setup an OpenGL 4.3 rendering environment
    rc = SDL_Init(SDL_INIT_VIDEO);
    if (rc < 0) {
        fprintf(stderr, "failed to initialize SDL with error: %s (%d)\n", SDL_GetError(), -rc);
        return EXIT_FAILURE;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    int width = CMUCAM_IMAGE_WIDTH * 2 * window_scale;
    int height = CMUCAM_IMAGE_HEIGHT * window_scale;
    SDL_Window *window = SDL_CreateWindow(
            "CMUCam v1.12 Viewer",
            SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED,
            width,
            height,
            SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE);
    if (!window) {
        fprintf(stderr, "failed to create SDL window with error: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }

    SDL_GLContext context = SDL_GL_CreateContext(window);
    if (!context) {
        fprintf(stderr, "failed to create SDL OpenGL context with error: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }

    SDL_GL_SetSwapInterval(1);
    SDL_ShowWindow(window);

    // Setup OpenGL resources
    const GLubyte *version = glGetString(GL_VERSION);
    printf("OpenGL: %s\n", version);

    glewExperimental = true;
    GLenum ret = glewInit();
    if (ret != GLEW_OK) {
        fprintf(stderr, "failed to initialize GLEW with error: %s\n", glewGetErrorString(ret));
        return EXIT_FAILURE;
    }

    rc = load_shaders();
    if (rc < 0) {
        // NOTE: prints its own error message
        return EXIT_FAILURE;
    }

    const GLfloat quad[] = {
            -1.0f, -1.0f, 0.0f, 0.0f,
            -1.0f,  1.0f, 1.0f, 0.0f,
            1.0f, -1.0f, 0.0f, 1.0f,
            1.0f,  1.0f, 1.0f, 1.0f,
    };

    GLuint quadVAO;
    glGenVertexArrays(1, &quadVAO);
    glBindVertexArray(quadVAO);

    GLuint quadVertexBuffer;
    glGenBuffers(1, &quadVertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, quadVertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad[0]) * 16, quad, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(quad[0]) * 4,
            (const void*) 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(quad[0]) * 4,
            (const void*) (sizeof(quad[0]) * 2));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    GLuint frame;
    GLubyte black[3] = {0};
    glGenTextures(1, &frame);
    glBindTexture(GL_TEXTURE_2D, frame);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGB8, CMUCAM_IMAGE_HEIGHT, CMUCAM_IMAGE_WIDTH);
    glClearTexSubImage(frame, 0, 0, 0, 0, CMUCAM_IMAGE_HEIGHT, CMUCAM_IMAGE_WIDTH, 1, GL_RGB, GL_UNSIGNED_BYTE, black);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForOpenGL(window, context);
    ImGui_ImplOpenGL3_Init("#version 430 core");

    glClearColor(0, 0, 0, 1);
    glViewport(0, 0, width, height);

    // Enable extended data
    rc = cmucam_set_line_mode(cmucam, true);
    if (rc < 0) {
        fprintf(stderr, "failed to enable cmucam line mode\n");
        return rc;
    }

    // Start CMUcam frame dump
    rc = cmucam_dump_frame(cmucam);
    if (rc < 0) {
        fprintf(stderr, "failed to start frame dump from CMUcam\n");
        return rc;
    }

    GLuint column = 0;
    bool quit = false;
    bool drag_finished = false;
    int drag_start_x = 0;
    int drag_start_y = 0;
    int drag_stop_x = 0;
    int drag_stop_y = 0;

    // App
    auto camera_state = CMUcamState::Frame;

    // CMUcam Color Parameters
    int color_mode = 0;
    bool auto_white_balance = false;
    bool auto_exposure = true;
    bool update_color_mode = false;
    bool update_auto_exposure = false;

    // CMUcam Tracking Parameters
    bool noise_filter = true;
    float tracking_color_min[3] = { 0.f, 0.f, 0.f };
    float tracking_color_max[3] = { 1.f, 1.f, 1.f };
    bool update_noise_filter = false;
    bool update_tracking_colors = false;
    bool track = false;

    // CMUcam Mean Parameters
    float mean_color[3] = { 0.f, 0.f, 0.f };
    float mean_deviation[3] = { 0.f, 0.f, 0.f };
    bool mean = false;

    // CMUcam Dump Frame Parameters
    bool dump_frame_continuous = true;
    bool dump_frame = true;

    // Main loop
    while (1) {
        uint8_t extended[1024];
        cmucam_packet packet;
        packet.extended.data = extended;
        packet.extended.capacity = sizeof extended;

        // If the camera is active, receive a packet
        if (camera_state != CMUcamState::Idle) {
            rc = cmucam_read_packet(cmucam, &packet);
            if (rc < 0) {
                return rc;
            }
        }

        // Do something with packet
        switch (packet.type) {
            case CMUCAM_PACKET_TYPE_F_START:
                column = 0;
            case CMUCAM_PACKET_TYPE_F_NEXT:
                // update texture
                //
                // we store the texture rotated so that the column updates from the CMUcam are sent to
                // OpenGL as row updates, which is generally more performant.
                glTextureSubImage2D(frame, 0, 0, column++, CMUCAM_IMAGE_HEIGHT, 1, GL_RGB, GL_UNSIGNED_BYTE, extended);
                break;
            case CMUCAM_PACKET_TYPE_F_END:
                // dump frame complete, return to idle state
                if (camera_state == CMUcamState::Frame) {
                    camera_state = CMUcamState::Idle;
                }
                break;
            default:
                printf("unimplemented packet type - %d", packet.type);
                break;
        }

        // Update stuff when camera is idle
        if (camera_state == CMUcamState::Idle) {
            if (quit) {
                break;
            }

            // update color mode or auto white balance settings if needed
            if (update_color_mode) {
                rc = cmucam_set_color_mode(cmucam, color_mode ? true : false, auto_white_balance);
                if (rc < 0) {
                    fprintf(stderr, "failed to set cmucam color mode\n");
                    return rc;
                }
                update_color_mode = false;
            }

            // update auto exposure settings if needed
            if (update_auto_exposure) {
                rc = cmucam_set_auto_exposure(cmucam, auto_exposure);
                if (rc < 0) {
                    fprintf(stderr, "failed to set cmucam auto exposure mode\n");
                    return rc;
                }
                update_auto_exposure = false;
            }

            // update noise filter settings if needed
            if (update_noise_filter) {
                rc = cmucam_set_noise_filter(cmucam, noise_filter);
                if (rc < 0) {
                    fprintf(stderr, "failed to set cmucam noise filter mode\n");
                    return rc;
                }
                update_noise_filter = false;
            }

            // update tracking color settings if needed
            if (update_tracking_colors) {
                update_tracking_colors = false;
            }

            // restart frame dump if requested
            if (dump_frame_continuous) {
                rc = cmucam_dump_frame(cmucam);
                if (rc < 0) {
                    return rc;
                }
                camera_state = CMUcamState::Frame;
            }
        }

        // Process window events before we start making OpenGL calls
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                quit = true;
            } else if (event.type == SDL_WINDOWEVENT) {
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    // update viewport with new window size
                    SDL_GetWindowSize(window, &width, &height);
                    glViewport(0, 0, width, height);
                }
            } else if (event.type == SDL_MOUSEBUTTONDOWN && !io.WantCaptureMouse) {
                // scale click into CMUcam coordinates
                float x = event.button.x;
                float y = event.button.y;
                x = (x < 0) ? 0 : (x >= width) ? width - 1 : x;
                y = (y < 0) ? 0 : (y >= height) ? height - 1 : y;
                x *= ((float) CMUCAM_IMAGE_WIDTH) / ((float) width);
                y *= ((float) CMUCAM_IMAGE_HEIGHT) / ((float) height);
                drag_start_x = (x < 1.f) ? 1.f : (x > CMUCAM_IMAGE_WIDTH) ? CMUCAM_IMAGE_WIDTH : x;
                drag_start_y = (y < 1.f) ? 1.f : (y > CMUCAM_IMAGE_HEIGHT) ? CMUCAM_IMAGE_HEIGHT : y;
            } else if (event.type == SDL_MOUSEBUTTONUP && !io.WantCaptureMouse) {
                // scale click into CMUcam coordinates
                float x = event.button.x;
                float y = event.button.y;
                x = (x < 0) ? 0 : (x >= width) ? width - 1 : x;
                y = (y < 0) ? 0 : (y >= height) ? height - 1 : y;
                x *= ((float) CMUCAM_IMAGE_WIDTH) / ((float) width);
                y *= ((float) CMUCAM_IMAGE_HEIGHT) / ((float) height);
                drag_stop_x = (x < 1.f) ? 1.f : (x > CMUCAM_IMAGE_WIDTH) ? CMUCAM_IMAGE_WIDTH : x;
                drag_stop_y = (y < 1.f) ? 1.f : (y > CMUCAM_IMAGE_HEIGHT) ? CMUCAM_IMAGE_HEIGHT : y;
                drag_finished = true;
            }
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // Render the UI
        ImGui::Begin("CMUcam Controls");
        if (ImGui::CollapsingHeader("Options", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Color Mode");
            ImGui::SameLine();
            update_color_mode |= ImGui::RadioButton("RGB", &color_mode, 0);
            ImGui::SameLine();
            update_color_mode |= ImGui::RadioButton("YUV", &color_mode, 1);
            update_color_mode |= ImGui::Checkbox("Auto White Balance", &auto_white_balance);
            update_auto_exposure |= ImGui::Checkbox("Auto Exposure", &auto_exposure);
        }
        if (ImGui::CollapsingHeader("Tracking", ImGuiTreeNodeFlags_DefaultOpen)) {
            update_noise_filter |= ImGui::Checkbox("Noise Filtering", &noise_filter);
            update_tracking_colors |= ImGui::ColorEdit3("Minimum", tracking_color_min,
                    ImGuiColorEditFlags_DisplayRGB);
            update_tracking_colors |= ImGui::ColorEdit3("Maximum", tracking_color_max,
                    ImGuiColorEditFlags_DisplayRGB);
            track = ImGui::Button("Track");
        }
        if (ImGui::CollapsingHeader("Mean", ImGuiTreeNodeFlags_DefaultOpen)) {
            mean = ImGui::Button("Get Mean");
        }
        if (ImGui::CollapsingHeader("Frame Dump", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Continuous", &dump_frame_continuous);
            if (camera_state == CMUcamState::Frame) {
                ImGui::Text("Dumping Column: %d", column);
            } else {
                dump_frame = ImGui::Button("Dump");
            }
        }
        ImGui::End();
        ImGui::Render();

        // draw camera data texture
        glClear(GL_COLOR_BUFFER_BIT);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, frame);
        glUseProgram(quad_shader_prog);
        glUniform1i(0, 0);
        glBindVertexArray(quadVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);

        // draw gui, swap buffers
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    cmucam_close(cmucam);
    return EXIT_SUCCESS;
}
