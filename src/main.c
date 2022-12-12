#include <GL/glew.h>
#include <GL/gl.h>

#include <SDL.h>

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmucam.h"
#include "shaders.h"

int main(int argc, char** argv) {
    int cmucam = cmucam_open("/dev/ttyUSB0");
    if (cmucam < 0) {
        return EXIT_FAILURE;
    }

    // Setup an OpenGL 4.3 rendering environment
    int rc = SDL_Init(SDL_INIT_VIDEO);
    if (rc < 0) {
        fprintf(stderr, "failed to initialize SDL with error: %s (%d)\n", SDL_GetError(), -rc);
        return EXIT_FAILURE;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    SDL_Window *window = SDL_CreateWindow(
            "CMUCam v1.12 Viewer",
            SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED,
            CMUCAM_IMAGE_WIDTH * 2,
            CMUCAM_IMAGE_HEIGHT,
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

    // Setup OpenGL resources
    const GLubyte *version = glGetString(GL_VERSION);
    printf("OpenGL: %s\n", version);

    glewExperimental = true;
    GLenum ret = glewInit();
    if (ret != GLEW_OK) {
        fprintf(stderr, "failed to initialize GLEW with error: %s\n", glewGetErrorString(ret));
        return EXIT_FAILURE;
    }

    rc = loadShaders();
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

    ret = glGetError();
    if (ret != GL_NO_ERROR) {
        fprintf(stderr, "there is a GL error!!! 0\n");
    }

    // use nearest filtering for integer scaling
    GLuint frame;
    glGenTextures(1, &frame);
    glBindTexture(GL_TEXTURE_2D, frame);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGB8, CMUCAM_IMAGE_HEIGHT, CMUCAM_IMAGE_WIDTH);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    glClearColor(0, 0, 0, 1);
    SDL_GL_SetSwapInterval(1);
    SDL_ShowWindow(window);
    
    // Render CMUcam frame dumps
    GLint column = 0;
    bool quit = false;

    rc = cmucam_dump_frame(cmucam);
    if (rc < 0) {
        return rc;
    }

    while (true) {
        // Fetch the next column from the camera
        uint8_t column_data[CMUCAM_IMAGE_HEIGHT * 3];
        rc = cmucam_dump_frame_next_column(cmucam, column_data);
        if (rc < 0) {
            fprintf(stderr, "failed to get next column from CMUcam\n");
            return EXIT_FAILURE;
        }

        // end-of-frame
        if (rc == 1) {
            // an opportunity to exit cleanly
            if (quit) {
                break;
            }

            rc = cmucam_dump_frame(cmucam);
            if (rc < 0) {
                fprintf(stderr, "failed to start frame dump from CMUcam\n");
                return rc;
            }

            column = 0;
            continue;
        }

        // Process window events before we start making OpenGL calls
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                quit = true;
            } else if (event.type == SDL_WINDOWEVENT) {
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    // update viewport with new window size
                    int width;
                    int height;
                    SDL_GetWindowSize(window, &width, &height);
                    glViewport(0, 0, width, height);
                }
            }
        }

        // update texture
        // we store the texture rotated so that the column updates from the CMUcam are sent to
        // OpenGL as row updates, which is generally more performant.
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, frame);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, column, CMUCAM_IMAGE_HEIGHT, 1, GL_RGB, GL_UNSIGNED_BYTE, column_data);
        column++;

        // redraw window
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(quad_shader_prog);
        glBindVertexArray(quadVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
        SDL_GL_SwapWindow(window);
    }

    cmucam_close(cmucam);
    return EXIT_SUCCESS;
}
