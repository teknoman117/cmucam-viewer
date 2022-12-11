#include <GL/glew.h>
#include <GL/gl.h>

#include <SDL.h>

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "shaders.h"

#define CMUCAM_IMAGE_WIDTH 81
#define CMUCAM_IMAGE_HEIGHT 143

int wait_cmucam(int fd) {
    char byte = '\0';
    do {
        fsync(fd);
        int rc = read(fd, &byte, 1);
        if (rc < 0) {
            fprintf(stderr, "failed to read from CMUcam - %d\n", rc);
            return rc;
        } else if (rc > 0) {
            fprintf(stderr, "got byte: %u\n", byte);
        }
    } while (byte != ':');
    return 0;
}

int reset_cmucam(int fd) {
    const char cmucam_reset_cmd[] = {'R', 'S', 0};
    int rc = write(fd, cmucam_reset_cmd, 3);
    if (rc < 0) {
        fprintf(stderr, "failed to write to CMUcam - %d\n", rc);
        return rc;
    }

    return wait_cmucam(fd);
}

int setup_cmucam(const char* path) {
    // Setup CMUcam
    int fd = open(path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "failed to open CMUcam - %d\n", fd);
        return fd;
    }
    usleep(10000);

    /*struct termios attrs;
    int rc = tcgetattr(fd, &attrs);
    if (rc < 0) {
        fprintf(stderr, "failed to get termios attributes - %d\n", rc);
        return rc;
    }

    // configure 115200 8n1, no flow control or line processing
    attrs.c_cflag &= ~PARENB;
    attrs.c_cflag &= ~CSTOPB;
    attrs.c_cflag &= ~CSIZE;
    attrs.c_cflag |= CS8;
    attrs.c_cflag &= ~CRTSCTS;
    attrs.c_cflag |= CREAD | CLOCAL;

    attrs.c_lflag &= ~ICANON;
    attrs.c_lflag &= ~ECHO;
    attrs.c_lflag &= ~ECHOE;
    attrs.c_lflag &= ~ECHONL;
    attrs.c_lflag &= ~ISIG;

    attrs.c_iflag &= ~(IXON | IXOFF | IXANY);
    attrs.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    attrs.c_oflag &= ~OPOST;
    attrs.c_oflag &= ~ONLCR;

    //attrs.c_cc[VTIME] = 0;
    //attrs.c_cc[VMIN] = 0;

    cfsetispeed(&attrs, B115200);
    cfsetospeed(&attrs, B115200);

    rc = tcsetattr(fd, TCSANOW, &attrs);
    if (rc < 0) {
        fprintf(stderr, "failed to get termios attributes - %d\n", rc);
        return rc;
    }*/

    // Boot the camera (get a prompt)
    /*const char* cmucam_boot_cmd = "\r";
    int rc = write(fd, &cmucam_boot_cmd, 1);
    if (rc < 0 || rc != 1) {
        fprintf(stderr, "failed to write to CMUcam - %d\n", rc);
    }

    rc = wait_cmucam(fd);
    if (rc < 0) {
        return rc;
    }*/
    printf("CMUcam booted\n");

    // Put the camera in raw mode
    const char* cmucam_rawmode_cmd = "RM 7\r";
    int rc = write(fd, cmucam_rawmode_cmd, 5);
    if (rc < 0 || rc != 5) {
        fprintf(stderr, "failed to write to CMUcam - %d\n", rc);
        return rc;
    }

    rc = wait_cmucam(fd);
    if (rc < 0) {
        return rc;
    }
    printf("CMUcam in raw mode\n");

    return fd;
}

int main(int argc, char** argv) {
    int cmucam = setup_cmucam("/dev/ttyUSB0");
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
            -1.0f,  1.0f, 0.0f, 1.0f,
            1.0f, -1.0f, 1.0f, 0.0f,
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
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGB8, CMUCAM_IMAGE_WIDTH, CMUCAM_IMAGE_HEIGHT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    SDL_GL_SetSwapInterval(0);
    glClearColor(0, 0, 0, 1);
    
    // Start CMUcam image dump
    const char cmucam_dumpframe_cmd[] = {'D', 'F', 0};
    rc = write(cmucam, &cmucam_dumpframe_cmd, 3);
    if (rc < 0 || rc != 3) {
        fprintf(stderr, "failed to write to CMUcam - %d\n", rc);
        return EXIT_FAILURE;
    }
    
    // Draw CMUcam image until exit is requested
    SDL_ShowWindow(window);
    GLint column = 0;
    bool quit = false;
    while (!quit) {
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                printf("Exit requested\n");
                quit = true;
            }
        }

        // process incoming frame from CMUcam
        uint8_t b = 0;
        rc = 0;
        do {
            rc = read(cmucam, &b, 1);
            if (rc < 0) {
                fprintf(stderr, "failed to read from CMUcam (1) - %d\n", rc);
                return EXIT_FAILURE;
            }
        } while (rc != 1);

        if (b == 1) {
            // new frame
            column = 0;
            printf("column 0\n");
        } else if (b == 2) {
            column++;
            printf("column %d\n", column);
        } else if (b == 3) {
            // discard
            continue;
        } else if (b == ':') {
            printf("completed frame\n");

            // start next frame
            rc = write(cmucam, &cmucam_dumpframe_cmd, 3);
            if (rc < 0 || rc != 3) {
                fprintf(stderr, "failed to write to CMUcam - %d\n", rc);
                return EXIT_FAILURE;
            }
            continue;
        }

        // read column
        uint8_t column_data[429];
        size_t offset = 0;
        while (offset != sizeof(column_data)) {
            rc = read(cmucam, column_data + offset, sizeof(column_data) - offset);
            if (rc < 0) {
                fprintf(stderr, "failed to read from CMUcam (2) - %d\n", rc);
                return EXIT_FAILURE;
            }
            offset += rc;
        }

        // update texture
        glBindTexture(GL_TEXTURE_2D, frame);
        glTexSubImage2D(GL_TEXTURE_2D, 0, column, 0, 1, CMUCAM_IMAGE_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, column_data);

        // redraw window
        int width;
        int height;
        SDL_GetWindowSize(window, &width, &height);

        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(quad_shader_prog);
        glBindVertexArray(quadVAO);
        glActiveTexture(GL_TEXTURE0);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindVertexArray(0);
        SDL_GL_SwapWindow(window);
    }

    reset_cmucam(cmucam);
    return EXIT_SUCCESS;
}
