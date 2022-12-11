#include <GL/glew.h>
#include <GL/gl.h>

#include <malloc.h>
#include <stdio.h>

#include "shaders.h"

GLuint quad_shader_vert = 0;
GLuint quad_shader_frag = 0;
GLuint quad_shader_prog = 0;

static const char *quad_shader_vert_source =
        "#version 430 core                       \n"
        "                                        \n"
        "layout (location = 0) in vec2 position; \n"
        "layout (location = 1) in vec2 uv_in;    \n"
        "                                        \n"
        "out vec2 uv;                            \n"
        "                                        \n"
        "void main() {                           \n"
        "    uv = uv_in;                         \n"
        "    gl_Position = vec4(position, 0, 1); \n"
        "}                                       \n";

static const char *quad_shader_frag_source =
        "#version 430 core                            \n"
        "                                             \n"
        "in vec2 uv;                                  \n"
        "layout (location = 0) uniform sampler2D tex; \n"
        "layout (location = 0) out vec4 color;        \n"
        "                                             \n"
        "void main() {                                \n"
        "    vec4 color_in = texture(tex, uv);        \n"
        "    color = vec4(color_in.rgb, 1);           \n"
        "}                                            \n";

GLuint compileShader(const char *source, GLenum type)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);

    int ret = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ret);
    if (!ret) {
        fprintf(stderr, "failed to compile shader\n");
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &ret);
        if (ret > 1) {
            char* log = calloc(ret + 1, 1);
            glGetShaderInfoLog(shader, ret, NULL, log);
            fprintf(stderr, "%s", log);
            free(log);
        }

        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

GLuint linkProgram(GLuint vertex_shader, GLuint fragment_shader)
{
    GLuint program = glCreateProgram();

    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);

    int ret = 0;
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &ret);
    if (!ret) {
        fprintf(stderr, "failed to link program\n");
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &ret);
        if (ret > 1) {
            char* log = calloc(ret + 1, 1);
            glGetProgramInfoLog(program, ret, NULL, log);
            fprintf(stderr, "%s", log);
            free(log);
        }

        glDeleteProgram(program);
        return 0;
    }

    return program;
}

int loadShaders() {
    quad_shader_vert = compileShader(quad_shader_vert_source, GL_VERTEX_SHADER);
    if (!quad_shader_vert) {
        return -1;
    }

    quad_shader_frag = compileShader(quad_shader_frag_source, GL_FRAGMENT_SHADER);
    if (!quad_shader_frag) {
        return -1;
    }

    quad_shader_prog = linkProgram(quad_shader_vert, quad_shader_frag);
    if (!quad_shader_prog) {
        return -1;
    }

    return 0;
}