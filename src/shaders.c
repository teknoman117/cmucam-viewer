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
        "layout (location = 1) uniform bool transpose;\n"
        "layout (location = 0) out vec4 color;        \n"
        "                                             \n"
        "void main() {                                \n"
        "    vec2 coord = uv;                         \n"
        "    if (transpose) {                         \n"
        "        coord = vec2(uv.y, 1.0 - uv.x);      \n"
        "    }                                        \n"
        "    vec4 color_in = texture(tex, coord);     \n"
        "    color = vec4(color_in.rgb, 1);           \n"
        "}                                            \n";

GLuint compile_shader(const char *source, GLenum type)
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

GLuint link_program(GLuint vertex_shader, GLuint fragment_shader)
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

int load_shaders() {
    quad_shader_vert = compile_shader(quad_shader_vert_source, GL_VERTEX_SHADER);
    if (!quad_shader_vert) {
        return -1;
    }

    quad_shader_frag = compile_shader(quad_shader_frag_source, GL_FRAGMENT_SHADER);
    if (!quad_shader_frag) {
        return -1;
    }

    quad_shader_prog = link_program(quad_shader_vert, quad_shader_frag);
    if (!quad_shader_prog) {
        return -1;
    }

    return 0;
}