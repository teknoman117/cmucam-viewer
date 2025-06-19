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

#ifndef CMUCAM_VIEWER_CMUCAM_H_
#define CMUCAM_VIEWER_CMUCAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdbool.h>

#define CMUCAM_IMAGE_WIDTH 80
#define CMUCAM_IMAGE_HEIGHT 143

#define CMUCAM_PACKET_TYPE_C ((uint8_t) 'C')
#define CMUCAM_PACKET_TYPE_C_SIZE 7
#define CMUCAM_PACKET_TYPE_M ((uint8_t) 'M')
#define CMUCAM_PACKET_TYPE_M_SIZE 9
#define CMUCAM_PACKET_TYPE_S ((uint8_t) 'S')
#define CMUCAM_PACKET_TYPE_S_SIZE 7

int cmucam_initialize();

int cmucam_open(const char* path);
int cmucam_close(int fd);

int cmucam_set_color_mode(int fd, bool yuv, bool auto_white_balance);
int cmucam_set_auto_exposure(int fd, bool on);
int cmucam_set_line_mode(int fd, bool on);
int cmucam_set_noise_filter(int fd, bool on);

int cmucam_dumpframe(int fd);
int cmucam_dumpframe_next_column(int fd, uint8_t* column);

int cmucam_set_window(int fd, int x, int y, int x2, int y2);
int cmucam_reset_window(int fd);
int cmucam_track_window(int fd);

int cmucam_track_color(int fd, uint8_t rmin, uint8_t rmax, uint8_t gmin, uint8_t gmax,
        uint8_t bmin, uint8_t bmax);

int cmucam_get_mean(int fd);

int cmucam_read_packet(int fd, uint8_t *packet, uint8_t n);
int cmucam_end_stream(int fd);

#ifdef __cplusplus
}
#endif

#endif /* CMUCAM_VIEWER_CMUCAM_H_ */
