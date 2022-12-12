#ifndef CMUCAM_VIEWER_CMUCAM_H_
#define CMUCAM_VIEWER_CMUCAM_H_

#include <inttypes.h>

#define CMUCAM_IMAGE_WIDTH 80
#define CMUCAM_IMAGE_HEIGHT 143

int cmucam_open(const char* path);
int cmucam_close(int fd);

int cmucam_dump_frame(int fd);
int cmucam_dump_frame_next_column(int fd, uint8_t* column);

#endif /* CMUCAM_VIEWER_CMUCAM_H_ */
