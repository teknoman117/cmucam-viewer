#ifndef CMUCAM_VIEWER_CMUCAM_H_
#define CMUCAM_VIEWER_CMUCAM_H_

#include <inttypes.h>

#define CMUCAM_IMAGE_WIDTH 80
#define CMUCAM_IMAGE_HEIGHT 143

int cmucam_initialize();

int cmucam_open(const char* path);
int cmucam_close(int fd);

int cmucam_dumpframe(int fd);
int cmucam_dumpframe_next_column(int fd, uint8_t* column);

#endif /* CMUCAM_VIEWER_CMUCAM_H_ */
