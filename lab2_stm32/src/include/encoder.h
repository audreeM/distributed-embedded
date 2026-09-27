#ifndef ENCODER_H_
#define ENCODER_H_
#include <stdint.h>

int encoder_init(void);
int32_t encoder_read(int side);   /* 0 = left, 1 = right. Signed total count. */

#endif
