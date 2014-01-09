#ifndef _IMAGE_U8X3_H
#define _IMAGE_U8X3_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct image_u8x3 image_u8x3_t;
struct image_u8x3
{
    int width, height;
    int stride; // bytes per line

    uint8_t *buf;
};

// Create or load an image. returns NULL on failure
image_u8x3_t *image_u8x3_create(int width, int height);

// force stride to be a multiple of 'alignment' bytes.
image_u8x3_t *image_u8x3_create_alignment(int width, int height, int alignment);

void image_u8x3_destroy(image_u8x3_t *im);

#ifdef __cplusplus
}
#endif

#endif
