#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "image_u8x3.h"

#define DEFAULT_ALIGNMENT 72

image_u8x3_t *image_u8x3_create(int width, int height)
{
    return image_u8x3_create_alignment(width, height, DEFAULT_ALIGNMENT);
}

// force stride to be a multiple of 'alignment' bytes.
image_u8x3_t *image_u8x3_create_alignment(int width, int height, int alignment)
{
    assert(alignment > 0);

    image_u8x3_t *im = (image_u8x3_t*) calloc(1, sizeof(image_u8x3_t));

    im->width  = width;
    im->height = height;
    im->stride = width*3;

    if ((im->stride % alignment) != 0)
        im->stride += alignment - (im->stride % alignment);

    im->buf = (uint8_t*) calloc(1, im->height*im->stride);

    return im;
}

void image_u8x3_destroy(image_u8x3_t *im)
{
    if (!im)
        return;

    free(im->buf);
    free(im);
}

