#ifndef _IMAGE_U32_H
#define _IMAGE_U32_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
    int stride;
#endif

typedef struct image_u32 image_u32_t;
struct image_u32
{
    int width, height;
    int stride;

    uint32_t *buf;
};

/////////////////////////////////////
// Designed to hold ABGR format images.
/////////////////////////////////////

// Create or load an image. returns NULL on failure
image_u32_t *image_u32_create(int width, int height);
image_u32_t *image_u32_create_from_pnm(const char *path);

void image_u32_destroy(image_u32_t *im);

// Write a pnm. Returns 0 on success
// Currently only supports GRAY and ABGR. Does not write out alpha for ABGR
int image_u32_write_pnm(const image_u32_t *im, const char *path);

#ifdef __cplusplus
}
#endif

#endif
