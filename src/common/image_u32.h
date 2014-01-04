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

// Create or load an image. returns NULL on failure
image_u32_t *image_u32_create(int width, int height);
image_u32_t *image_u32_create_from_pnm(const char *path);

void image_u32_destroy(image_u32_t *im);

// Write a pnm. Returns 0 on success
// Currently only supports GRAY and ABGR. Does not write out alpha for ABGR
int image_u32_write_pnm(const image_u32_t *im, const char *path);

// Convert an image to the bpp specified. Currently supports GRAY, RGB, and RGBA.
// ARGB would be mistaken for RGBA. Alpha is ignored when converting from RGBA
// and is set to 0xFF when converting to RGBA. Grayscale conversion uses:
//     i = (r + g + g + b)/4)
// If im->bpp == desired_bpp, a copy is made
//image_u32_t *image_u32_convert(const image_u32_t *im, int desired_bpp);

#ifdef __cplusplus
}
#endif

#endif
