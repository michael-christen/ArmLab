#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "image_u32.h"

#define ALIGNMENT 4

image_u32_t *image_u32_create(int width, int height)
{
    image_u32_t *im = (image_u32_t*) calloc(1, sizeof(image_u32_t));

    im->width  = width;
    im->height = height;
    im->stride = width;

    if ((im->stride % ALIGNMENT) != 0)
        im->stride += ALIGNMENT - (im->stride % ALIGNMENT);

    im->buf = (uint32_t*) calloc(1, im->height*im->stride*sizeof(uint32_t));

    return im;
}

void image_u32_destroy(image_u32_t *im)
{
    free(im->buf);
    free(im);
}

////////////////////////////////////////////////////////////
// PNM file i/o

// Create an RGBA image from PNM
// TODO Refactor this to load u32 and convert to u32 using existing function
image_u32_t *image_u32_create_from_pnm(const char *path)
{
    int width, height, format;
    image_u32_t *im = NULL;
    uint8_t *buf = NULL;

    FILE *f = fopen(path, "rb");
    if (f == NULL)
        return NULL;

    if (3 != fscanf(f, "P%d\n%d %d\n255", &format, &width, &height))
        goto error;

    // Format 5 == Binary Gray
    // Format 6 == Binary RGB
    im = image_u32_create(width, height);

    // Dump one character, new line after 255
    int res = fread(im->buf, 1, 1, f);
    if (res != 1)
        goto error;

    int sz = im->width*im->height;
    if (format == 6) {
        sz *= 3;
    }

    buf = calloc(1, sz);
    if (sz != fread(buf, 1, sz, f))
        goto error;


    // Copy data from buf to the image buffer. Don't forget to compensate for
    // alignment
    for (int y = 0; y < im->height; y++) {
        for (int x = 0; x < im->width; x++) {
            uint32_t abgr = 0;
            if (format == 5) {
                uint8_t gray = buf[y*im->width + x];
                abgr = gray | (gray << 8) | (gray << 16) | (0xff << 24);
            } else if (format == 6) {
                uint8_t a = 0xff;
                uint8_t r = buf[y*im->width*3 + 3*x];
                uint8_t g = buf[y*im->width*3 + 3*x+1];
                uint8_t b = buf[y*im->width*3 + 3*x+2];
                abgr = (a & 0xff) << 24 | (b & 0xff) << 16 | (g & 0xff) << 8 | r;
            }
            im->buf[y*im->stride + x] = abgr;
        }
    }

    free(buf);
    fclose(f);
    return im;

error:
    fclose(f);
    printf("Failed to read image %s\n",path);
    if (im != NULL)
        image_u32_destroy(im);

    if (buf != NULL)
        free(buf);

    return NULL;
}

int image_u32_write_pnm(const image_u32_t *im, const char *path)
{
    FILE *f = fopen(path, "wb");
    int res = 0;

    if (f == NULL) {
        res = -1;
        goto finish;
    }

    // Only outputs to RGB
    fprintf(f, "P6\n%d %d\n255\n", im->width, im->height);

    for (int y = 0; y < im->height; y++) {
        for (int x = 0; x < im->width; x++) {
            uint32_t abgr = im->buf[y*im->stride + x];
            uint8_t r = (uint8_t)((abgr >> 0) & 0xff);
            uint8_t g = (uint8_t)((abgr >> 8) & 0xff);
            uint8_t b = (uint8_t)((abgr >> 16) & 0xff);

            fwrite(&r, 1, 1, f);
            fwrite(&g, 1, 1, f);
            fwrite(&b, 1, 1, f);
        }
    }
finish:
    if (f != NULL)
        fclose(f);

    return res;
}

////////////////////////////////////////////////////////////
// Conversion

