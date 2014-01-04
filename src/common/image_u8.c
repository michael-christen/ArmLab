#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "image_u8.h"

#define ALIGNMENT 16

image_u8_t *image_u8_create(int width, int height)
{
    image_u8_t *im = (image_u8_t*) calloc(1, sizeof(image_u8_t));

    im->width  = width;
    im->height = height;
    im->stride = width;

    if ((im->stride % ALIGNMENT) != 0)
        im->stride += ALIGNMENT - (im->stride % ALIGNMENT);

    im->buf = (uint8_t*) calloc(1, im->height*im->stride);

    return im;
}

void image_u8_destroy(image_u8_t *im)
{
    free(im->buf);
    free(im);
}

////////////////////////////////////////////////////////////
// PNM file i/o

// Create a grayscale image from PNM.
// TODO Refactor this to load u32 and convert to u8 using existing function
image_u8_t *image_u8_create_from_pnm(const char *path)
{
    int width, height, format;
    image_u8_t *im = NULL;
    uint8_t *buf = NULL;

    FILE *f = fopen(path, "rb");
    if (f == NULL)
        return NULL;

    if (3 != fscanf(f, "P%d\n%d %d\n255", &format, &width, &height))
        goto error;

    // Format 5 == Binary Gray
    // Format 6 == Binary RGB
    im = image_u8_create(width, height);

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

    // Copy data from buf to the image buffer. Convert to gray if necessary
    // Gray conversion for RGB is gray = (r + g + g + b)/4
    for (int y = 0; y < im->height; y++) {
        for (int x = 0; x < im->width; x++) {
            uint8_t gray = 0;
            if (format == 5) {
                gray = buf[y*im->width + x];
            } else if (format == 6) {
                gray = (buf[y*im->width*3 + x] +      // r
                        buf[y*im->width*3 + x+1] +    // g
                        buf[y*im->width*3 + x+1] +    // g
                        buf[y*im->width*3 + x+2]) /   // b
                        4;
            }
            im->buf[y*im->stride + x] = gray;
        }
    }

    free(buf);
    fclose(f);
    return im;

error:
    fclose(f);
    printf("Failed to read image %s\n",path);
    if (im != NULL)
        image_u8_destroy(im);

    if (buf != NULL)
        free(buf);

    return NULL;
}

int image_u8_write_pnm(const image_u8_t *im, const char *path)
{
    FILE *f = fopen(path, "wb");
    int res = 0;

    if (f == NULL) {
        res = -1;
        goto finish;
    }

    // Only outputs to grayscale
    fprintf(f, "P5\n%d %d\n255\n", im->width, im->height);

    for (int y = 0; y < im->height; y++) {
        if (im->width != fwrite(&im->buf[y*im->stride], 1, im->width, f)) {
            res = -2;
            goto finish;
        }
    }
finish:
    if (f != NULL)
        fclose(f);

    return res;
}

////////////////////////////////////////////////////////////
// Conversion

image_u8_t *image_u8_convert(const image_u8_t *in, int desired_bpp)
{
    /*
    if (in == NULL)
        return NULL;

    if (in->bpp == desired_bpp) {
        image_u8_t *out = image_u8_create(in->width, in->height, in->bpp);
        memcpy(out->buf, in->buf, in->stride*in->height);
        return out;
    }

    if (in->bpp == 1)
    {
        if (desired_bpp == 3) {

            image_u8_t *out = image_u8_create(in->width, in->height, desired_bpp);

            for (int y = 0; y < in->height; y++) {
                for (int x = 0; x < in->width; x++) {

                    uint8_t gray = in->buf[y*in->stride + x];

                    out->buf[y*out->stride + x*out->bpp + 0] = gray;
                    out->buf[y*out->stride + x*out->bpp + 1] = gray;
                    out->buf[y*out->stride + x*out->bpp + 2] = gray;
                }
            }

            return out;
        }

        if (desired_bpp == 4) {

            image_u8_t *out = image_u8_create(in->width, in->height, desired_bpp);

            for (int y = 0; y < in->height; y++) {
                for (int x = 0; x < in->width; x++) {

                    uint8_t gray = in->buf[y*in->stride + x*in->bpp];

                    out->buf[y*out->stride + x*out->bpp + 0] = gray;
                    out->buf[y*out->stride + x*out->bpp + 1] = gray;
                    out->buf[y*out->stride + x*out->bpp + 2] = gray;
                    out->buf[y*out->stride + x*out->bpp + 3] = 0xFF;
                }
            }

            return out;
        }
    }

    if (in->bpp == 3)
    {
        if (desired_bpp == 1) {

            image_u8_t *out = image_u8_create(in->width, in->height, desired_bpp);

            for (int y = 0; y < in->height; y++) {
                for (int x = 0; x < in->width; x++) {

                    int r = in->buf[y*in->stride + x*in->bpp + 0];
                    int g = in->buf[y*in->stride + x*in->bpp + 1];
                    int b = in->buf[y*in->stride + x*in->bpp + 2];

                    // XXX better intensity conversion?
                    out->buf[y*out->stride + x] = (r + g + g + b) >> 2;
                }
            }

            return out;
        }

        if (desired_bpp == 4) {

            image_u8_t *out = image_u8_create(in->width, in->height, desired_bpp);

            for (int y = 0; y < in->height; y++) {
                for (int x = 0; x < in->width; x++) {

                    out->buf[y*out->stride + x*out->bpp + 0] = in->buf[y*in->stride + x*in->bpp + 0];
                    out->buf[y*out->stride + x*out->bpp + 1] = in->buf[y*in->stride + x*in->bpp + 1];
                    out->buf[y*out->stride + x*out->bpp + 2] = in->buf[y*in->stride + x*in->bpp + 2];
                    out->buf[y*out->stride + x*out->bpp + 3] = 0xFF;
                }
            }

            return out;
        }
    }

    if (in->bpp == 4)
    {
        if (desired_bpp == 1) {

            image_u8_t *out = image_u8_create(in->width, in->height, desired_bpp);

            for (int y = 0; y < in->height; y++) {
                for (int x = 0; x < in->width; x++) {

                    int r = in->buf[y*in->stride + x*in->bpp + 0];
                    int g = in->buf[y*in->stride + x*in->bpp + 1];
                    int b = in->buf[y*in->stride + x*in->bpp + 2];

                    // XXX better intensity conversion?
                    out->buf[y*out->stride + x] = (r + g + g + b) >> 2;
                }
            }

            return out;
        }

        if (desired_bpp == 3) {

            image_u8_t *out = image_u8_create(in->width, in->height, desired_bpp);

            for (int y = 0; y < in->height; y++) {
                for (int x = 0; x < in->width; x++) {

                    out->buf[y*out->stride + x*out->bpp + 0] = in->buf[y*in->stride + x*in->bpp + 0];
                    out->buf[y*out->stride + x*out->bpp + 1] = in->buf[y*in->stride + x*in->bpp + 1];
                    out->buf[y*out->stride + x*out->bpp + 2] = in->buf[y*in->stride + x*in->bpp + 2];
                }
            }

            return out;
        }
    }

    fprintf(stderr, "[image_u8_convert] Conversion of image from bpp %d to %d unsupported\n",
            in->bpp, desired_bpp);

    */
    return NULL;
}

