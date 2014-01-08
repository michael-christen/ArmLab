#include "image_convert.h"

#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include "common/image_u32.h"

static int clamp(int v)
{
    if (v < 0)
        return 0;
    if (v > 255)
        return 255;
    return v;
}

static image_u32_t *convert_rgb24(image_source_data_t *frmd)
{
    image_u32_t *im = image_u32_create(frmd->ifmt.width,
                                       frmd->ifmt.height);

    int width = im->width;
    int height = im->height;
    int stride = im->stride;
    uint32_t a = 0xff << 24;

    uint8_t *rgb = (uint8_t*)(frmd->data);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = (y*width+x)*3;
            int r = rgb[idx+0];
            int g = rgb[idx+1];
            int b = rgb[idx+2];

            im->buf[y*stride+x] = r | (g<<8) | (b<<16) | a;
        }
    }

    return im;
}

static image_u32_t *convert_yuyv(image_source_data_t *frmd)
{
    image_u32_t *im = image_u32_create(frmd->ifmt.width,
                                       frmd->ifmt.height);

    int width = im->width;
    int height = im->height;
    int stride = im->stride;
    uint32_t a = 0xff << 24;

    int sstride = width*2;
    uint8_t *yuyv = (uint8_t*)(frmd->data);

    for (int i = 0; i < height; i++) {

        for (int j = 0; j < width / 2; j++) {
            int y1 = yuyv[i*sstride + 4*j+0]&0xff;
            int u  = yuyv[i*sstride + 4*j+1]&0xff;
            int y2 = yuyv[i*sstride + 4*j+2]&0xff;
            int v  = yuyv[i*sstride + 4*j+3]&0xff;

            int cb = ((u-128) * 454)>>8;
            int cr = ((v-128) * 359)>>8;
            int cg = ((v-128) * 183 + (u-128) * 88)>>8;
            int r, g, b;

            r = clamp(y1 + cr);
            b = clamp(y1 + cb);
            g = clamp(y1 - cg);
            im->buf[i*stride + 2*j+0] = (r) | (g<<8) | (b<<16) | a;

            r = clamp(y2 + cr);
            b = clamp(y2 + cb);
            g = clamp(y2 - cg);
            im->buf[i*stride + 2*j+1] = (r) | (g<<8) | (b<<16) | a;
        }
    }
    return im;
}

static image_u32_t *debayer_rggb(image_source_data_t *frmd)
{
    image_u32_t *im = image_u32_create(frmd->ifmt.width,
                                       frmd->ifmt.height);

    uint8_t *d = (uint8_t*)(frmd->data);
    uint32_t *out = im->buf;
    int width = im->width;
    int height = im->height;
    int stride = im->stride;
    uint32_t a = 0xff << 24;

    // loop over each 2x2 bayer block and compute the pixel values for each element.
    for (int y = 0; y < height; y+=2) {
        for (int x = 0; x < width; x+=2) {

            int r = 0, g = 0, b = 0;

            // compute indices into bayer pattern for the nine 2x2 blocks we'll use.
            int X00 = (y-2)*stride+(x-2);
            int X01 = (y-2)*stride+(x+0);
            int X02 = (y-2)*stride+(x+2);
            int X10 = (y+0)*stride+(x-2);
            int X11 = (y+0)*stride+(x+0);
            int X12 = (y+0)*stride+(x+2);
            int X20 = (y+2)*stride+(x-2);
            int X21 = (y+2)*stride+(x+0);
            int X22 = (y+2)*stride+(x+2);

            // handle the edges of the screen.
            if (y < 2) {
                X00 += 2*stride;
                X01 += 2*stride;
                X02 += 2*stride;
            }
            if (y+2 >= height) {
                X20 -= 2*stride;
                X21 -= 2*stride;
                X22 -= 2*stride;
            }
            if (x < 2) {
                X00 += 2;
                X10 += 2;
                X20 += 2;
            }
            if (x+2 >= width) {
                X02 -= 2;
                X12 -= 2;
                X22 -= 2;
            }

            // top left pixel (R)
            r = (d[X11]&0xff);
            g = ((d[X01+stride]&0xff)+(d[X10+1]&0xff)+(d[X11+1]&0xff)+(d[X11+stride]&0xff)) / 4;
            b = ((d[X00+stride+1]&0xff)+(d[X10+stride+1]&0xff)+(d[X10+stride+1]&0xff)+(d[X11+stride+1]&0xff)) / 4;
            out[y*stride+x] = (r)+(g<<8)+(b<<16)+a;

            // top right pixel (G)
            r = ((d[X11]&0xff)+(d[X12]&0xff)) / 2;
            g = (d[X11+1]&0xff);
            b = ((d[X01+stride+1]&0xff)+(d[X11+stride+1]&0xff)) / 2;
            out[y*stride+x+1] = (r)+(g<<8)+(b<<16)+a;

            // bottom left pixel (G)
            r = ((d[X11]&0xff)+(d[X21]&0xff)) / 2;
            g = (d[X11+stride]&0xff);
            b = ((d[X10+stride+1]&0xff)+(d[X11+stride+1]&0xff)) / 2;
            out[y*stride+stride+x] = (r)+(g<<8)+(b<<16)+a;

            // bottom right pixel (B)
            r = ((d[X11]&0xff)+(d[X12]&0xff)+(d[X21]&0xff)+(d[X22]&0xff)) / 4;
            g = ((d[X11+1]&0xff)+(d[X11+stride]&0xff)+(d[X12+stride]&0xff)+(d[X21+1]&0xff))/ 4;
            b = (d[X11+stride+1]&0xff);
            out[y*stride+stride+x+1] = (r)+(g<<8)+(b<<16)+a;
        }
    }

    return im;
}

static image_u32_t *debayer_gbrg(image_source_data_t *frmd)
{
    image_u32_t *im = image_u32_create(frmd->ifmt.width,
                                       frmd->ifmt.height);

    uint8_t *d = (uint8_t*)(frmd->data);
    uint32_t *out = im->buf;
    int width = im->width;
    int height = im->height;
    int stride = im->stride;
    uint32_t a = 0xff << 24;

    // Loop over each 2x2 bayer block and compute the pixel values for
    // each element
    for (int y = 0; y < height; y+=2) {
        for (int x = 0; x < width; x+=2) {
            int r = 0, g = 0, b = 0;

            // compute indices into bayer pattern for the nine 2x2 blocks we'll use.
            int X00 = (y-2)*stride+(x-2);
            int X01 = (y-2)*stride+(x+0);
            int X02 = (y-2)*stride+(x+2);
            int X10 = (y+0)*stride+(x-2);
            int X11 = (y+0)*stride+(x+0);
            int X12 = (y+0)*stride+(x+2);
            int X20 = (y+2)*stride+(x-2);
            int X21 = (y+2)*stride+(x+0);
            int X22 = (y+2)*stride+(x+2);

            // handle the edges of the screen.
            if (y < 2) {
                X00 += 2*stride;
                X01 += 2*stride;
                X02 += 2*stride;
            }
            if (y+2 >= height) {
                X20 -= 2*stride;
                X21 -= 2*stride;
                X22 -= 2*stride;
            }
            if (x < 2) {
                X00 += 2;
                X10 += 2;
                X20 += 2;
            }
            if (x+2 >= width) {
                X02 -= 2;
                X12 -= 2;
                X22 -= 2;
            }

            // top left pixel (G)
            r = ((d[X01+stride]&0xff) + (d[X11+stride]&0xff)) / 2;
            g = d[X11]&0xff;
            b = ((d[X10+1]&0xff) + (d[X11+1]&0xff)) / 2;;
            out[y*stride+x] = r+(g<<8)+(b<<16)+a;

            // top right pixel (B)
            r = ((d[X01+stride]&0xff)+(d[X02+stride]&0xff)+(d[X01+stride]&0xff) + (d[X12+stride]&0xff)) / 4;
            g = ((d[X01+stride+1]&0xff)+(d[X11]&0xff)+(d[X12]&0xff)+(d[X11+stride+1]&0xff)) / 4;
            b = (d[X11+1]&0xff);
            out[y*stride+x+1] = r+(g<<8)+(b<<16)+a;

            // bottom left pixel (R)
            r = (d[X11+stride]&0xff);
            g = ((d[X11]&0xff)+(d[X10+stride+1]&0xff)+(d[X11+stride+1]&0xff)+(d[X21]&0xff)) / 4;
            b = ((d[X10+1]&0xff)+(d[X11+1]&0xff)+(d[X20+1]&0xff)+(d[X21+1]&0xff)) / 4;
            out[y*stride+stride+x] = r+(g<<8)+(b<<16)+a;

            // bottom right pixel (G)
            r = ((d[X11+stride]&0xff)+(d[X12+stride]&0xff)) / 2;
            g = (d[X11+stride+1]&0xff);
            b = ((d[X11+1]&0xff)+(d[X21+1]&0xff)) / 2;
            out[y*stride+stride+x+1] = r+(g<<8)+(b<<16)+a;
        }
    }

    return im;
}

static image_u32_t *gray8(image_source_data_t *frmd)
{
    image_u32_t *im = image_u32_create(frmd->ifmt.width,
                                       frmd->ifmt.height);
    uint8_t *buf = (uint8_t*)(frmd->data);
    for (int y = 0; y < im->height; y++) {
        for (int x = 0; x < im->width; x++) {
            int idx = y*im->width + x;
            int gray = buf[idx] & 0xff;
            im->buf[y*im->stride+x] = (0xff000000) | gray << 16 | gray << 8 | gray;
        }
    }

    return im;
}

image_u32_t *image_convert_u32(image_source_data_t *frmd)
{
    if (!strcmp("BAYER_GBRG", frmd->ifmt.format)) {
        return debayer_gbrg(frmd);
    } else if (!strcmp("GRAY8", frmd->ifmt.format)) {
        return gray8(frmd);

    } else if (!strcmp("BAYER_RGGB", frmd->ifmt.format)) {
        return debayer_rggb(frmd);

//    } else if (!strcmp("BAYER_GRBG", frmd->ifmt.format)) {

//    } else if (!strcmp("GRAY16", frmd->ifmt->format)) {
    } else if (!strcmp("RGB", frmd->ifmt.format)) {
        return convert_rgb24(frmd);

    } else if (!strcmp("YUYV", frmd->ifmt.format)) {
        return convert_yuyv(frmd);
    } else {
        printf("ERR: Format %s not supported\n", frmd->ifmt.format);
    }

    // XXX More formats to come...See jcam.ImageConvert
    return NULL;
}
