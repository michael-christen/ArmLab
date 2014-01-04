#include "image_convert.h"

#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include "common/image_u32.h"

image_u32_t *debayer_gbrg(frame_data_t *frmd)
{
    image_u32_t *im = image_u32_create(frmd->ifmt->width,
                                       frmd->ifmt->height);

    uint8_t *d = (uint8_t*)(frmd->data);
    uint32_t *out = im->buf;
    int width = im->width;
    int height = im->height;
    int stride = im->stride;
    int a = 0xff;
    // Loop over each 2x2 bayer block and compute the pixel values for
    // each element
    for (int y = 0; y < height; y+=2) {
        for (int x = 0; x < width; x+=2) {
            int r = 0, g = 0, b = 0;

            // compute indices into bayer pattern for the nine 2x2 blocks we'll use.
            int X00 = (y-2)*width+(x-2);
            int X01 = (y-2)*width+(x+0);
            int X02 = (y-2)*width+(x+2);
            int X10 = (y+0)*width+(x-2);
            int X11 = (y+0)*width+(x+0);
            int X12 = (y+0)*width+(x+2);
            int X20 = (y+2)*width+(x-2);
            int X21 = (y+2)*width+(x+0);
            int X22 = (y+2)*width+(x+2);

            // handle the edges of the screen.
            if (y < 2) {
                X00 += 2*width;
                X01 += 2*width;
                X02 += 2*width;
            }
            if (y+2 >= height) {
                X20 -= 2*width;
                X21 -= 2*width;
                X22 -= 2*width;
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
            r = ((d[X01+width]&0xff) + (d[X11+width]&0xff)) / 2;
            g = d[X11]&0xff;
            b = ((d[X10+1]&0xff) + (d[X11+1]&0xff)) / 2;;
            out[y*stride+x] = r+(g<<8)+(b<<16)+(a<<24);

            // top right pixel (B)
            r = ((d[X01+width]&0xff)+(d[X02+width]&0xff)+(d[X01+width]&0xff) + (d[X12+width]&0xff)) / 4;
            g = ((d[X01+width+1]&0xff)+(d[X11]&0xff)+(d[X12]&0xff)+(d[X11+width+1]&0xff)) / 4;
            b = (d[X11+1]&0xff);
            out[y*stride+x+1] = r+(g<<8)+(b<<16)+(a<<24);

            // bottom left pixel (R)
            r = (d[X11+width]&0xff);
            g = ((d[X11]&0xff)+(d[X10+width+1]&0xff)+(d[X11+width+1]&0xff)+(d[X21]&0xff)) / 4;
            b = ((d[X10+1]&0xff)+(d[X11+1]&0xff)+(d[X20+1]&0xff)+(d[X21+1]&0xff)) / 4;
            out[y*stride+stride+x] = r+(g<<8)+(b<<16)+(a<<24);

            // bottom right pixel (G)
            r = ((d[X11+width]&0xff)+(d[X12+width]&0xff)) / 2;
            g = (d[X11+width+1]&0xff);
            b = ((d[X11+1]&0xff)+(d[X21+1]&0xff)) / 2;
            out[y*stride+stride+x+1] = r+(g<<8)+(b<<16)+(a<<24);
        }
    }

    return im;
}

image_u32_t *convert_to_image(frame_data_t *frmd)
{
    if (!strcmp("BAYER_GBRG", frmd->ifmt->format)) {
        return debayer_gbrg(frmd);
    } else if (!strcmp("BAYER_RGGB", frmd->ifmt->format)) {
        printf("format not yet implemented\n");
    } else if (!strcmp("BAYER_GRBG", frmd->ifmt->format)) {
        printf("format not yet implemented\n");
    } else if (!strcmp("GRAY16", frmd->ifmt->format)) {
        printf("format not yet implemented\n");
    } else if (!strcmp("GRAY8", frmd->ifmt->format)) {
        image_u32_t *im = image_u32_create(frmd->ifmt->width,
                                           frmd->ifmt->height);
        for (int y = 0; y < im->height; y++) {
            for (int x = 0; x < im->width; x++) {
                im->buf[y*im->stride+x] = ((uint32_t*)(frmd->data))[y*im->width+x];
            }
        }

        return im;
    } else if (!strcmp("YUYV", frmd->ifmt->format)) {
        printf("format not yet implemented\n");
    } else {
        printf("ERR: Format %s not supported\n", frmd->ifmt->format);
    }

    // XXX More formats to come...See jcam.ImageConvert
    return NULL;
}
