#include "image_util.h"

#include <stdlib.h>
#include <assert.h>

image_u8_t * image_util_convert_rgb_to_rgba(image_u8_t * input)
{
/*    image_u8_t * output = calloc(1, sizeof(image_u8_t));
    output->bpp = 4;
    output->width = input->width;
    output->height = input->height;
    output->stride = output->width * output->bpp; // guarantee 4 byte alignment

    //printf("size %d height %d stride %d width %d\n", output->stride*output->height, output->height, output->stride, output->width);
    output->buf = calloc(output->stride*output->height, sizeof(uint8_t));
    assert(input->bpp == 3);

    for (int y = 0; y < input->height; y++)
        for (int x = 0; x < input->width; x++) {
            int in_idx = y*input->stride + x*input->bpp;
            int r = input->buf[in_idx + 0];
            int g = input->buf[in_idx + 1];
            int b = input->buf[in_idx + 2];
            int a = 0xff;

            int out_idx = y*output->stride +x*output->bpp;
            output->buf[out_idx + 0] = r;
            output->buf[out_idx + 1] = g;
            output->buf[out_idx + 2] = b;
            output->buf[out_idx + 3] = a;
        }

    return output;*/
    return input; // XXX

}

