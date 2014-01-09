#ifndef IMAGE_CONVERT_H
#define IMAGE_CONVERT_H

#include "common/image_u32.h"
#include "common/image_u8x3.h"

#include "imagesource/image_source.h"

// Convert frame from imagesource to a useable image_u32_t
// packed like this:
// (a<<24)+(b<<16)+(g<<8)+r.
image_u32_t *image_convert_u32(image_source_data_t *frmd);

// r, g, b, r, g, b,...
image_u8x3_t *image_convert_u8x3(image_source_data_t *isdata);

#endif
