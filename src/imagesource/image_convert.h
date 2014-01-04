#ifndef IMAGE_CONVERT_H
#define IMAGE_CONVERT_H

#include "common/image_u32.h"

#include "imagesource/image_source.h"

// Convert frame from imagesource to a useable image_u32_t
image_u32_t *convert_to_image(frame_data_t *frmd);

#endif
