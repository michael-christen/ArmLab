#ifndef VXO_IMAGE_H
#define VXO_IMAGE_H

#include "vx_resc.h"
#include "vx_object.h"

// Can pass some flags to change the way the image is rendered
#define VXO_IMAGE_NOFLAGS 0
#define VXO_IMAGE_FLIPY   1

#ifdef __cplusplus
extern "C" {
#endif

// img_flags is used for flipping the image
// tex_flags is for tell OpenGL how to filter the image
//   MIN_FILTER = when image is small (being minified), uses a mip-map
//   MAG_FILTER = when image is very large (being magnified), uses blurring
//   REPEAT sets sampling to wrap around near edges (e.g. for rendering
//      a sphere)

vx_object_t *vxo_image(vx_resc_t * tex, int width, int height, int pixel_format, int img_flags);

vx_object_t * vxo_image_texflags(vx_resc_t * tex, int width, int height,
                                 int format, int img_flags, int tex_flags);

// Note, there's currently no image_u8 wrapper because the format of the u8 image is not necessarily fixed. It can contain RGB, GRAY, RGBA, ARGB, etc.

#ifdef __cplusplus
}
#endif
#endif
