#include "vxo_image.h"
#include "vx_program.h"
#include "vx_codes.h"

// assumes an image whose byte order is R, G, B, A
// (that's (a<<24)+(b<<16)+(g<<8)+(r<<0) on little endian machines (x86, ARM)
// or (r<<24)+(g<<16)+(b<<8)+(a<<0) on big endian machines.
vx_object_t * vxo_image_from_u32(image_u32_t *im, int img_flags, int tex_flags)
{
    vx_resc_t *buf = vx_resc_copyui(im->buf, im->stride * im->height);
    return vxo_image_texflags(buf, im->width, im->height, GL_RGBA, img_flags, tex_flags);
}

vx_object_t * vxo_image(vx_resc_t * tex, int width, int height, int format, int img_flags)
{
    // The decision to make min filter active by default is that when
    // zoomed out (and image is small), you won't see aliasing. But, if
    // you want to zoom into a texture to inspect the pixels, we won't
    // blur anything.
    return vxo_image_texflags(tex, width, height, format, img_flags, VX_TEX_MIN_FILTER);
}

vx_object_t * vxo_image_texflags(vx_resc_t * tex, int width, int height,
                              int format, int img_flags, int tex_flags)
{
    float data[] = { 0.0f, 0.0f,
                     (float)width, 0.0f,
                     0.0f, (float)height,
                     (float)width, (float) height };

    float texcoords[] = { 0.0f, 0.0f,
                          1.0f, 0.0f,
                          0.0f, 1.0f,
                          1.0f, 1.0f };

    if ((img_flags & VXO_IMAGE_FLIPY) != 0) {
        for (int i = 0; i < 4; i++)
            texcoords[2* i + 1] = 1 - texcoords[2*i + 1];
    }

    vx_program_t * program = vx_program_load_library("texture");

    vx_program_set_vertex_attrib(program, "position", vx_resc_copyf(data, 8), 2);
    vx_program_set_vertex_attrib(program, "texIn", vx_resc_copyf(texcoords, 8), 2);
    vx_program_set_texture(program, "texture", tex,  width, height, format, tex_flags);
    vx_program_set_draw_array(program, 4, GL_TRIANGLE_STRIP);

    return program->super;
}
