#include "vx_ray3.h"

void vx_ray3_intersect_xy(vx_ray3_t * ray, double zheight, double * vec3_out)
{
    double dist = (ray->source[2] - zheight)/ray->dir[2];

    vec3_out[0] = ray->source[0] - ray->dir[0]*dist;
    vec3_out[1] = ray->source[1] - ray->dir[1]*dist;
    vec3_out[2] = zheight;
}
