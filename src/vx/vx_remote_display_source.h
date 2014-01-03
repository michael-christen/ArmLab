#ifndef VX_REMOTE_DISPLAY_SOURCE_H
#define VX_REMOTE_DISPLAY_SOURCE_H

#include "vx_display.h"

typedef struct vx_remote_display_source vx_remote_display_source_t;

#ifdef __cplusplus
extern "C" {
#endif

// Create a new connection manager, with a callback when a new connection is established
// User should free/manager *app after this call returns
vx_remote_display_source_t * vx_remote_display_source_create(vx_application_t * app);
void vx_remote_display_source_destroy(vx_remote_display_source_t* remote_source);


#ifdef __cplusplus
}
#endif

#endif
