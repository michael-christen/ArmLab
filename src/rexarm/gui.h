#ifndef __GUI_H
#define __GUI_H

#include "eecs467_util.h"

typedef struct gstate gstate_t;

void gui_update_servo_pos(double servo_pos[]);
void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name);
void* render_loop(void *data);
void* render_arm(void* data);
void* gui_create(int argc, char **argv);

#endif
