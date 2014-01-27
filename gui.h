#ifndef __GUI_H
#define __GUI_H

#include "eecs467_util.h"

#include "../vx/vx_types.h"
#include "../vx/vx_event.h"
#include "../vx/vx_event_handler.h"

#include <lcm/lcm.h>
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"

typedef struct gstate gstate_t;


typedef struct event_state event_state_t;
struct event_state {
    int x, y;
	vx_mouse_event_t last_mouse;
	int init_last;
};

void gui_update_servo_pos(double servo_pos[]);
void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name);
void* render_loop(void *data);
void* render_arm(void* data);
void* gui_create(int argc, char **argv);

#endif
