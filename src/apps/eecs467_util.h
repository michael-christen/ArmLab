#ifndef EECS467_UTIL_H
#define EECS467_UTIL_H

// XXX This is evil
#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "common/image_u32.h"
#include "common/pg.h"

// imagesource
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

// Holds world state, threading tools
typedef struct
{
    int running;

    const char *url;

    vx_application_t app;

    parameter_gui_t *pg;

    vx_world_t *world;  // Where vx objects are live
    zhash_t *layers;

    pthread_mutex_t mutex;  // for accessing the arrays
    pthread_t animate_thread;
} state_t;

void display_finished(vx_application_t *app, vx_display_t *disp);
void display_started(vx_application_t *app, vx_display_t *disp);

void state_destroy(state_t *state);
state_t* state_create();

void init_gui(state_t *state, int w, int h);

#endif
