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

#include "common/string_util.h"
#include "common/getopt.h"

typedef struct
{
    int running;

    vx_application_t app;

    vx_world_t * world;
    zhash_t * layers;

    pthread_mutex_t mutex; // for accessing the arrays
    pthread_t animate_thread;
} state_t;



static void draw(state_t * state, vx_world_t * world)
{
    if (1) {
        vx_buffer_add_back(vx_world_get_buffer(world, "grid"),
                           vxo_grid());
        vx_buffer_set_draw_order(vx_world_get_buffer(world, "grid"), -100);
        vx_buffer_swap(vx_world_get_buffer(world, "grid"));
    }

    if (1) {
        vx_object_t *vt = vxo_text_create(VXO_TEXT_ANCHOR_TOP_RIGHT, "<<right,#0000ff>>Heads Up!\n");
        vx_buffer_t *vb = vx_world_get_buffer(world, "text");
        vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP_RIGHT,vt));
        vx_buffer_swap(vb);
    }
}

static void display_finished(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;
    pthread_mutex_lock(&state->mutex);

    vx_layer_t * layer = NULL;

    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_remove(state->layers, &disp, NULL, &layer);

    vx_layer_destroy(layer);

    pthread_mutex_unlock(&state->mutex);
}

static void display_started(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;

    vx_layer_t * layer = vx_layer_create(state->world);
    vx_layer_set_display(layer, disp);

    pthread_mutex_lock(&state->mutex);
    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_put(state->layers, &disp, &layer, NULL, NULL);
    pthread_mutex_unlock(&state->mutex);
}

static void state_destroy(state_t * state)
{

    vx_world_destroy(state->world);
    assert(zhash_size(state->layers) == 0);

    zhash_destroy(state->layers);
    free(state);

    pthread_mutex_destroy(&state->mutex);

}

static state_t * state_create()
{
    state_t * state = calloc(1, sizeof(state_t));
    state->running = 1;
    state->app.impl=state;
    state->app.display_started=display_started;
    state->app.display_finished=display_finished;


    state->world = vx_world_create();
    state->layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    pthread_mutex_init (&state->mutex, NULL);

    return state;
}


void * render_loop(void * data)
{
    state_t * state = data;
    uint32_t loopCt = 0;
    while(state->running) {
        double rad = (vx_mtime() % 5000) * 2* M_PI / 5e3;

        vx_object_t * vo = vxo_chain(vxo_mat_rotate_z(rad),
                                     vxo_mat_translate2(0,10),
                                     vxo_box(vxo_mesh_style(vx_blue)));

        char * label  = sprintf_alloc("<<right,#000000>>Rad %1.4f! Loop Ct %d\n", rad, loopCt);
        vx_object_t *vt = vxo_text_create(VXO_TEXT_ANCHOR_TOP_LEFT, label);
        vx_buffer_add_back(vx_world_get_buffer(state->world, "rotating-square"), vxo_pix_coords(VX_ORIGIN_TOP_LEFT,vt));
        free(label);
        vx_buffer_add_back(vx_world_get_buffer(state->world, "rotating-square"), vo);
        vx_buffer_swap(vx_world_get_buffer(state->world, "rotating-square"));
        usleep(100000);
        loopCt++;
    }

    return NULL;
}

int main(int argc, char ** argv)
{
    getopt_t *gopt = getopt_create();
    getopt_add_bool   (gopt, 'h', "help", 0, "Show help");
    getopt_add_bool (gopt, '\0', "no-gtk", 0, "Don't show gtk window, only advertise remote connection");
    getopt_add_string (gopt, '\0', "pnm", "", "Path for pnm file to render as texture (.e.g BlockM.pnm)");
    getopt_add_bool (gopt, '\0', "stay-open", 0, "Stay open after gtk exits to continue handling remote connections");

    // parse and print help
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt,"help")) {
        printf ("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage (gopt);
        exit (1);
    }

    vx_global_init(); // Call this to initialize the vx-wide lock. Required to start the GL thread or to use the program library

    state_t * state = state_create();


    draw(state, state->world);

    vx_remote_display_source_t * cxn = vx_remote_display_source_create(&state->app);
    pthread_create(&state->animate_thread, NULL, render_loop, state);

    pthread_join(state->animate_thread, NULL);
    vx_remote_display_source_destroy(cxn);

    state_destroy(state);
    vx_global_destroy();
    getopt_destroy(gopt);
}
