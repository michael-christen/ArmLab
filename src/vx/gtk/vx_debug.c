#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

// core api
#include "vx/vx.h"

#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/time_util.h"
#include "common/image_util.h"
#include "common/getopt.h"

typedef struct
{
    int running;

    image_u8_t *img;

    vx_application_t app;

    vx_world_t * world;
    zhash_t * layers;

    pthread_mutex_t mutex; // for accessing the arrays
    pthread_t animate_thread;

    pthread_mutex_t display_mutex;
    pthread_cond_t display_cond;
} state_t;



static void draw(state_t * state, vx_world_t * world)
{

    if (1) {
        vx_buffer_add_back(vx_world_get_buffer(world, "grid"),
                           vxo_grid());
        vx_buffer_set_draw_order(vx_world_get_buffer(world, "grid"), -100);
        vx_buffer_swap(vx_world_get_buffer(world, "grid"));
    }

    // Draw from the vx shape library
    if (1) {
        vx_buffer_add_back(vx_world_get_buffer(world, "fixed-cube"),
                           vxo_chain(vxo_mat_translate3(3.0,0,0),
                                     vxo_mat_scale3(2,2,2),
                                     /* vxo_box(vxo_mesh_style(vx_orange)))); */
                                     vxo_box(vxo_mesh_style_fancy(vx_orange, vx_orange, vx_white, 1.0, 400.0, 2))));


        vx_buffer_add_back(vx_world_get_buffer(world, "fixed-cube"),
                           vxo_chain(vxo_mat_translate3(0,3.0,0),
                                     vxo_mat_scale3(1,1,1),
                                     vxo_depth_test(0,
                                                    vxo_box(vxo_mesh_style_solid(vx_green)))));
        vx_buffer_swap(vx_world_get_buffer(world, "fixed-cube"));
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

    pthread_mutex_lock(&state->display_mutex);
    pthread_cond_signal(&state->display_cond);
    pthread_mutex_unlock(&state->display_mutex);
}

static void state_destroy(state_t * state)
{

    if (state->img != NULL)
        image_u8_destroy(state->img);

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
    pthread_mutex_init (&state->display_mutex, NULL);
    pthread_cond_init (&state->display_cond, NULL);

    return state;
}


void * render_loop(void * data)
{
    state_t * state = data;

    pthread_mutex_lock(&state->display_mutex);
    pthread_cond_wait(&state->display_cond, &state->display_mutex);
    pthread_mutex_unlock(&state->display_mutex);

    for (int i = 0; i < 10; i++) {
        double rad = (utime_now() % 5000000) * 2* M_PI / 5e6;

        vx_object_t * vo = vxo_chain(vxo_mat_rotate_z(rad),
                                     vxo_mat_translate2(0,10),
                                     vxo_box(vxo_mesh_style(vx_blue)));

        vx_buffer_add_back(vx_world_get_buffer(state->world, "rotating-square"), vo);
        vx_buffer_swap(vx_world_get_buffer(state->world, "rotating-square"));
        timeutil_usleep(5000);
    }
    vx_buffer_swap(vx_world_get_buffer(state->world, "rotating-square"));

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

    // Load a pnm from file, and repack the data so that it's understandable by vx
    if (strcmp(getopt_get_string(gopt,"pnm"),"")) {
        image_u8_t * img2 = image_u8_create_from_pnm(getopt_get_string(gopt, "pnm"));
        state->img = image_util_convert_rgb_to_rgba(img2);
        image_u8_destroy(img2);
    }



    vx_remote_display_source_t * cxn = vx_remote_display_source_create(&state->app);
    pthread_create(&state->animate_thread, NULL, render_loop, state);

    if (!getopt_get_bool(gopt,"no-gtk")) {
        gdk_threads_init ();
        gdk_threads_enter ();

        gtk_init (&argc, &argv);

        vx_gtk_display_source_t * appwrap = vx_gtk_display_source_create(&state->app);
        GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
        GtkWidget * canvas = vx_gtk_display_source_get_widget(appwrap);
        gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
        gtk_container_add(GTK_CONTAINER(window), canvas);
        gtk_widget_show (window);
        gtk_widget_show (canvas); // XXX Show all causes errors!

        g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

        gtk_main (); // Blocks as long as GTK window is open
        gdk_threads_leave ();

        vx_gtk_display_source_destroy(appwrap);

        // quit when gtk closes? Or wait for remote displays/Ctrl-C
        if (!getopt_get_bool(gopt, "stay-open"))
            state->running = 0;
    }

    pthread_join(state->animate_thread, NULL);
    vx_remote_display_source_destroy(cxn);

    state_destroy(state);
    vx_global_destroy();
    getopt_destroy(gopt);
}
