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

// imagesource
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

// Holds world state, threading tools
typedef struct
{
    int running;

    char *url;

    vx_application_t app;

    vx_world_t *world;  // Where vx objects are live
    zhash_t *layers;

    pthread_mutex_t mutex;  // for accessing the arrays
    pthread_t animate_thread;
} state_t;

static void display_finished(vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;
    pthread_mutex_lock(&state->mutex);

    vx_layer_t *layer = NULL;
    zhash_remove(state->layers, &disp, NULL, &layer);
    vx_layer_destroy(layer);

    pthread_mutex_unlock(&state->mutex);
}

static void display_started(vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;

    vx_layer_t *layer = vx_layer_create(state->world);
    vx_layer_set_display(layer, disp);

    pthread_mutex_lock(&state->mutex);
    // store a reference to the world and layer that we associate with each
    // vx_display_t
    zhash_put(state->layers, &disp, &layer, NULL, NULL);
    pthread_mutex_unlock(&state->mutex);
}

static void state_destroy(state_t *state)
{
    vx_world_destroy(state->world);
    assert(zhash_size(state->layers) == 0);

    zhash_destroy(state->layers);
    free(state);

    pthread_mutex_destroy(&state->mutex);
}

static state_t* state_create()
{
    state_t *state = calloc(1, sizeof(state_t));
    state->running = 1;
    state->app.impl = state;
    state->app.display_started = display_started;
    state->app.display_finished = display_finished;

    state->world = vx_world_create();
    state->layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    pthread_mutex_init(&state->mutex, NULL);

    return state;
}

// === Your code goes here ==================================================
// The render loop handles your visualization updates. It is the function run
// by the animate_thread. It periodically renders the contents on the
// vx world contained by state
void* render_loop(void *data)
{
    int fps = 60;
    state_t *state = data;

    // Set up the imagesource
    image_source_t *isrc = image_source_open(state->url);

    if (isrc == NULL) {
        printf("Error opening device.\n");
    } else {
        // Print out possible formats
        for (int i = 0; i < isrc->num_formats(isrc); i++) {
            image_source_format_t *ifmt = isrc->get_format(isrc, i);
            printf("%3d: %4d x %4d (%s)\n", i, ifmt->width, ifmt->height, ifmt->format);
        }
        isrc->start(isrc);
    }

    // Continue running until we are signaled otherwise. This happens
    // when the window is closed/Ctrl+C is received.
    while (state->running) {

        // Get the most recent camera frame, if applicable
        if (isrc != NULL) {
            frame_data_t * frmd = calloc(1, sizeof(frame_data_t));
            int res = isrc->get_frame(isrc, frmd);
            if (res < 0) {
                printf("get_frame fail: %d\n", res);
            } else {
                // Handle frame
                image_u32_t *im = convert_to_image(frmd);

                vx_resc_t *buf = vx_resc_copyui(im->buf,
                                                im->stride*im->height);

                vx_object_t *vim = vxo_image_texflags(buf,
                                                      im->width,
                                                      im->height,
                                                      GL_RGBA,
                                                      VXO_IMAGE_FLIPY,
                                                      VX_TEX_MIN_FILTER |
                                                      VX_TEX_MAG_FILTER);

                vx_buffer_add_back(vx_world_get_buffer(state->world, "image"),
                                   vxo_chain(vxo_mat_translate3(-im->width/2,-im->height/2,0),
                                             vim));
                vx_buffer_swap(vx_world_get_buffer(state->world, "image"));
                image_u32_destroy(im);
            }

            fflush(stdout);
            isrc->release_frame(isrc, frmd);
        }
        // Example rendering
        double rad = (vx_mtime() % 5000) * 2 * M_PI / 5e3;   // [0,2PI]
        double osc = ((vx_mtime() % 5000) / 5e3) * 2 - 1;    // [-1, 1]

        // Creates a blue box and applies a series of rigid body transformations
        // to it. A vxo_chain applies its arguments sequentially. In this case,
        // then, we rotate our coordinate frame by rad radians, as determined
        // by the current time above. Then, the origin of our coordinate frame
        // is translated 0 meters along its X-axis and 10 meters along its
        // Y-axis. Finally, a 1x1x1 cube (or box) is rendered centered at the
        // origin, and is rendered with the blue mesh style, meaning it has
        // solid, blue sides.
        vx_object_t *vo = vxo_chain(vxo_mat_rotate_z(rad),
                                    vxo_mat_translate2(0,10),
                                    vxo_box(vxo_mesh_style(vx_blue)));

        // Then, we add this object to a buffer awaiting a render order
        vx_buffer_add_back(vx_world_get_buffer(state->world, "rot-square"), vo);

        // Now we will render a red box that translates back and forth. This
        // time, there is no rotation of our coordinate frame, so the box will
        // just slide back and forth along the X axis. This box is rendered
        // with a red line style, meaning it will appear as a red wireframe,
        // in this case, with lines 2 px wide.
        vo = vxo_chain(vxo_mat_translate2(osc*5,0),
                       vxo_box(vxo_lines_style(vx_red, 2)));

        // We add this object to a different buffer so it may be rendered
        // separately if desired
        vx_buffer_add_back(vx_world_get_buffer(state->world, "osc-square"), vo);

        // Now, we update both buffers
        vx_buffer_swap(vx_world_get_buffer(state->world, "rot-square"));
        vx_buffer_swap(vx_world_get_buffer(state->world, "osc-square"));

        usleep(1000000/fps);
    }

    if (isrc != NULL)
        isrc->stop(isrc);

    return NULL;
}

// This is intended to give you a starting point to work with for any program
// requiring a GUI. This handles all of the GTK and vx setup, allowing you to
// fill in the functionality with your own code.
int main(int argc, char **argv)
{
    // Parse arguments from the command line, showing the help
    // screen if required
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show help");
    getopt_add_string(gopt, '\0', "url", "", "Camera URL");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help"))
    {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(gopt);
        exit(1);
    }

    // Call this to initialize the vx-wide lock. Required to start the GL
    // threrad or to use the program library
    vx_global_init();

    state_t *state = state_create();

    // Set up the imagesource
    if (strncmp(getopt_get_string(gopt, "url"), "", 1)) {
        state->url = getopt_get_string(gopt, "url");
    } else {
        // No URL specified. Show all available and then
        // use the first

        char **urls = image_source_enumerate();
        printf("Cameras:\n");
        for (int i = 0; urls[i] != NULL; i++)
            printf("  %3d: %s\n", i, urls[i]);

        if (urls[0]==NULL) {
            printf("Found no cameras.\n");
            return -1;
        }

        state->url = urls[0];
    }

    // PNM/Drawing stuff XXX

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your
    // display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create(&state->app);
    pthread_create(&state->animate_thread, NULL, render_loop, state);

    // Initialize GTK
    gdk_threads_init();
    gdk_threads_enter();

    gtk_init(&argc, &argv);

    // Creates a GTK window to wrap around our vx display canvas. The vx world
    // is rendered to the canvas widget, which acts as a viewport into your
    // virtual world.
    vx_gtk_display_source_t *appwrap = vx_gtk_display_source_create(&state->app);
    GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    GtkWidget *canvas = vx_gtk_display_source_get_widget(appwrap);
    gtk_window_set_default_size(GTK_WINDOW(window), 800, 600);
    gtk_container_add(GTK_CONTAINER(window), canvas);
    gtk_widget_show(window);
    gtk_widget_show(canvas);    // XXX Show all causes errors!

    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_main(); // Blocks as long as GTK window is open
    gdk_threads_leave();

    vx_gtk_display_source_destroy(appwrap);

    // Quit when GTK closes
    state->running = 0;


    pthread_join(state->animate_thread, NULL);
    vx_remote_display_source_destroy(cxn);

    state_destroy(state);
    vx_global_destroy();
    getopt_destroy(gopt);
}
