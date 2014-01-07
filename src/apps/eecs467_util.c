#include "eecs467_util.h"

void display_finished(vx_application_t *app, vx_display_t *disp)
{
    state_t *state = app->impl;
    pthread_mutex_lock(&state->mutex);

    vx_layer_t *layer = NULL;
    zhash_remove(state->layers, &disp, NULL, &layer);
    vx_layer_destroy(layer);

    pthread_mutex_unlock(&state->mutex);
}

void display_started(vx_application_t *app, vx_display_t *disp)
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

void state_destroy(state_t *state)
{
    vx_world_destroy(state->world);
    assert(zhash_size(state->layers) == 0);

    pg_destroy(state->pg);

    zhash_destroy(state->layers);
    free(state);

    pthread_mutex_destroy(&state->mutex);
}

state_t* state_create()
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

void init_gui(state_t *state, int w, int h)
{
    // Creates a GTK window to wrap around our vx display canvas. The vx world
    // is rendered to the canvas widget, which acts as a viewport into your
    // virtual world.
    vx_gtk_display_source_t *appwrap = vx_gtk_display_source_create(&state->app);
    GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    GtkWidget *canvas = vx_gtk_display_source_get_widget(appwrap);
    GtkWidget *pgui = pg_get_widget(state->pg);
    gtk_window_set_default_size(GTK_WINDOW(window), w, h);

    // Pack a parameter gui and canvas into a vertical box
    GtkWidget *vbox = (GtkWidget*)gtk_vbox_new(0, 0);
    gtk_box_pack_start(GTK_BOX(vbox), canvas, 1, 1, 0);
    gtk_widget_show(canvas);    // XXX Show all causes errors!
    gtk_box_pack_start(GTK_BOX(vbox), pgui, 0, 0, 0);
    gtk_widget_show(pgui);

    gtk_container_add(GTK_CONTAINER(window), vbox);
    gtk_widget_show(window);
    gtk_widget_show(vbox);

    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_main(); // Blocks as long as GTK window is open
    gdk_threads_leave();

    vx_gtk_display_source_destroy(appwrap);
}
