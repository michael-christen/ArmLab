#include <gtk/gtk.h>
#include <pthread.h>
#include <endian.h>
#include <assert.h>
#include <stdlib.h>

#include "common/ssocket.h"
#include "common/ioutils.h"
#include "common/getopt.h"

#include "vx/gtk/vx_gtk_display.h"
#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_global.h"
#include "vx/vx_code_output_stream.h"
#include "vx/vx_display.h"

#include "vx/vx_tcp_display.h" // Constants
#include "vx/vx_tcp_util.h"

typedef struct
{
    vx_display_t * disp;
    vx_gtk_display_source_t * src;

    vx_application_t app;
    vx_display_listener_t display_listener;

    // reading is guaranteed to be in a single thread
    // but writing is event based (callbacks):
    pthread_mutex_t write_mutex;

    ssocket_t * ssocket;
    const char * ip;
    int port;

    int running;

    pthread_t listen_thread;
} state_t;


void * listen_run(void * usr)
{
    state_t * state = usr;
    state->running = 1;

    while (state->running) {
        int code = 0;
        int res_cnt = read_fully(ssocket_get_fd(state->ssocket), &code, sizeof(int));
        if (res_cnt < 0) { // error or closed
            printf("ERR: TCP connection error or closed. Exiting!\n"); // XXX better handling of this
            exit(1);
        }
        code = be32toh(code);

        int len = 0;
        res_cnt = read_fully(ssocket_get_fd(state->ssocket), &len, sizeof(int));
        if (res_cnt < 0) { // error or closed
            printf("ERR: TCP connection error or closed. Exiting!\n"); // XXX better handling of this
            exit(1);
        }
        len = be32toh(len);

        uint8_t * buf = malloc(len);
        res_cnt = read_fully(ssocket_get_fd(state->ssocket), buf, len);
        if (res_cnt != len) {
            printf("ERR: TCP connection error or closed. Exiting!\n"); // XXX better handling of this
            exit(1);
        }


        if (code == VX_TCP_CODES) {
            state->disp->send_codes(state->disp, buf, len);
        } else if (code == VX_TCP_ADD_RESOURCES) {
            vx_code_input_stream_t * cins = vx_code_input_stream_create(buf, len);
            zhash_t * resources = vx_tcp_util_unpack_resources(cins);
            state->disp->send_resources(state->disp, resources);
            zhash_destroy(resources);
            vx_code_input_stream_destroy(cins);
        }


        free(buf);
    }
    pthread_exit(NULL);
}

static void display_finished(vx_application_t * app, vx_display_t * disp)
{
    // quit the TCP thread;

    state_t * state = app->impl;
    disp->remove_listener(disp, &state->display_listener);

    printf("exiting\n"); // XXX This isn't working properly.
    close(ssocket_get_fd(state->ssocket));
    ssocket_destroy(state->ssocket);
    pthread_join(state->listen_thread, NULL);
    printf("exiting joined\n");
}

static void display_started(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;
    state->disp = disp;

    disp->add_listener(disp, &state->display_listener);

    // Initialize connection
    state->ssocket = ssocket_create();
    if (ssocket_connect(state->ssocket, state->ip, state->port)) {
        printf("DBG: Connection Failed. Exiting...\n");
        exit(1);
    }
    printf("NFO: Connected to Vx server on %s:%d\n",state->ip, state->port);

    // Add the display listener, which will send on TCP

    // start the TCP listen thread.
    pthread_create(&state->listen_thread, NULL, listen_run, state);
}

static void write_codes(state_t * state, int op_type, vx_code_output_stream_t * couts)

{
    vx_code_output_stream_t * combined = vx_code_output_stream_create(couts->pos + 8);
    combined->write_uint32(combined, op_type);
    combined->write_uint32(combined, couts->pos);
    combined->write_bytes(combined, couts->data, couts->pos);

    pthread_mutex_lock(&state->write_mutex);
    write_fully(ssocket_get_fd(state->ssocket), combined->data, combined->pos);
    pthread_mutex_unlock(&state->write_mutex);

    vx_code_output_stream_destroy(combined);
}


static void viewport_changed (vx_display_listener_t * listener, int width, int height)
{
    state_t * state = listener->impl;

    vx_code_output_stream_t * couts = vx_code_output_stream_create(128);
    couts->write_uint32(couts, width);
    couts->write_uint32(couts, height);

    write_codes(state, VX_TCP_VIEWPORT_SIZE, couts);

    vx_code_output_stream_destroy(couts);
}

static void event_dispatch_key (vx_display_listener_t * listener, int layerID,
                                vx_key_event_t * event)
{
    state_t * state = listener->impl;

    vx_code_output_stream_t * couts = vx_code_output_stream_create(256);
    couts->write_uint32(couts, layerID);

    couts->write_uint32(couts, event->modifiers);
    couts->write_uint32(couts, event->key_code);
    couts->write_uint32(couts, event->released);

    write_codes(state, VX_TCP_EVENT_KEY, couts);

    vx_code_output_stream_destroy(couts);

}

static void event_dispatch_mouse(vx_display_listener_t * listener, int layerID,
                                 vx_camera_pos_t *pos, vx_mouse_event_t * event)
{
    state_t * state = listener->impl;

    vx_code_output_stream_t * couts = vx_code_output_stream_create(256);
    couts->write_uint32(couts, layerID);

    vx_tcp_util_pack_camera_pos(pos, couts);

    couts->write_float(couts, (float)event->x);
    couts->write_float(couts, (float)event->y);

    couts->write_uint32(couts, event->button_mask);
    couts->write_uint32(couts, event->scroll_amt);
    couts->write_uint32(couts, event->modifiers);


    write_codes(state, VX_TCP_EVENT_MOUSE, couts);

    vx_code_output_stream_destroy(couts);
}

static void event_dispatch_touch (vx_display_listener_t * listener, int layerID,
                                  vx_camera_pos_t *pos, vx_touch_event_t * event)
{
    // Doesn't exist for GTK
    assert(0);
}

int main(int argc, char ** argv)
{
    vx_global_init();

    getopt_t * gopt = getopt_create();
    getopt_add_bool   (gopt, 'h', "help", 0, "Show help");
    getopt_add_string (gopt, 'a', "ip-address", "localhost", "Hostname to connect to");
    getopt_add_int    (gopt, 'p', "port", "15151", "Port to connect to");

    // parse and print help
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt,"help")) {
        printf ("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage (gopt);
        exit (1);
    }

    state_t * state = calloc(1, sizeof(state_t));
    pthread_mutex_init(&state->write_mutex, NULL);

    // App
    state->app.display_started =display_started;
    state->app.display_finished =display_finished;
    state->app.impl = state;

    // Display listener
    state->display_listener.viewport_changed = viewport_changed;
    state->display_listener.event_dispatch_key = event_dispatch_key;
    state->display_listener.event_dispatch_mouse = event_dispatch_mouse;
    state->display_listener.event_dispatch_touch = event_dispatch_touch;
    state->display_listener.impl = state;


    // XXX hardcoded
    state->ip = getopt_get_string(gopt, "ip-address");
    state->port = getopt_get_int(gopt, "port");

    gdk_threads_init ();
    gdk_threads_enter ();

    gtk_init (&argc, &argv);

    state->src = vx_gtk_display_source_create(&state->app);
    GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas = vx_gtk_display_source_get_widget(state->src);
    gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
    gtk_container_add(GTK_CONTAINER(window), canvas);
    gtk_widget_show (window);
    gtk_widget_show (canvas); // XXX Show all causes errors!

    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_main (); // Blocks as long as GTK window is open
    gdk_threads_leave ();

    vx_gtk_display_source_destroy(state->src);

    free(state);
    vx_global_destroy();
    getopt_destroy(gopt);
}
