#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdint.h>

#include "common/getopt.h"
#include "common/image_u8x3.h"
#include "image_source.h"
#include "image_convert.h"

typedef struct state state_t;
struct state
{
    char     *url; // image_source url
    getopt_t *gopt;

    pthread_t runthread;

    GtkWidget *window;
    GtkWidget *image;

};

void my_gdkpixbufdestroy(guchar *pixels, gpointer data)
{
    free(pixels);
}

void *runthread(void *_p)
{
    state_t *state = (state_t*) _p;

    image_source_t *isrc = image_source_open(state->url);
    if (isrc->start(isrc))
        goto error;

    printf("Image source formats:\n");
    for (int i = 0; i < isrc->num_formats(isrc); i++) {
        image_source_format_t ifmt;
        isrc->get_format(isrc, i, &ifmt);
        printf("\t%d\t%4d x %4d (%s)\n", i, ifmt.width, ifmt.height, ifmt.format);
    }

    printf("Image source features:\n");
    for (int i = 0; i < isrc->num_features(isrc); i++) {
        const char *feature_name = isrc->get_feature_name(isrc, i);
        char *feature_type = isrc->get_feature_type(isrc, i);
        double v = isrc->get_feature_value(isrc, i);

        printf("\t%-20s [%-10s], current = %f\n", feature_name, feature_type, v);
        free(feature_type);
    }

    while (1) {

        image_source_data_t isdata;
        if (isrc->get_frame(isrc, &isdata))
            goto error;

        image_u8x3_t *im = image_convert_u8x3(&isdata);
        if (im != NULL) {
            gdk_threads_enter();

            GdkPixbuf *pixbuf = gdk_pixbuf_new_from_data((guchar*) im->buf,
                                                         GDK_COLORSPACE_RGB, 0, 8,
                                                         im->width, im->height, im->stride,
                                                         my_gdkpixbufdestroy,
                                                         NULL);


            gtk_image_set_from_pixbuf(GTK_IMAGE(state->image), pixbuf);
            g_object_unref(G_OBJECT(pixbuf));

            gdk_threads_leave();
        }

        isrc->release_frame(isrc, &isdata);
    }

  error:
    isrc->stop(isrc);
    printf("exiting\n");

    return NULL;
}

int main(int argc, char *argv[] )
{
    state_t *state = calloc(1, sizeof(state_t));
    state->gopt = getopt_create();

    getopt_add_bool(state->gopt, 'h', "--help", 0, "Show this help");

    if (!getopt_parse(state->gopt, argc, argv, 0)) {
        getopt_do_usage(state->gopt);
        exit(-1);
    }

    const zarray_t *args = getopt_get_extra_args(state->gopt);
    if (zarray_size(args) > 0) {
        zarray_get(args, 0, &state->url);
    } else {
        char **urls = image_source_enumerate();

        printf("Cameras:\n");
        for (int i = 0; urls[i] != NULL; i++)
            printf("  %3d: %s\n", i, urls[i]);

        if (urls[0] == NULL) {
            printf("No cameras found.\n");
            exit(0);
        }
        state->url = urls[0];
    }

    g_type_init();
    gtk_init (&argc, &argv);
    gdk_threads_init();

    state->window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(state->window), state->url);

    state->image = gtk_image_new_from_file("/tmp/csedw.jpg");

    gtk_container_add(GTK_CONTAINER(state->window), state->image);
    gtk_widget_show(state->image);
    gtk_widget_show(state->window);

    pthread_create(&state->runthread, NULL, runthread, state);

    gtk_main();



    return 0;
}
