//#include "eecs467_util.h"    // This is where a lot of the internals live
#include "gui.h"

#define NUM_SERVOS 6
// It's good form for every application to keep its gstate in a struct.

struct gstate
{
    char *url;
    int running;

    vx_application_t  vxapp;
    getopt_t         *gopt;
    parameter_gui_t  *pg;

    vx_world_t *world;  // Where vx objects are live

    zhash_t *layers; // vx_display_t* -> vx_layer_t*

    pthread_mutex_t mutex;  // for accessing the arrays
    pthread_t animate_thread;
};

int camera_zoom = 400;
int camera_show = 1;
int arm_zoom = 25;
int view_above = 1;
double servo_positions[NUM_SERVOS];

lcm_t *lcm;
const char *gui_channel;

void gui_update_servo_pos(double servo_pos[]){
	int i;
	for(i = 0; i < NUM_SERVOS ;i++){
		servo_positions[i] = servo_pos[i];
	}
	servo_positions[0] += M_PI;
	//servo_positions[4] -= M_PI/2.0;
}

// === Parameter listener =================================================
// This function is handled to the parameter gui (via a parameter listener)
// and handles events coming from the parameter gui. The parameter listener
// also holds a void* pointer to "impl", which can point to a struct holding
// gstate, etc if need be.
void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
   /* if (!strcmp("s1", name)) {
       servo_positions[0] = pg_gi(pg, name);//camera_zoom = pg_gi(pg, name);
    } else if (!strcmp("s2", name)) {
        servo_positions[1] = pg_gi(pg, name);//arm_zoom = pg_gi(pg, name);
	} if (!strcmp("s3", name)) {
       servo_positions[2] = pg_gi(pg, name);
    } if (!strcmp("s4", name)) {
       servo_positions[3] = pg_gi(pg, name);
    } if (!strcmp("s5", name)) {
       servo_positions[5] = pg_gi(pg, name);*/
	if (!strcmp("sl1", name)) {
       camera_zoom = pg_gi(pg, name);
    } else if (!strcmp("sl2", name)) {
       arm_zoom = pg_gi(pg, name);
    } else if (!strcmp("cb1", name)){
		//camera_show = pg_gb(pg, name);
	} else {
        printf("%s changed\n", name);
    }
}

static int64_t utime_now()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

static int custom_mouse_event(vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    event_state_t *state = vh->impl;

    // Stack allocate the most recent mouse event, so we can return from this function at any time
    vx_mouse_event_t last_mouse;
    if (state->init_last) {
        memcpy(&last_mouse, &state->last_mouse, sizeof(vx_mouse_event_t));
        memcpy(&state->last_mouse, mouse, sizeof(vx_mouse_event_t));
    } else {
        memcpy(&state->last_mouse, mouse, sizeof(vx_mouse_event_t));
        state->init_last = 1;
        return 0;
    }

    int diff_button = mouse->button_mask ^ last_mouse.button_mask;
    //int button_down = diff_button & mouse->button_mask; // which button(s) just got pressed?
    int button_up = diff_button & last_mouse.button_mask; // which button(s) just got released?
	if(button_up){
		dynamixel_status_list_t stats;
		stats.len = 1;
		stats.statuses = malloc(sizeof(dynamixel_status_t));
		stats.statuses[0].utime = utime_now();
		stats.statuses[0].speed = mouse->x;
		stats.statuses[0].load = mouse->y;
		dynamixel_status_list_t_publish(lcm, gui_channel, &stats);
	}

    // Store the last mouse click
    state->x = mouse->x;
    state->y = mouse->y;

    return 0; // Returning 0 says that you have consumed the event. If the event is not consumed (return 1), then it is passed down the chain to the other event handlers.
}

// === Your code goes here ================================================
// The render loop handles your visualization updates. It is the function run
// by the animate_thread. It periodically renders the contents on the
// vx world contained by gstate
void* render_loop(void *data)
{
    //int fps = 60;
    gstate_t *gstate = data;

	while(!zhash_size(gstate->layers)){
		usleep(1000);
	}

	event_state_t *event_state = malloc(sizeof(event_state_t));
	vx_event_handler_t *my_event_handler = malloc(sizeof(vx_event_handler_t));
	my_event_handler->dispatch_order = 0; // Lower number here means that you get higher priority in processing events.
	my_event_handler->impl = event_state;
	my_event_handler->mouse_event = custom_mouse_event;

	zhash_iterator_t it;
	zhash_iterator_init(gstate->layers, &it);
	vx_display_t *key;
	vx_layer_t *value;
	
	float lowLeft[2] = {0, 0};
	float upRight[2] = {1296, 964};	//Camera dimensions
	while(zhash_iterator_next(&it, &key, &value)){
		vx_layer_camera_fit2D(value, lowLeft, upRight, 1); 
		vx_layer_add_event_handler(value, my_event_handler);
	}

    // Set up the imagesource
	printf("test1\n");
    image_source_t *isrc = image_source_open(gstate->url);
	printf("test2\n");
    if (isrc == NULL) {
        printf("Error opening device.\n");
    } else {
        // Print out possible formats. If no format was specified in the
        // url, then format 0 is picked by default.
        // e.g. of setting the format parameter to format 2:
        //
        // --url=dc1394://bd91098db0as9?fidx=2
        for (int i = 0; i < isrc->num_formats(isrc); i++) {
            image_source_format_t ifmt;
            isrc->get_format(isrc, i, &ifmt);
            printf("%3d: %4d x %4d (%s)\n", i, ifmt.width, ifmt.height, ifmt.format);
        }
        isrc->start(isrc);
    }

    // Continue running until we are signaled otherwise. This happens
    // when the window is closed/Ctrl+C is received.
    while (gstate->running && camera_show) {

        // Get the most recent camera frame and render it to screen.
        if (isrc != NULL) {
            image_source_data_t * frmd = calloc(1, sizeof(image_source_data_t));
            int res = isrc->get_frame(isrc, frmd);
            if (res < 0) {
                printf("get_frame fail: %d\n", res);
            } else {
                // Handle frame
                image_u32_t *im = image_convert_u32(frmd);
                if (im != NULL) {

                    vx_object_t *vim = vxo_image_from_u32(im,
                                                          VXO_IMAGE_FLIPY,
                                                          VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

                    vx_buffer_add_back(vx_world_get_buffer(gstate->world, "image"),
                                       vxo_chain(vxo_mat_translate3(0,0,camera_zoom),
                                                 vim));
                    vx_buffer_swap(vx_world_get_buffer(gstate->world, "image"));
                    image_u32_destroy(im);
                }
            }

            fflush(stdout);
            isrc->release_frame(isrc, frmd);

			char statusText[128] = "";
		
			sprintf(statusText, "Servo angles:\n0[%f] 1[%f] 2[%f]\n3[%f] 4[%f] 5[%f]", servo_positions[0]-M_PI, servo_positions[1], servo_positions[2], servo_positions[3], servo_positions[4], servo_positions[5]);
		
			vx_object_t* text = vxo_text_create(VXO_TEXT_ANCHOR_BOTTOM_LEFT, statusText);
			vx_buffer_add_back(vx_world_get_buffer(gstate->world, "text"), vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, text));
			vx_buffer_swap(vx_world_get_buffer(gstate->world, "text"));
        }
		/*
        // Example rendering of vx primitives
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
                                    vxo_sphere(vxo_mesh_style(vx_blue)));

        // Then, we add this object to a buffer awaiting a render order
        vx_buffer_add_back(vx_world_get_buffer(gstate->world, "rot-sphere"), vo);

        // Now we will render a red box that translates back and forth. This
        // time, there is no rotation of our coordinate frame, so the box will
        // just slide back and forth along the X axis. This box is rendered
        // with a red line style, meaning it will appear as a red wireframe,
        // in this case, with lines 2 px wide.
        vo = vxo_chain(vxo_mat_translate2(osc*5,0),
                       vxo_box(vxo_lines_style(vx_red, 2)));

        // We add this object to a different buffer so it may be rendered
        // separately if desired
        vx_buffer_add_back(vx_world_get_buffer(gstate->world, "osc-square"), vo);

        // Now, we update both buffers
        vx_buffer_swap(vx_world_get_buffer(gstate->world, "rot-sphere"));
        vx_buffer_swap(vx_world_get_buffer(gstate->world, "osc-square"));


        uslee(1000000/fps);*/
    }

    if (isrc != NULL)
        isrc->stop(isrc);

    return NULL;
}

void* render_arm(void* data){
	gstate_t *gstate = data;

	while(!zhash_size(gstate->layers)){
		usleep(1000);
	}

	event_state_t *event_state = malloc(sizeof(event_state_t));
	vx_event_handler_t *my_event_handler = malloc(sizeof(vx_event_handler_t));
	my_event_handler->dispatch_order = 0; // Lower number here means that you get higher priority in processing events.
	my_event_handler->impl = event_state;
	my_event_handler->mouse_event = custom_mouse_event;

	zhash_iterator_t it;
	zhash_iterator_init(gstate->layers, &it);
	vx_display_t *key;
	vx_layer_t *value;

	while(zhash_iterator_next(&it, &key, &value) && view_above){
		vx_layer_add_event_handler(value, my_event_handler);
	}

	float blue[4] = {0.0f, 0.0f, 1.0f, 1.0f};

	vx_buffer_add_back(vx_world_get_buffer(gstate->world, "grid"),  vxo_chain(vxo_mat_translate3(0,0,-2*arm_zoom),
		vxo_mat_scale3(1.5, 1.5, 1),
		vxo_grid()));
	vx_buffer_swap(vx_world_get_buffer(gstate->world, "grid"));

	while(gstate->running){
		double block1size = 7;
		double block2size = 4;
		double block3size = 10;
		double block4size = 10;
		double block5size = 10;
		double block6size = 8;
		double totalsize = block1size;
		double totalxshift = 0;
		double totalzshift = arm_zoom;
		double totalyshift = 0;
		double totalangle = 0;	

		if(view_above){
			totalzshift -= 25;
		}else{
			totalyshift = -25;
		}

		

		vx_object_t *block1 = vxo_chain(vxo_mat_rotate_x(view_above*M_PI/2),
				vxo_mat_translate3(totalxshift, block1size/2.0 + totalyshift, totalzshift),
				vxo_mat_rotate_y(M_PI/2.0),
				vxo_mat_scale3(3, block1size, 5),
				vxo_box(vxo_mesh_style(blue),
				vxo_lines_style(vx_red, 2.0f)));
		vx_buffer_add_back(vx_world_get_buffer(gstate->world, "block1"), block1);

		vx_object_t *block2 = vxo_chain(vxo_mat_rotate_x(view_above*M_PI/2),
				vxo_mat_translate3(totalxshift, totalsize + block2size/2.0 + totalyshift, totalzshift),
				vxo_mat_rotate_y(servo_positions[0]),
				vxo_mat_scale3(3, block2size, 5),
				vxo_box(vxo_mesh_style(blue),
				vxo_lines_style(vx_red, 2.0f)));
		vx_buffer_add_back(vx_world_get_buffer(gstate->world, "block2"), block2);
		
		totalsize += block2size;
	
		vx_object_t *block3 = vxo_chain(vxo_mat_rotate_x(view_above*M_PI/2),
				vxo_mat_translate3(totalxshift-sin(servo_positions[1])*block3size/2.0*cos(servo_positions[0]), totalsize + block3size/2.0*cos(servo_positions[1]) + totalyshift, totalzshift + block3size/2.0*sin(servo_positions[0])*sin(servo_positions[1])),
				vxo_mat_rotate_y(servo_positions[0]),
				vxo_mat_rotate_z(servo_positions[1]),
				vxo_mat_scale3(3, block3size, 5),
				vxo_box(vxo_mesh_style(blue),
				vxo_lines_style(vx_red, 2.0f)));
		vx_buffer_add_back(vx_world_get_buffer(gstate->world, "block3"), block3);

		totalangle += servo_positions[1];
		totalxshift -= block3size*sin(totalangle)*cos(servo_positions[0]);
		totalzshift += block3size*sin(servo_positions[0])*sin(servo_positions[1]);
		totalsize += block3size*cos(totalangle);

		vx_object_t *block4 = vxo_chain(vxo_mat_rotate_x(view_above*M_PI/2),
				vxo_mat_translate3(totalxshift -sin(totalangle + servo_positions[2])*block4size/2.0*cos(servo_positions[0]), totalsize + block4size/2.0*cos(totalangle + servo_positions[2]) + totalyshift, totalzshift + block4size/2.0*sin(servo_positions[0])*sin(totalangle + servo_positions[2])),
				vxo_mat_rotate_y(servo_positions[0]),
				vxo_mat_rotate_z(totalangle + servo_positions[2]),
				vxo_mat_scale3(3, block4size, 5),
				vxo_box(vxo_mesh_style(blue),
				vxo_lines_style(vx_red, 2.0f)));
		vx_buffer_add_back(vx_world_get_buffer(gstate->world, "block4"), block4);

		totalangle += servo_positions[2];
		totalxshift -= block4size*sin(totalangle)*cos(servo_positions[0]);
		totalzshift += block4size*sin(servo_positions[0])*sin(totalangle);
		totalsize += block4size*cos(totalangle);

		vx_object_t *block5 = vxo_chain(vxo_mat_rotate_x(view_above*M_PI/2),
				vxo_mat_translate3(totalxshift -sin(totalangle + servo_positions[3])*block5size/2.0*cos(servo_positions[0]), totalsize + block5size/2.0*cos(totalangle + servo_positions[3]) + totalyshift, totalzshift + block5size/2.0*sin(servo_positions[0])*sin(totalangle + servo_positions[3])),
				vxo_mat_rotate_y(servo_positions[0]),
				vxo_mat_rotate_z(totalangle + servo_positions[3]),
				vxo_mat_scale3(3, block5size, 5),
				vxo_box(vxo_mesh_style(blue),
				vxo_lines_style(vx_red, 2.0f)));
		vx_buffer_add_back(vx_world_get_buffer(gstate->world, "block5"), block5);

		totalangle += servo_positions[3];
		totalxshift -= block5size*sin(totalangle)*cos(servo_positions[0]);
		totalzshift += block5size*sin(servo_positions[0])*sin(totalangle);
		totalsize += block5size*cos(totalangle);

		vx_object_t *block6 = vxo_chain(vxo_mat_rotate_x(view_above*M_PI/2.0),
				vxo_mat_translate3(cos(servo_positions[0])*cos(totalangle) + totalxshift -sin(totalangle)*block6size/2.0*cos(servo_positions[0]), totalsize + block6size/2.0*cos(totalangle) + sin(totalangle) + totalyshift, totalzshift - sin(servo_positions[0])*cos(totalangle) + block6size/2.0*sin(servo_positions[0])*sin(totalangle)),
				vxo_mat_rotate_y(servo_positions[0]),//+servo_positions[4]*cos(totalangle)),
				//vxo_mat_rotate_x(servo_positions[4]*sin(totalangle)),
				vxo_mat_rotate_z(totalangle),
				vxo_mat_scale3(1, block6size, 5),
				vxo_box(vxo_mesh_style(blue),
				vxo_lines_style(vx_red, 2.0f)));
		vx_buffer_add_back(vx_world_get_buffer(gstate->world, "block6"), block6);

		vx_object_t *claw = vxo_chain(vxo_mat_rotate_x(view_above*M_PI/2.0),
				vxo_mat_translate3(-cos(servo_positions[0])*cos(totalangle) + totalxshift - cos(totalangle-servo_positions[5])*block6size/2.0*cos(servo_positions[0]), totalsize + block6size/2.0*cos(totalangle - servo_positions[5] + M_PI/2.0) - sin(totalangle) + totalyshift, totalzshift + sin(servo_positions[0])*cos(totalangle) + block6size/2.0*sin(servo_positions[0])*sin(totalangle - servo_positions[5] + M_PI/2.0)),
				vxo_mat_rotate_y(servo_positions[0]),//+servo_positions[4]*cos(totalangle)),
				//vxo_mat_rotate_x(servo_positions[4]*sin(totalangle)),
				vxo_mat_rotate_z(totalangle - servo_positions[5]),
				vxo_mat_scale3(block6size, 1, 5),
				vxo_box(vxo_mesh_style(blue),
				vxo_lines_style(vx_red, 2.0f)));
		vx_buffer_add_back(vx_world_get_buffer(gstate->world, "claw"), claw);	


		vx_buffer_swap(vx_world_get_buffer(gstate->world, "block1"));
		vx_buffer_swap(vx_world_get_buffer(gstate->world, "block2"));
		vx_buffer_swap(vx_world_get_buffer(gstate->world, "block3"));
		vx_buffer_swap(vx_world_get_buffer(gstate->world, "block4"));
		vx_buffer_swap(vx_world_get_buffer(gstate->world, "block5"));
		vx_buffer_swap(vx_world_get_buffer(gstate->world, "block6"));
		vx_buffer_swap(vx_world_get_buffer(gstate->world, "claw"));
	
		char statusText[128] = "";
		
		sprintf(statusText, "Servo angles:\n0[%f] 1[%f] 2[%f]\n3[%f] 4[%f] 5[%f]", servo_positions[0]-M_PI, servo_positions[1], servo_positions[2], servo_positions[3], servo_positions[4], servo_positions[5]);
		
		vx_object_t* text = vxo_text_create(VXO_TEXT_ANCHOR_BOTTOM_LEFT, statusText);
	vx_buffer_add_back(vx_world_get_buffer(gstate->world, "text"), vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, text));
	vx_buffer_swap(vx_world_get_buffer(gstate->world, "text"));
	usleep(5000);
	}
	return NULL;
}

void* gui_create(int argc, char **argv){
	eecs467_init(argc, argv);

    gstate_t *gstate = calloc(1, sizeof(gstate_t));
    gstate->world = vx_world_create();
    gstate->layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    gstate->vxapp.display_started = eecs467_default_display_started;
    gstate->vxapp.display_finished = eecs467_default_display_finished;
    gstate->vxapp.impl = eecs467_default_implementation_create(gstate->world, gstate->layers);

    gstate->running = 1;

    // Parse arguments from the command line, showing the help
    // screen if required

    gstate->gopt = getopt_create();
    getopt_add_bool(gstate->gopt, 'h', "help", 0, "Show help");
    getopt_add_string(gstate->gopt, '\0', "url", "", "Camera URL");
	getopt_add_string(gstate->gopt, '\0', "gui-channel", "ARM_GUI", "GUI channel");
	getopt_add_string(gstate->gopt, 'm', "mode", "VIEW_MODE", "click | view");


    if (!getopt_parse(gstate->gopt, argc, argv, 1) || getopt_get_bool(gstate->gopt, "help"))
    {
        printf("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage(gstate->gopt);
        exit(1);
    }

	if(!strcmp(getopt_get_string(gstate->gopt, "mode"),"click")){
		view_above = 1;
	} else if(!strcmp(getopt_get_string(gstate->gopt, "mode"),"view")){
		view_above = 0;
	} else if(!strcmp(getopt_get_string(gstate->gopt, "mode"), "camera")){
		view_above = 2;
	} else{
		printf("Argument --mode|-m expected. Options are 'click', 'view', or 'camera'\n");
		exit(1);
	}

	lcm = lcm_create(NULL);
	gui_channel = getopt_get_string(gstate->gopt, "gui-channel");

    // Set up the imagesource. This looks for a camera url specified on
    // the command line and, if none is found, enumerates a list of all
    // cameras imagesource can find and picks the first url it fidns.
    if (strncmp(getopt_get_string(gstate->gopt, "url"), "", 1)) {
        gstate->url = strdup(getopt_get_string(gstate->gopt, "url"));
        printf("URL: %s\n", gstate->url);
    } else {
        // No URL specified. Show all available and then
        // use the first

        zarray_t *urls = image_source_enumerate();
        printf("Cameras:\n");
        for (int i = 0; i < zarray_size(urls); i++) {
            char *url;
            zarray_get(urls, i, &url);
            printf("  %3d: %s\n", i, url);
        }

        if (zarray_size(urls) == 0) {
            printf("Found no cameras.\n");
            return NULL;
        }

        zarray_get(urls, 0, &gstate->url);
    }

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your
    // display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create(&gstate->vxapp);
	
	parameter_gui_t *pg = pg_create();
	if(view_above == 2){
    	pthread_create(&gstate->animate_thread, NULL, render_loop, gstate);
		pg_add_double_slider(pg, "sl1", "Zoom Camera", 0, 800, camera_zoom);
	} else{
		pthread_create(&gstate->animate_thread, NULL, render_arm, gstate);
	}
	
    // Initialize a parameter gui
   
    //pg_add_double_slider(pg, "sl2", "Zoom Arm", 0, 20, 5);
	/*pg_add_double_slider(pg, "s1", "Rotation", -M_PI, M_PI, 0);
	pg_add_double_slider(pg, "s2", "Servo 1", -M_PI, M_PI, 0);
	pg_add_double_slider(pg, "s3", "Servo 2", -M_PI, M_PI, 0);
	pg_add_double_slider(pg, "s4", "Servo 3", -M_PI, M_PI, 0);
	pg_add_double_slider(pg, "s5", "Hand", 0, M_PI, M_PI);*/
   /* pg_add_check_boxes(pg,
                       "cb1", "Camera", 1,
                       NULL);
    pg_add_buttons(pg,
                   "but1", "Button 1",
                   "but2", "Button 2",
                   "but3", "Button 3",
                   NULL);*/

    parameter_listener_t *my_listener = calloc(1,sizeof(parameter_listener_t*));
    my_listener->impl = NULL;
    my_listener->param_changed = my_param_changed;
    pg_add_listener(pg, my_listener);
	
    gstate->pg = pg;

    eecs467_gui_run(&gstate->vxapp, gstate->pg, 800, 600);
    // Quit when GTK closes
    gstate->running = 0;

    pthread_join(gstate->animate_thread, NULL);
    vx_remote_display_source_destroy(cxn);

    // Cleanup
    //free(my_listener);

    vx_global_destroy();
    getopt_destroy(gstate->gopt);
	
	return NULL;//gstate;
}
