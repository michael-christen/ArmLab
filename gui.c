//#include "eecs467_util.h"    // This is where a lot of the internals live
#include "gui.h"

#define NUM_SERVOS 6
#define DISPLAY_H 800
#define DISPLAY_W 1000
// It's good form for every application to keep its gstate in a struct.

struct gstate
{
    char *url;
    int running;

    vx_application_t  vxapp;
    getopt_t         *gopt;
    parameter_gui_t  *pg;
	vx_display_t 	 *disp;

    vx_world_t *world;  // Where vx objects are live

    zhash_t *layers; // vx_display_t* -> vx_layer_t*

    pthread_mutex_t mutex;  // for accessing the arrays
    pthread_t animate_thread;
};

int camera_zoom = 650;
int camera_show = 1;
int arm_zoom = 25;
int view_above = 1;
double servo_positions[NUM_SERVOS];

lcm_t *lcm;
const char *gui_channel;

ball_t balls[MAX_NUM_BALLS];
int num_balls;

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
		stats.statuses[0].voltage = DISPLAY_H;
		stats.statuses[0].temperature = DISPLAY_W;
		dynamixel_status_list_t_publish(lcm, gui_channel, &stats);
		printf("clicked x:%f, clicked y:%f\n", mouse->x, mouse->y);
	}

    // Store the last mouse click
    state->x = mouse->x;
    state->y = mouse->y;

    return 0; // Returning 0 says that you have consumed the event. If the event is not consumed (return 1), then it is passed down the chain to the other event handlers.
}

static void eh_destroy(vx_event_handler_t * vh)
{
    free(vh->impl);
    free(vh);
}

// === Your code goes here ================================================
// The render loop handles your visualization updates. It is the function run
// by the animate_thread. It periodically renders the contents on the
// vx world contained by gstate
void* render_camera(void *data)
{
    //int fps = 60;
    gstate_t *gstate = data;
    int i;

	while(!zhash_size(gstate->layers)){
		usleep(1000);
	}

	vx_world_t *new_world = vx_world_create();
	vx_layer_t *layer = vx_layer_create(new_world);

	event_state_t *event_state = malloc(sizeof(event_state_t));
	vx_event_handler_t *my_event_handler = malloc(sizeof(vx_event_handler_t));
	my_event_handler->dispatch_order = 0; // Lower number here means that you get higher priority in processing events.
	my_event_handler->impl = event_state;
	my_event_handler->destroy = eh_destroy;
	my_event_handler->mouse_event = custom_mouse_event;

	zhash_iterator_t it;
	zhash_iterator_init(gstate->layers, &it);
	vx_display_t *key;
	vx_layer_t *value;

	zhash_iterator_next(&it, &key, &value);
	vx_layer_set_display(layer, key);

	zhash_put(gstate->layers, &key, &layer, NULL, NULL);

	float position[4] = {0.5f, 0.5f, 0.5f, 0.5f};
	float lowLeft[2] = {0, 0};
	float upRight[2] = {1296, 964};	//Camera dimensions

	vx_layer_set_viewport_rel(layer, position);
	vx_layer_add_event_handler(layer, my_event_handler);
	vx_layer_camera_fit2D(layer, lowLeft, upRight, 1); 

    // Set up the imagesource
	printf("test1\n");
    image_source_t *isrc = image_source_open(gstate->url);
    printf("opening camera: %s", gstate->url);
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

		num_balls = blob_detection(im, balls);
		if(num_balls) {
		    printf("%d Balls\n", num_balls);
		    for(i = 0; i < num_balls; ++i) {
			printf("X: %f, Y: %f, pxs: %d\n",
				balls[i].x, 
				balls[i].y, 
				balls[i].num_px);
		    }
		}

                if (im != NULL) {

                    vx_object_t *vim = vxo_image_from_u32(im,
                                                          VXO_IMAGE_FLIPY,
                                                          VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

                    vx_buffer_add_back(vx_world_get_buffer(new_world, "image"),
                                       vxo_chain(vxo_mat_translate3(0,0,camera_zoom),
                                                 vim));
                    vx_buffer_swap(vx_world_get_buffer(new_world, "image"));

		    /*
		    vx_object_t * vo = vxo_chain(
			    vxo_mat_translate3(state->tm_x +
				(state->template_im->width >> 1),
				state->query_im->height - state->tm_y -
				(state->template_im->width >> 1), 0),
			    vxo_mat_scale3(state->template_im->width,state->template_im->height,1),
			    vxo_box(vxo_lines_style(vx_purple, 3))
			    );
			    */
		    /*
		    vx_object_t * vo =
			vxo_chain(
				vxo_mat_scale3(500, 500, 1),
				vxo_box(vxo_lines_style(vx_purple, 3))
			);
		    vx_buffer_add_back(vx_world_get_buffer(new_world, "square"), vo);
		    vx_buffer_swap(vx_world_get_buffer(new_world, "square"));
		    */


                    image_u32_destroy(im);
                }
            }

            fflush(stdout);
            isrc->release_frame(isrc, frmd);
        }
    }

    if (isrc != NULL)
        isrc->stop(isrc);

	vx_layer_destroy(layer);
	vx_world_destroy(new_world);
    return NULL;
}

typedef struct ArmState Arm_t;
struct ArmState{
		double block1size;
		double block2size;
		double block3size;
		double block4size;
		double block5size;
		double block6size;
		double totalsize;
		double totalxshift;
		double totalzshift;
		double totalyshift;
		double totalangle;
		float color[4];
};

vx_object_t* render1(int above, Arm_t* arm){
	vx_object_t *block1 = vxo_chain(vxo_mat_rotate_x(above*M_PI/2),
					vxo_mat_translate3(arm->totalxshift, arm->block1size/2.0 + arm->totalyshift, arm->totalzshift),
					vxo_mat_rotate_y(M_PI/2.0),
					vxo_mat_scale3(3, arm->block1size, 5),
					vxo_box(vxo_mesh_style(arm->color),
					vxo_lines_style(vx_yellow, 2.0f)));

	return block1;
}

vx_object_t* render2(int above, Arm_t* arm){
	vx_object_t *block2 = vxo_chain(vxo_mat_rotate_x(above*M_PI/2),
				vxo_mat_translate3(arm->totalxshift, arm->totalsize + arm->block2size/2.0 + arm->totalyshift, arm->totalzshift),
				vxo_mat_rotate_y(servo_positions[0]),
				vxo_mat_scale3(3, arm->block2size, 5),
				vxo_box(vxo_mesh_style(arm->color),
				vxo_lines_style(vx_yellow, 2.0f)));

	arm->totalsize += arm->block2size;
	return block2;
}

vx_object_t* render3(int above, Arm_t* arm){
	vx_object_t *block3 = vxo_chain(vxo_mat_rotate_x(above*M_PI/2),
				vxo_mat_translate3(arm->totalxshift-sin(servo_positions[1])*arm->block3size/2.0*cos(servo_positions[0]), arm->totalsize + arm->block3size/2.0*cos(servo_positions[1]) + arm->totalyshift, arm->totalzshift + arm->block3size/2.0*sin(servo_positions[0])*sin(servo_positions[1])),
				vxo_mat_rotate_y(servo_positions[0]),
				vxo_mat_rotate_z(servo_positions[1]),
				vxo_mat_scale3(3, arm->block3size, 5),
				vxo_box(vxo_mesh_style(arm->color),
				vxo_lines_style(vx_yellow, 2.0f)));

	arm->totalangle += servo_positions[1];
	arm->totalxshift -= arm->block3size*sin(arm->totalangle)*cos(servo_positions[0]);
	arm->totalzshift += arm->block3size*sin(servo_positions[0])*sin(servo_positions[1]);
	arm->totalsize += arm->block3size*cos(arm->totalangle);
	return block3;
}

vx_object_t* render4(int above, Arm_t* arm){
	vx_object_t *block4 = vxo_chain(vxo_mat_rotate_x(above*M_PI/2),
				vxo_mat_translate3(arm->totalxshift -sin(arm->totalangle + servo_positions[2])*arm->block4size/2.0*cos(servo_positions[0]), arm->totalsize + arm->block4size/2.0*cos(arm->totalangle + servo_positions[2]) + arm->totalyshift, arm->totalzshift + arm->block4size/2.0*sin(servo_positions[0])*sin(arm->totalangle + servo_positions[2])),
				vxo_mat_rotate_y(servo_positions[0]),
				vxo_mat_rotate_z(arm->totalangle + servo_positions[2]),
				vxo_mat_scale3(3, arm->block4size, 5),
				vxo_box(vxo_mesh_style(arm->color),
				vxo_lines_style(vx_yellow, 2.0f)));

	arm->totalangle += servo_positions[2];
	arm->totalxshift -= arm->block4size*sin(arm->totalangle)*cos(servo_positions[0]);
	arm->totalzshift += arm->block4size*sin(servo_positions[0])*sin(arm->totalangle);
	arm->totalsize += arm->block4size*cos(arm->totalangle);
	return block4;
}

vx_object_t* render5(int above, Arm_t* arm){
	vx_object_t *block5 = vxo_chain(vxo_mat_rotate_x(above*M_PI/2),
				vxo_mat_translate3(arm->totalxshift -sin(arm->totalangle + servo_positions[3])*arm->block5size/2.0*cos(servo_positions[0]), arm->totalsize + arm->block5size/2.0*cos(arm->totalangle + servo_positions[3]) + arm->totalyshift, arm->totalzshift + arm->block5size/2.0*sin(servo_positions[0])*sin(arm->totalangle + servo_positions[3])),
				vxo_mat_rotate_y(servo_positions[0]),
				vxo_mat_rotate_z(arm->totalangle + servo_positions[3]),
				vxo_mat_scale3(3, arm->block5size, 5),
				vxo_box(vxo_mesh_style(arm->color),
				vxo_lines_style(vx_yellow, 2.0f)));

	arm->totalangle += servo_positions[3];
	arm->totalxshift -= arm->block5size*sin(arm->totalangle)*cos(servo_positions[0]);
	arm->totalzshift += arm->block5size*sin(servo_positions[0])*sin(arm->totalangle);
	arm->totalsize += arm->block5size*cos(arm->totalangle);
	return block5;
}

vx_object_t* render6(int above, Arm_t* arm){
	vx_object_t *block6 = vxo_chain(vxo_mat_rotate_x(above*M_PI/2.0),
				vxo_mat_translate3(cos(servo_positions[0])*cos(arm->totalangle) + arm->totalxshift -sin(arm->totalangle)*arm->block6size/2.0*cos(servo_positions[0]), arm->totalsize + arm->block6size/2.0*cos(arm->totalangle) + sin(arm->totalangle) + arm->totalyshift, arm->totalzshift - sin(servo_positions[0])*cos(arm->totalangle) + arm->block6size/2.0*sin(servo_positions[0])*sin(arm->totalangle)),
				vxo_mat_rotate_y(servo_positions[0]),//+servo_positions[4]*cos(arm->totalangle)),
				//vxo_mat_rotate_x(servo_positions[4]*sin(arm->totalangle)),
				vxo_mat_rotate_z(arm->totalangle),
				vxo_mat_scale3(1, arm->block6size, 5),
				vxo_box(vxo_mesh_style(arm->color),
				vxo_lines_style(vx_yellow, 2.0f)));
	return block6;
}

vx_object_t* renderClaw(int above, Arm_t* arm){
	vx_object_t *claw = vxo_chain(vxo_mat_rotate_x(above*M_PI/2.0),
				vxo_mat_translate3(-cos(servo_positions[0])*cos(arm->totalangle) + arm->totalxshift - cos(arm->totalangle-servo_positions[5])*arm->block6size/2.0*cos(servo_positions[0]), arm->totalsize + arm->block6size/2.0*cos(arm->totalangle - servo_positions[5] + M_PI/2.0) - sin(arm->totalangle) + arm->totalyshift, arm->totalzshift + sin(servo_positions[0])*cos(arm->totalangle) + arm->block6size/2.0*sin(servo_positions[0])*sin(arm->totalangle - servo_positions[5] + M_PI/2.0)),
				vxo_mat_rotate_y(servo_positions[0]),//+servo_positions[4]*cos(arm->totalangle)),
				//vxo_mat_rotate_x(servo_positions[4]*sin(arm->totalangle)),
				vxo_mat_rotate_z(arm->totalangle - servo_positions[5]),
				vxo_mat_scale3(arm->block6size, 1, 5),
				vxo_box(vxo_mesh_style(arm->color),
				vxo_lines_style(vx_yellow, 2.0f)));
	return claw;
}

void renderBalls(int above, vx_world_t* world) {
    num_balls = 2;
    ball_t currentBall;
    vx_object_t *ball;
    int i;
    for(i=0; i < num_balls; ++i) {
	currentBall.x = 5 + i*5; 
	currentBall.y = 5;
	currentBall.num_px = 150;
	balls[i] = currentBall; 
	ball = vxo_chain(
		vxo_mat_scale3(3,3,3),
		vxo_mat_translate3(balls[i].x,balls[i].y,0),
		vxo_sphere(vxo_mesh_style(vx_yellow))
	);
	vx_buffer_add_back(vx_world_get_buffer(world, "balls"), ball);	
    }

    vx_buffer_swap(vx_world_get_buffer(world, "balls"));
}

void render_elements(int above, vx_world_t* world){
	Arm_t* arm = calloc(1, sizeof(Arm_t));;

	arm->block1size = 7;
	arm->block2size = 4;
	arm->block3size = 10;
	arm->block4size = 10;
	arm->block5size = 10;
	arm->block6size = 8;
	arm->color[0] = 0.0f;
	arm->color[1] = 0.0f;
	arm->color[2] = 1.0f;
	arm->color[3] = 1.0f;
	arm->totalsize = arm->block1size;
	arm->totalxshift = 0;
	arm->totalzshift = arm_zoom;
	arm->totalyshift = 0;
	arm->totalangle = 0;	

	if(above){
		arm->totalzshift -= 25;
	}else{
		arm->totalyshift = -18;
	}

	/*if(above){
		vx_buffer_add_back(vx_world_get_buffer(world, "block1"), vxo_pix_coords(VX_ORIGIN_CENTER, render1(above, arm)));
		vx_buffer_add_back(vx_world_get_buffer(world, "block2"), vxo_pix_coords(VX_ORIGIN_CENTER, render2(above, arm)));
		vx_buffer_add_back(vx_world_get_buffer(world, "block3"), vxo_pix_coords(VX_ORIGIN_CENTER, render3(above, arm)));
		vx_buffer_add_back(vx_world_get_buffer(world, "block4"), vxo_pix_coords(VX_ORIGIN_CENTER, render4(above, arm)));
		vx_buffer_add_back(vx_world_get_buffer(world, "block5"), vxo_pix_coords(VX_ORIGIN_CENTER, render5(above, arm)));
		vx_buffer_add_back(vx_world_get_buffer(world, "block6"), vxo_pix_coords(VX_ORIGIN_CENTER, render6(above, arm)));
		vx_buffer_add_back(vx_world_get_buffer(world, "claw"), vxo_pix_coords(VX_ORIGIN_CENTER, renderClaw(above, arm)));	
	}else{*/
		vx_buffer_add_back(vx_world_get_buffer(world, "block1"), render1(above, arm));
		vx_buffer_add_back(vx_world_get_buffer(world, "block2"), render2(above, arm));
		vx_buffer_add_back(vx_world_get_buffer(world, "block3"), render3(above, arm));
		vx_buffer_add_back(vx_world_get_buffer(world, "block4"), render4(above, arm));
		vx_buffer_add_back(vx_world_get_buffer(world, "block5"), render5(above, arm));
		vx_buffer_add_back(vx_world_get_buffer(world, "block6"), render6(above, arm));
		vx_buffer_add_back(vx_world_get_buffer(world, "claw"), renderClaw(above, arm));	
	//}

	vx_buffer_swap(vx_world_get_buffer(world, "block1"));
	vx_buffer_swap(vx_world_get_buffer(world, "block2"));
	vx_buffer_swap(vx_world_get_buffer(world, "block3"));
	vx_buffer_swap(vx_world_get_buffer(world, "block4"));
	vx_buffer_swap(vx_world_get_buffer(world, "block5"));
	vx_buffer_swap(vx_world_get_buffer(world, "block6"));
	vx_buffer_swap(vx_world_get_buffer(world, "claw"));
	renderBalls(above, world);
}

void* render_above(void* data){
	gstate_t *gstate = data;

	while(!zhash_size(gstate->layers)){
		usleep(1000);
	}

	event_state_t *event_state = malloc(sizeof(event_state_t));
	vx_event_handler_t *my_event_handler = malloc(sizeof(vx_event_handler_t));
	my_event_handler->dispatch_order = 0; // Lower number here means that you get higher priority in processing events.
	my_event_handler->impl = event_state;
	my_event_handler->destroy = eh_destroy;
	my_event_handler->mouse_event = custom_mouse_event;

	vx_world_t *new_world = vx_world_create();
	vx_layer_t *layer = vx_layer_create(new_world);
	
	zhash_iterator_t it;
	zhash_iterator_init(gstate->layers, &it);
	vx_display_t *key;
	vx_layer_t *value;

	zhash_iterator_next(&it, &key, &value);
	vx_layer_set_display(layer, key);

	zhash_put(gstate->layers, &key, &layer, NULL, NULL);

	float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
	float position[4] = {0.0f, 0.5f, 0.5f, 0.5f};

	//while(zhash_iterator_next(&it, &key, &value)){
		vx_layer_set_background_color(layer, black);
		vx_layer_set_viewport_rel(layer, position);
		vx_layer_add_event_handler(layer, my_event_handler);
	//}

	/*vx_buffer_add_back(vx_world_get_buffer(gstate->world, "grid"),  vxo_chain(vxo_mat_translate3(0,0,-2*arm_zoom),
		vxo_mat_scale3(1.5, 1.5, 1),
		vxo_grid()));
	vx_buffer_swap(vx_world_get_buffer(gstate->world, "grid"));*/

	while(gstate->running){
		render_elements(1, new_world);

		vx_buffer_add_back(vx_world_get_buffer(new_world, "line"),
			vxo_pix_coords(VX_ORIGIN_BOTTOM,
			vxo_chain(vxo_mat_scale3(DISPLAY_W/2.0, 1, 1),
			vxo_rect(vxo_mesh_style(vx_green),
			vxo_lines_style(vx_green, 2.0f),
			vxo_points_style(vx_green, 2.0f)))));
		vx_buffer_swap(vx_world_get_buffer(new_world, "line"));

		vx_buffer_add_back(vx_world_get_buffer(new_world, "crossx"),
			vxo_pix_coords(VX_ORIGIN_CENTER,
			vxo_chain(vxo_mat_translate3(0, 0, -1),
			vxo_mat_scale3(DISPLAY_W/2.0, 1, 1),
			vxo_rect(vxo_mesh_style(vx_red),
			vxo_lines_style(vx_red, 2.0f),
			vxo_points_style(vx_red, 2.0f)))));
		vx_buffer_swap(vx_world_get_buffer(new_world, "crossx"));

		vx_buffer_add_back(vx_world_get_buffer(new_world, "crossy"),
			vxo_pix_coords(VX_ORIGIN_CENTER,
			vxo_chain(vxo_mat_translate3(0, 0, -1),
			vxo_mat_scale3(1, DISPLAY_H/2.0, 1),
			vxo_rect(vxo_mesh_style(vx_red),
			vxo_lines_style(vx_red, 2.0f),
			vxo_points_style(vx_red, 2.0f)))));
		vx_buffer_swap(vx_world_get_buffer(new_world, "crossy"));

		usleep(25000);
	}
	vx_layer_destroy(layer);
	vx_world_destroy(new_world);
	return NULL;
}

void* render_view(void* data){
	gstate_t *gstate = data;

	while(!zhash_size(gstate->layers)){
		usleep(1000);
	}

	vx_world_t *new_world = vx_world_create();
	vx_layer_t *layer = vx_layer_create(new_world);

	zhash_iterator_t it;
	zhash_iterator_init(gstate->layers, &it);
	vx_display_t *key;
	vx_layer_t *value;

	zhash_iterator_next(&it, &key, &value);
	vx_layer_set_display(layer, key);
	zhash_put(gstate->layers, &key, &layer, NULL, NULL);

	float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
	float position[4] = {0.0f, 0.0f, 0.5f, 0.5f};

	vx_layer_set_background_color(layer, black);
	vx_layer_set_viewport_rel(layer, position);

	/*vx_buffer_add_back(vx_world_get_buffer(gstate->world, "grid"),  vxo_chain(vxo_mat_translate3(0,0,-2*arm_zoom),
		vxo_mat_scale3(1.5, 1.5, 1),
		vxo_grid()));
	vx_buffer_swap(vx_world_get_buffer(gstate->world, "grid"));*/

	while(gstate->running){
		render_elements(0, new_world);
		
		vx_buffer_add_back(vx_world_get_buffer(new_world, "line"),
			vxo_pix_coords(VX_ORIGIN_TOP,
			vxo_chain(vxo_mat_scale3(DISPLAY_W/2.0, 1, 1),
			vxo_rect(vxo_mesh_style(vx_green),
			vxo_lines_style(vx_green, 2.0f),
			vxo_points_style(vx_green, 2.0f)))));
		vx_buffer_swap(vx_world_get_buffer(new_world, "line"));

		usleep(25000);
	}
	vx_layer_destroy(layer);
	vx_world_destroy(new_world);
	return NULL;
}

void* render_status(void* data){
	gstate_t *gstate = data;

	while(!zhash_size(gstate->layers)){
		usleep(1000);
	}

	vx_world_t *new_world = vx_world_create();
	vx_layer_t *layer = vx_layer_create(new_world);

	zhash_iterator_t it;
	zhash_iterator_init(gstate->layers, &it);
	vx_display_t *key;
	vx_layer_t *value;

	zhash_iterator_next(&it, &key, &value);
	vx_layer_set_display(layer, key);
	zhash_put(gstate->layers, &key, &layer, NULL, NULL);

	float position[4] = {0.5f, 0.0f, 0.5f, 0.5f};

	vx_layer_set_viewport_rel(layer, position);

	while(gstate->running){		
		char statusText[128] = "";
		
		sprintf(statusText, " Servo angles:\n 0: [%f]\n 1: [%f]\n 2: [%f]\n 3: [%f]\n 4: [%f]\n 5: [%f]", servo_positions[0]-M_PI, servo_positions[1], servo_positions[2], servo_positions[3], servo_positions[4], servo_positions[5]);
		
		vx_object_t* text = vxo_text_create(VXO_TEXT_ANCHOR_TOP_LEFT, statusText);
		vx_buffer_add_back(vx_world_get_buffer(new_world, "text"), vxo_pix_coords(VX_ORIGIN_TOP_LEFT, text));
		vx_buffer_swap(vx_world_get_buffer(new_world, "text"));
		usleep(25000);
	}
	vx_layer_destroy(layer);
	vx_world_destroy(new_world);
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

	/*if(!strcmp(getopt_get_string(gstate->gopt, "mode"),"click")){
		view_above = 1;
	} else if(!strcmp(getopt_get_string(gstate->gopt, "mode"),"view")){
		view_above = 0;
	} else if(!strcmp(getopt_get_string(gstate->gopt, "mode"), "camera")){
		view_above = 2;
	} else{
		printf("Argument --mode|-m expected. Options are 'click', 'view', or 'camera'\n");
		exit(1);
	}*/

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
	if(zarray_size(urls) >= 3) {
	    zarray_get(urls, 2, &gstate->url);
	}
	else {
	    zarray_get(urls, 0, &gstate->url);
	}

    }

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your
    // display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create(&gstate->vxapp);
	
	parameter_gui_t *pg = pg_create();

	pg_add_double_slider(pg, "sl1", "Zoom Camera", 0, 800, camera_zoom);
	pthread_create(&gstate->animate_thread, NULL, render_camera, gstate);
	pthread_create(&gstate->animate_thread, NULL, render_view, gstate);
	pthread_create(&gstate->animate_thread, NULL, render_above, gstate);
	pthread_create(&gstate->animate_thread, NULL, render_status, gstate);

	
	
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

    eecs467_gui_run(&gstate->vxapp, gstate->pg, DISPLAY_W, DISPLAY_H);
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
