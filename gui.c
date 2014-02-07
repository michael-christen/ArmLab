//#include "eecs467_util.h"    // This is where a lot of the internals live
#include "gui.h"

#define NUM_SERVOS 6
#define MAX_RADIUS 34
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
const char *ball_channel;
const char *action_channel;

ball_t balls[MAX_NUM_BALLS];
int num_balls;
pthread_mutex_t ball_mutex;

//These are variables for the equation
// x = ax + by + c, y = dx + ey + f
double scalingFactors[6] = {1,0,0,0,1,0};
double iscalingFactors[6] = {1,0,0,0,1,0};
//Pixel samples
//px, py, 1
int calibrated = 0;
double samples[3*NUM_SAMPLES_FOR_ISCALING];
double isamples[2*NUM_SAMPLES_FOR_ISCALING];
int numSamples = 0;
pthread_mutex_t scaling_mutex;
pthread_cond_t scaling_cv;

int calib_cam = 0;	//Calibrating upper left quadrant
double calib_positions[NUM_SERVOS] = {0, M_PI/2.0, 0, 0, 0, M_PI/2.0};

volatile int getting_balls;
clock_t ball_clock;

void* initScalingFactors(void *data) {
    int i;
    //X, Y positions for calibration
   
    double positions[2*NUM_SAMPLES_FOR_ISCALING] = {
	-15.1, -14.9,
	0.2,   -15.3,
	14.9,  -15.1,
	15.3,  0.6,
	15.3,  15,
	-0.1,  15.3,
	-14.9, 15.3,
	-15.0, -0.2 
    };
    
    double ipositions[3*NUM_SAMPLES_FOR_ISCALING] = {
	-15.1, -14.9, 1.0,
	0.2,   -15.3, 1.0,
	14.9,  -15.1, 1.0,
	15.3,  0.6, 1.0,
	15.3,  15, 1.0,
	-0.1,  15.3, 1.0,
	-14.9, 15.3, 1.0,
	-15.0, -0.2,  1.0
    };
    pthread_mutex_lock(&scaling_mutex);

    while(numSamples < NUM_SAMPLES_FOR_ISCALING) {
	pthread_cond_wait(&scaling_cv, &scaling_mutex);
    }

    printf("\nInitializing Scaling Factors\n");
    //px, py, 1, 0,   0, 0
    //0,  0 , 0, px, py, 1
    //Samples
    matd_t *sample_mat = matd_create_data(NUM_SAMPLES_FOR_ISCALING, 3, samples);
    matd_t *isample_mat = matd_create_data(NUM_SAMPLES_FOR_ISCALING, 2, isamples);

    pthread_mutex_unlock(&scaling_mutex);
    //Positions
    matd_t *pos_mat = matd_create_data(NUM_SAMPLES_FOR_ISCALING, 2, positions);
    matd_t *ipos_mat = matd_create_data(NUM_SAMPLES_FOR_ISCALING, 3, ipositions);
    //x = inv(trans(A)*A)*trans(A)*b
    matd_t * sol_mat = matd_op("(M' * M)^-1 * M' * M", sample_mat,sample_mat,sample_mat, pos_mat);
    matd_t * isol_mat = matd_op("(M' * M)^-1 * M' * M", ipos_mat, ipos_mat, ipos_mat, isample_mat);

    matd_t * solved_mat = matd_multiply(sample_mat, sol_mat);

/*
    printf("Inverted solution matrix\n");
    matd_print(isol_mat, " %lf ");

    printf("Inverted Position matrix\n");
    matd_print(ipos_mat, " %lf ");

    printf("Inverted Sample matrix\n");
    matd_print(isample_mat, " %lf ");
*/

    printf("Sample matrix\n");
    matd_print(sample_mat, " %lf ");

    printf("Solution matrix\n");
    matd_print(sol_mat, " %lf ");

    printf("Position matrix\n");
    matd_print(pos_mat, " %lf ");

    printf("Check matrix\n");
    matd_print(solved_mat, " %lf ");


    for(i = 0; i < 6; ++i) {
	scalingFactors[i] = matd_get(sol_mat,i%3,i/3);
	iscalingFactors[i] = matd_get(isol_mat, i%3, i/3);
	printf("Scaling factor (%d) = %f\n",i, scalingFactors[i]);
    }
    calibrated = 1;

    //Clean up
    matd_destroy(sample_mat);
    matd_destroy(pos_mat);
    matd_destroy(sol_mat);
    matd_destroy(isample_mat);
    matd_destroy(ipos_mat);
    matd_destroy(isol_mat);
    matd_destroy(solved_mat);
    printf("Scaling Factors initialized\n");
    return NULL;
}

void stop_getting_balls(){
    getting_balls = 0;
}

void gui_update_servo_pos(double servo_pos[]){
	int i;
	for(i = 0; i < NUM_SERVOS ;i++){
		servo_positions[i] = servo_pos[i];
	}

	servo_positions[0] += M_PI;
}

// === Parameter listener =================================================
// This function is handled to the parameter gui (via a parameter listener)
// and handles events coming from the parameter gui. The parameter listener
// also holds a void* pointer to "impl", which can point to a struct holding
// gstate, etc if need be.
void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    /*if (!strcmp("s0", name)) {
       servo_positions[0] = pg_gi(pg, name);//camera_zoom = pg_gi(pg, name);
    } else if (!strcmp("s1", name)) {
        servo_positions[1] = pg_gi(pg, name);//arm_zoom = pg_gi(pg, name);
	} else if (!strcmp("s2", name)) {
       servo_positions[2] = pg_gi(pg, name);
    } else if (!strcmp("s3", name)) {
       servo_positions[3] = pg_gi(pg, name);
    } else if (!strcmp("s4", name)) {
       servo_positions[4] = pg_gi(pg, name);
    } else if (!strcmp("s5", name)) {
       servo_positions[5] = pg_gi(pg, name);*/
	if (!strcmp("sl1", name)) {
       camera_zoom = pg_gi(pg, name);
    } else if (!strcmp("sl2", name)) {
       arm_zoom = pg_gi(pg, name);
    } else if (!strcmp("but1", name)){
		//camera_show = pg_gb(pg, name);
		calib_cam = ~calib_cam;
	} else if (!strcmp("but2", name)){
		arm_action_t arm_action;
	    arm_action.getBalls = 1;
	    arm_action.goToHome = 0;
	    arm_action.x = arm_action.y = arm_action.goToPoint = 0;   
	    arm_action_t_publish(lcm, action_channel, &arm_action);
	    getting_balls = 1;
	    ball_clock = clock();
	} else if (!strcmp("but3", name)){
		arm_action_t arm_action;
	    arm_action.getBalls = 0;
	    arm_action.goToHome = 0;
	    arm_action.x = arm_action.y = arm_action.goToPoint = 0;    
	    arm_action_t_publish(lcm, action_channel, &arm_action);
	    getting_balls = 0;
	} else if (!strcmp("but4", name)){
		arm_action_t arm_action;
	    arm_action.getBalls = 0;
	    arm_action.goToHome = 1;
	    arm_action.x = arm_action.y = arm_action.goToPoint = 0;    
	    arm_action_t_publish(lcm, action_channel, &arm_action);
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

static int camera_mouse_event(vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
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
    double man_point[3];
	if(button_up){
		vx_ray3_t ray;
		vx_camera_pos_compute_ray(pos, mouse->x, mouse->y, &ray);
		vx_ray3_intersect_xy(&ray, camera_zoom, man_point);
		pthread_mutex_lock(&scaling_mutex);
		//Clicked in camera area
		//Not valid
		man_point[1] = 964 - man_point[1];
		printf("realx: %f, realy: %f\n", mouse->x, mouse->y);
		printf("x: %f, y: %f, z: %f\n", man_point[0], man_point[1], man_point[2]);
		if(calib_cam && numSamples < NUM_SAMPLES_FOR_ISCALING) {
		    //???Need to convert this to picture pixels 
		    samples[numSamples*3] = man_point[0];
		    samples[numSamples*3+1] = man_point[1];
		    samples[numSamples*3+2] = 1;
		    isamples[numSamples*2] = man_point[0];
		    isamples[numSamples*2 + 1] = 964 - man_point[1];
		    if(++numSamples >= NUM_SAMPLES_FOR_ISCALING) {
			pthread_cond_signal(&scaling_cv);
		    }
		    printf("NumSamples: %d\n", numSamples);
		}
		pthread_mutex_unlock(&scaling_mutex);
	}

    // Store the last mouse click
    state->x = mouse->x;
    state->y = mouse->y;

    return 0; // Returning 0 says that you have consumed the event. If the event is not consumed (return 1), then it is passed down the chain to the other event handlers.
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
    double man_point[3];
	if(button_up){
		vx_ray3_t ray;
		vx_camera_pos_compute_ray(pos, mouse->x, mouse->y, &ray);
		vx_ray3_intersect_xy(&ray, 0.0, man_point);
		printf("x: %f, man_x: %f\n", mouse->x, man_point[0]);
		printf("y: %f, man_y: %f\n", mouse->y, man_point[1]);
		arm_action_t arm_action;
	    arm_action.getBalls = arm_action.goToHome = 0;
	    arm_action.goToPoint = 1;
	    arm_action.x = man_point[0];
	    arm_action.y = man_point[1];
	    arm_action_t_publish(lcm, action_channel, &arm_action);
		pthread_mutex_lock(&scaling_mutex);
		//Clicked in camera area
		//Not valid
		if(calib_cam && mouse->x > 500 && mouse->y > 380 && numSamples < NUM_SAMPLES_FOR_ISCALING) {
		    //???Need to convert this to picture pixels 
		    samples[numSamples*3] = mouse->x;
		    samples[numSamples*3+1] = mouse->y;
		    samples[numSamples*3+2] = 1;
		    if(++numSamples >= NUM_SAMPLES_FOR_ISCALING) {
			pthread_cond_signal(&scaling_cv);
		    }
		    printf("NumSamples: %d", numSamples);
		}
		pthread_mutex_unlock(&scaling_mutex);
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

void transform_balls() {
    int i;
    ball_t temp_b;
    for(i = 0; i < num_balls; ++i) {
	temp_b.x = 
	    scalingFactors[0]*balls[i].x +
	    scalingFactors[1]*balls[i].y +
	    scalingFactors[2];
	temp_b.y = 
	    scalingFactors[3]*balls[i].x +
	    scalingFactors[4]*balls[i].y +
	    scalingFactors[5];
      //  printf("Temp.x: %f, Ball.x: %f\n",temp_b.x, balls[i].x);
       // printf("Temp.y: %f, Ball.y: %f\n",temp_b.y, balls[i].y);
        balls[i] = temp_b;
    }
}

// === Your code goes here ================================================
// The render loop handles your visualization updates. It is the function run
// by the animate_thread. It periodically renders the contents on the
// vx world contained by gstate
void* render_camera(void *data)
{
    //int fps = 60;
    gstate_t *gstate = data;

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
	my_event_handler->mouse_event = camera_mouse_event;

	zhash_iterator_t it;
	zhash_iterator_init(gstate->layers, &it);
	vx_display_t *key;
	vx_layer_t *value;

	zhash_iterator_next(&it, &key, &value);
	vx_layer_set_display(layer, key);

	zhash_put(gstate->layers, &key, &layer, NULL, NULL);
	int camera_width = 1296;
	int camera_height = 964;

	float position[4] = {0.5f, 0.5f, 0.5f, 0.5f};
	float lowLeft[2] = {0, 0};
	float upRight[2] = {camera_width, camera_height};	//Camera dimensions

	vx_layer_set_viewport_rel(layer, position);
	vx_layer_add_event_handler(layer, my_event_handler);
	vx_layer_camera_fit2D(layer, lowLeft, upRight, 1); 

    // Set up the imagesource
    image_source_t *isrc = image_source_open(gstate->url);
    printf("opening camera: %s", gstate->url);
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

		//pthread_mutex_lock(&ball_mutex);
		num_balls = blob_detection(im, balls);
		transform_balls();
		//pthread_mutex_unlock(&ball_mutex);
		
		if(num_balls) {
		    ball_list_t ball_list;
		    ball_list.len = num_balls;
		    ball_list.balls = malloc(sizeof(ball_info_t) * num_balls);

		    for(int i = 0; i < num_balls; ++i) {
				ball_list.balls[i].utime = utime_now();		    
				ball_list.balls[i].x = balls[i].x;
				ball_list.balls[i].y = balls[i].y; 
				ball_list.balls[i].num_pxs = balls[i].num_px;
		    }
		    
		    ball_list_t_publish(lcm, ball_channel, &ball_list);
		    free(ball_list.balls);
		}
		

                if (im != NULL) {

		    /*
                    vx_object_t *vim = vxo_image_from_u32(im,
                                                          VXO_IMAGE_FLIPY,
                                                          VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);
							  */
                    vx_object_t *vim = vxo_image_from_u32(im,
                                                          VXO_IMAGE_FLIPY,
                                                          VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

                    vx_buffer_add_back(vx_world_get_buffer(new_world, "image"),
                                       vxo_chain(vxo_mat_translate3(0,0,camera_zoom),
                                                 vim));
                    vx_buffer_swap(vx_world_get_buffer(new_world, "image"));

		    if(calibrated){
			    double xaxisx0 = iscalingFactors[0] * 31 + iscalingFactors[2];
			    double xaxisy0 = iscalingFactors[3] * 31 + iscalingFactors[5];
			    double xaxisx1 = iscalingFactors[0] * -31 + iscalingFactors[2];
			    double xaxisy1 = iscalingFactors[3] * -31 + iscalingFactors[5];
			    double yaxisx0 = iscalingFactors[1] * 31 + iscalingFactors[2];
			    double yaxisy0 = iscalingFactors[4] * 31 + iscalingFactors[5];
			    double yaxisx1 = iscalingFactors[1] * -31 + iscalingFactors[2];
			    double yaxisy1 = iscalingFactors[4] * -31 + iscalingFactors[5];
			    float points [12] = {xaxisx0, xaxisy0, camera_zoom+1, xaxisx1, xaxisy1, camera_zoom+1,
				yaxisx0, yaxisy0, camera_zoom+1, yaxisx1, yaxisy1, camera_zoom+1};
			    int npoints = 4;
			    vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
			    vx_buffer_add_back(vx_world_get_buffer(new_world, "lines"),
				vxo_lines(verts, npoints, GL_LINES, vxo_points_style(vx_red, 3.0f)));
			    vx_buffer_swap(vx_world_get_buffer(new_world, "lines"));
		    }

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
					vxo_mat_translate3(arm->totalxshift,
					arm->block1size/2.0 + arm->totalyshift,
					arm->totalzshift),
					vxo_mat_rotate_y(M_PI/2.0),
					vxo_mat_scale3(3, arm->block1size, 5),
					vxo_box(vxo_mesh_style(arm->color),
					vxo_lines_style(vx_yellow, 2.0f)));

	return block1;
}

vx_object_t* render2(int above, Arm_t* arm){
	vx_object_t *block2 = vxo_chain(vxo_mat_rotate_x(above*M_PI/2),
				vxo_mat_translate3(arm->totalxshift, 
				arm->totalsize + arm->block2size/2.0 + arm->totalyshift,
				arm->totalzshift),
				vxo_mat_rotate_y(servo_positions[0]),
				vxo_mat_scale3(3, arm->block2size, 5),
				vxo_box(vxo_mesh_style(arm->color),
				vxo_lines_style(vx_yellow, 2.0f)));

	arm->totalsize += arm->block2size;
	return block2;
}

vx_object_t* render3(int above, Arm_t* arm){
	vx_object_t *block3 = vxo_chain(vxo_mat_rotate_x(above*M_PI/2),
				vxo_mat_translate3(arm->totalxshift-sin(servo_positions[1])*arm->block3size/2.0*cos(servo_positions[0]),
				arm->totalsize + arm->block3size/2.0*cos(servo_positions[1]) + arm->totalyshift,
				arm->totalzshift + arm->block3size/2.0*sin(servo_positions[0])*sin(servo_positions[1])),
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
				vxo_mat_translate3(arm->totalxshift -
				sin(arm->totalangle + servo_positions[2])*arm->block4size/2.0*cos(servo_positions[0]),
				arm->totalsize + arm->block4size/2.0*cos(arm->totalangle + servo_positions[2]) + arm->totalyshift,
				arm->totalzshift + arm->block4size/2.0*sin(servo_positions[0])*sin(arm->totalangle + servo_positions[2])),
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
				vxo_mat_translate3(arm->totalxshift -
				sin(arm->totalangle + servo_positions[3])*arm->block5size/2.0*cos(servo_positions[0]),
				arm->totalsize + arm->block5size/2.0*cos(arm->totalangle + servo_positions[3]) + arm->totalyshift,
				arm->totalzshift + arm->block5size/2.0*sin(servo_positions[0])*sin(arm->totalangle + servo_positions[3])),
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
				vxo_mat_translate3(
				//X
				cos(servo_positions[0])*cos(arm->totalangle) + 
				arm->totalxshift - sin(arm->totalangle)*arm->block6size/2.0*cos(servo_positions[0]),
				//Y
				arm->totalsize + arm->block6size/2.0*cos(arm->totalangle) + 
				sin(arm->totalangle) + arm->totalyshift,
				//Z
				arm->totalzshift - sin(servo_positions[0])*cos(arm->totalangle) + 
				arm->block6size/2.0*sin(servo_positions[0])*sin(arm->totalangle)),
				//ROTATE
				vxo_mat_rotate_y(servo_positions[0]),// - servo_positions[4]*cos(arm->totalangle)),
				//vxo_mat_rotate_x(servo_positions[4]*sin(arm->totalangle)),
				vxo_mat_rotate_z(arm->totalangle),
				vxo_mat_scale3(1, arm->block6size, 5),
				vxo_box(vxo_mesh_style(arm->color),
				vxo_lines_style(vx_yellow, 2.0f)));
	return block6;
}

vx_object_t* renderClaw(int above, Arm_t* arm){
	vx_object_t *claw = vxo_chain(vxo_mat_rotate_x(above*M_PI/2.0),
				vxo_mat_translate3(
				//X
				-cos(servo_positions[0])*cos(arm->totalangle) + 
				arm->totalxshift - cos(arm->totalangle-servo_positions[5])*arm->block6size/2.0*cos(servo_positions[0]),
				//Y
				arm->totalsize + arm->block6size/2.0*cos(arm->totalangle - servo_positions[5] + M_PI/2.0) - 
				sin(arm->totalangle) + arm->totalyshift,
				//Z
				arm->totalzshift + sin(servo_positions[0])*cos(arm->totalangle) + 
				arm->block6size/2.0*sin(servo_positions[0])*sin(arm->totalangle - servo_positions[5] + M_PI/2.0)),
				//ROTATE
				vxo_mat_rotate_y(servo_positions[0]),//+servo_positions[4]*cos(arm->totalangle)),
				//vxo_mat_rotate_x(servo_positions[4]*sin(arm->totalangle)),
				vxo_mat_rotate_z(arm->totalangle - servo_positions[5]),
				vxo_mat_scale3(arm->block6size, 1, 5),
				vxo_box(vxo_mesh_style(arm->color),
				vxo_lines_style(vx_yellow, 2.0f)));
	return claw;
}

void renderBalls(int above, vx_world_t* world) {
    //pthread_mutex_unlock(&ball_mutex);
    ball_t currentBall;
    vx_object_t *ball;
    int i;
    for(i=0; i < num_balls; ++i) {
	/*
		currentBall.x = 5 + i*5; 
		currentBall.y = 5;
		currentBall.num_px = 150;
		balls[i] = currentBall; 
		*/    
		ball = vxo_chain(
			vxo_mat_rotate_x(above*M_PI/2.0),
			vxo_mat_translate3(balls[i].x,-15*!above+3*above,balls[i].y+25*!above),
			vxo_mat_scale3(3,3,3),
			vxo_sphere(vxo_mesh_style(vx_yellow))
		);
		vx_buffer_add_back(vx_world_get_buffer(world, "balls"), ball);	
    }
    //pthread_mutex_unlock(&ball_mutex);

    vx_buffer_swap(vx_world_get_buffer(world, "balls"));
}

void render_elements(int above, vx_world_t* world){
	Arm_t* arm = calloc(1, sizeof(Arm_t));;

	arm->block1size = 7;
	arm->block2size = 4.5;
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

	vx_buffer_add_back(vx_world_get_buffer(world, "block1"), render1(above, arm));
	vx_buffer_add_back(vx_world_get_buffer(world, "block2"), render2(above, arm));
	vx_buffer_add_back(vx_world_get_buffer(world, "block3"), render3(above, arm));
	vx_buffer_add_back(vx_world_get_buffer(world, "block4"), render4(above, arm));
	vx_buffer_add_back(vx_world_get_buffer(world, "block5"), render5(above, arm));
	vx_buffer_add_back(vx_world_get_buffer(world, "block6"), render6(above, arm));
	vx_buffer_add_back(vx_world_get_buffer(world, "claw"), renderClaw(above, arm));	

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
	//float transparent[4] = {0.0f, 0.0f, 0.0f, 0.0f};

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

		vx_buffer_add_back(vx_world_get_buffer(new_world, "crossx"),
			vxo_chain(vxo_mat_translate3(0, 0, .1),
			vxo_mat_scale3(61, 1, 1),
			vxo_rect(vxo_mesh_style(vx_red),
			vxo_lines_style(vx_red, 1.0f),
			vxo_points_style(vx_red, 1.0f))));
		vx_buffer_swap(vx_world_get_buffer(new_world, "crossx"));

		vx_buffer_add_back(vx_world_get_buffer(new_world, "crossy"),
			vxo_pix_coords(VX_ORIGIN_CENTER,
			vxo_chain(vxo_mat_translate3(0, 0, -1),
			vxo_mat_scale3(1, DISPLAY_H/2.0, 1),
			vxo_rect(vxo_mesh_style(vx_red),
			vxo_lines_style(vx_red, 2.0f),
			vxo_points_style(vx_red, 2.0f)))));

		vx_buffer_add_back(vx_world_get_buffer(new_world, "crossy"),
			vxo_chain(vxo_mat_translate3(0, 0, .1),
			vxo_mat_scale3(1, 61, 1),
			vxo_rect(vxo_mesh_style(vx_red),
			vxo_lines_style(vx_red, 1.0f),
			vxo_points_style(vx_red, 1.0f))));
		vx_buffer_swap(vx_world_get_buffer(new_world, "crossy"));

		
		vx_buffer_add_back(vx_world_get_buffer(new_world, "board"), 
			vxo_chain(vxo_mat_translate3(0,0,0),
			vxo_mat_scale3(61, 61, 1),
			vxo_rect(vxo_mesh_style(vx_gray),
			vxo_lines_style(vx_gray, 2.0f),
			vxo_points_style(vx_gray, 2.0f))));
		vx_buffer_swap(vx_world_get_buffer(new_world, "board"));

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

	/*default_cam_mgr_t* dcm = default_cam_mgr_create();
	uint64_t* mtime = 0;
	vx_camera_pos_t* pos = default_cam_mgr_get_cam_target(dcm, mtime);
	double pixelsToRadians = M_PI /  (pos->viewport[2] >  pos->viewport[3] ? pos->viewport[2] : pos->viewport[3]);

	zarray_t * fp = zarray_create(sizeof(matd_t*));
    matd_t * eye = matd_create_data(3,1, pos->eye);    zarray_add(fp, &eye);
    matd_t * lookat = matd_create_data(3,1, pos->lookat); zarray_add(fp, &lookat);
    matd_t * up = matd_create_data(3,1, pos->up);     zarray_add(fp, &up);

	matd_t *qx = matd_create(4,1); zarray_add(fp, &qx);
    matd_t *qy = matd_create(4,1); zarray_add(fp, &qy);

    matd_t * p2eye = matd_subtract(eye, lookat); zarray_add(fp, &p2eye);
    matd_t * left = matd_crossproduct(p2eye,up); zarray_add(fp, &left);

    vx_util_angle_axis_to_quat(-50*pixelsToRadians, up->data, qx->data);
    vx_util_angle_axis_to_quat(-50*pixelsToRadians, left->data, qy->data);

    matd_t * qcum = matd_create(4,1); zarray_add(fp, &qcum);
    vx_util_quat_multiply(qx->data,qy->data, qcum->data);

    // apply rotation
    default_cam_mgr_rotate(dcm, qcum->data, 0);
	pos = default_cam_mgr_get_cam_target(dcm, mtime);

	//vx_layer_camera_lookat(layer, pos->eye, pos->lookat, pos->up, 0);*/

	while(gstate->running){
		render_elements(0, new_world);
		
		vx_buffer_add_back(vx_world_get_buffer(new_world, "line"),
			vxo_pix_coords(VX_ORIGIN_TOP,
			vxo_chain(vxo_mat_scale3(DISPLAY_W/2.0, 1, 1),
			vxo_rect(vxo_mesh_style(vx_green),
			vxo_lines_style(vx_green, 2.0f),
			vxo_points_style(vx_green, 2.0f)))));
		vx_buffer_swap(vx_world_get_buffer(new_world, "line"));
		
		vx_buffer_add_back(vx_world_get_buffer(new_world, "board"), 
			vxo_chain(vxo_mat_translate3(0,-18,arm_zoom),
			vxo_mat_scale3(61, 1, 61),
			vxo_box(vxo_mesh_style(vx_gray),
			vxo_lines_style(vx_gray, 2.0f),
			vxo_points_style(vx_gray, 2.0f))));
		vx_buffer_swap(vx_world_get_buffer(new_world, "board"));

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

	int last_getting_balls = getting_balls;
	int started = 0;

	while(gstate->running){		
		char angleText[128] = "";
		char ballText[128] = "";
		char statusText[64] = "";
		char timeText[64] = "";
		
		sprintf(angleText, " Servo angles:\n 0: [%f]\n 1: [%f]\n 2: [%f]\n 3: [%f]\n 4: [%f]\n 5: [%f]", servo_positions[0]-M_PI, servo_positions[1], servo_positions[2], servo_positions[3], servo_positions[4], servo_positions[5]);
		vx_object_t* texta = vxo_text_create(VXO_TEXT_ANCHOR_BOTTOM_RIGHT, angleText);
		vx_buffer_add_back(vx_world_get_buffer(new_world, "text"), vxo_pix_coords(VX_ORIGIN_BOTTOM_RIGHT, texta));
		
		int num_valid_balls = num_balls;
		int balls_valid[num_balls];
		
		for(int i = 0; i < num_balls; i++){
		      if(fabs(balls[i].x) > 29.5 || fabs(balls[i].y) >
			      29.5 || (balls[i].y > -9 && balls[i].y < 9 &&
			      balls[i].x < -13)){
		            num_valid_balls--;
		            balls_valid[i] = 0;
		      }else{
		             double r = sqrt(pow(balls[i].x, 2) + pow(balls[i].y, 2));
		             if(r > MAX_RADIUS){
		                num_valid_balls--;
		                balls_valid[i] = 0;
		             }else{
		                balls_valid[i] = 1;
		             }
		      }
		}
		//pthread_mutex_lock(&ball_mutex);
		sprintf(ballText, " %d reachable balls:\n", num_valid_balls);
		for(int i = 0; i < num_balls; ++i) {
		    if(balls_valid[i]){
		        sprintf(ballText +strlen(ballText), " %d ( %f, %f)\n", i,
			        balls[i].x, balls[i].y);
			}
		}
		//pthread_mutex_lock(&ball_mutex);
		//printf("%s", statusText);
		vx_object_t* textb = vxo_text_create(VXO_TEXT_ANCHOR_TOP_LEFT, ballText);
		vx_buffer_add_back(vx_world_get_buffer(new_world, "text"), vxo_pix_coords(VX_ORIGIN_TOP_LEFT, textb));

		vx_object_t* status;
		vx_object_t* time;
		double time_elapsed;

		if(getting_balls){
			clock_t cur_time = clock();
			time_elapsed = (double)(cur_time - ball_clock)/CLOCKS_PER_SEC;
		}
		sprintf(timeText, "\n%s%gs\n", "<<#000000,right>>", time_elapsed);
		
		if(getting_balls){
			sprintf(statusText, "%s\n%s", "<<#00ff00>>Retrieving balls...", timeText);
			status = vxo_text_create(VXO_TEXT_ANCHOR_TOP_RIGHT, statusText);
			vx_buffer_add_back(vx_world_get_buffer(new_world, "text"), vxo_pix_coords(VX_ORIGIN_TOP_RIGHT, status));
		}
		
		if(!last_getting_balls && getting_balls){
			started = 1;
		}

		if(started && !getting_balls){
			sprintf(statusText, "%s\n%s", "<<#ff0000>>DONE!   ", timeText);
			status = vxo_text_create(VXO_TEXT_ANCHOR_TOP_RIGHT, statusText);
			vx_buffer_add_back(vx_world_get_buffer(new_world, "text"), vxo_pix_coords(VX_ORIGIN_TOP_RIGHT, status));
		}

		/*if(started){
			
			time = vxo_text_create(VXO_TEXT_ANCHOR_TOP_RIGHT, timeText);
			vx_buffer_add_back(vx_world_get_buffer(new_world, "text"), vxo_pix_coords(VX_ORIGIN_TOP_RIGHT, time));
		}*/

		last_getting_balls = getting_balls;

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
	getopt_add_string(gstate->gopt, '\0', "ball-channel", "ARM_BALLS", "Ball positions channel");
	getopt_add_string(gstate->gopt, '\0', "action-channel", "ARM_ACTION", "Arm action channel");
	getopt_add_bool(gstate->gopt, 'c', "camera", 0, "laptop");


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
	ball_channel = getopt_get_string(gstate->gopt, "ball-channel");
	action_channel = getopt_get_string(gstate->gopt, "action-channel");

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
	if(getopt_get_bool(gstate->gopt, "camera")) {
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
	pthread_create(&gstate->animate_thread, NULL, initScalingFactors, gstate);

	
	
    // Initialize a parameter gui
   
    //pg_add_double_slider(pg, "sl2", "Zoom Arm", 0, 20, 5);
	/*pg_add_double_slider(pg, "s0", "Rotation", -M_PI, M_PI, 0);
	pg_add_double_slider(pg, "s1", "Servo 1", -M_PI, M_PI, 0);
	pg_add_double_slider(pg, "s2", "Servo 2", -M_PI, M_PI, 0);
	pg_add_double_slider(pg, "s3", "Servo 3", -M_PI, M_PI, 0);
	pg_add_double_slider(pg, "s4", "Servo 4", -M_PI, M_PI, 0);
	pg_add_double_slider(pg, "s5", "Hand", 0, M_PI, 0);*/
   /* pg_add_check_boxes(pg,
                       "cb1", "Camera", 1,
                       NULL);*/
    pg_add_buttons(pg,
                   "but1", "Calibrate Camera",
                   NULL);
                   
    pg_add_buttons(pg,
                   "but2", "Get Balls",
                   NULL);
                   
    pg_add_buttons(pg,
                   "but3", "Stop Getting Balls",
                   NULL);
                   
   pg_add_buttons(pg,
                   "but4", "Go to home",
                   NULL);

    parameter_listener_t *my_listener = calloc(1,sizeof(parameter_listener_t));
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
