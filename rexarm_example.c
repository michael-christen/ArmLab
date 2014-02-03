#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>

#include <lcm/lcm.h>
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"
#include "lcmtypes/ball_list_t.h"
#include "lcmtypes/ball_info_t.h"
#include "lcmtypes/arm_action_t.h"

#include "common/dynamixel_device.h"
#include "common/dynamixel_serial_bus.h"
#include "common/getopt.h"
#include "common/math_util.h"

#include "gui.h" //gui depicting arm

#define NUM_SERVOS 6
#define _USE_MATH_DEFINES //PI
#define ARM_L1 11.5
#define ARM_L2 10
#define ARM_L3 10
#define ARM_L4 18
#define ARM_CLAW_WIDTH 5
#define RADIAN_ERROR 0.08

typedef struct state state_t;
struct state
{
    // LCM
    lcm_t *lcm;
    const char *command_channel;
    const char *lcm_channel;
    const char *status_channel;
    const char *gui_channel;
    const char *ball_channel;
    const char *action_channel;

    pthread_t lcm_handle_thread;
    pthread_t command_thread;
    pthread_t status_thread;
    pthread_t command_mail_thread;
    pthread_t gui_thread;
    
    ball_info_t balls[MAX_NUM_BALLS];
    volatile int gettingBalls, num_balls, balls_left;
};

double cur_speeds[NUM_SERVOS];
double cur_positions[NUM_SERVOS];

pthread_mutex_t cmd_mutex;
pthread_cond_t cmd_cv;
int new_cmd;

pthread_mutex_t command_mutex;
pthread_cond_t command_cv, command_exit_cv;

const lcm_recv_buf_t *command_rbuf, *status_rbuf;
dynamixel_status_list_t *command_msg, *status_msg;

pthread_mutex_t status_mutex;
pthread_cond_t status_cv, status_exit_cv;

dynamixel_command_list_t global_cmds;
int neverMoved = 1;

double UL_scaling_factor = 4.594595;

static int64_t utime_now()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void getServoAngles(double *servos, double theta, double r, double height) {
	double yDisp, h, t2a, t2b, t3a, t4a, rCritHeight, rCritHeightAngle, rCrit, rCritFar;
	
	if (height < ARM_L1) {
        rCritFar = sqrt(pow(ARM_L2 + ARM_L3 + ARM_L4, 2) - pow(ARM_L1 - height, 2));
    } else {
    	rCritFar = sqrt(pow(ARM_L2 + ARM_L3 + ARM_L4, 2) - pow(height - ARM_L1, 2));
	}
	
	if (r < rCrit || (r >= rCrit && height >= ARM_L1)) {
        if (r < rCrit) {
            yDisp = ARM_L4 + height - ARM_L1;
        } else {
            height += ARM_CLAW_WIDTH * ((r - rCrit) / (rCritFar - rCrit));
            yDisp = height - ARM_L1;
        }
    } else {
        height += ARM_CLAW_WIDTH * ((r - rCrit) / (rCritFar - rCrit));
        yDisp = ARM_L1 - height;
    }
	
	rCritHeight = sqrt(pow(ARM_L2 + ARM_L3, 2) - pow(yDisp, 2));
	rCritHeightAngle = (ARM_L2 + ARM_L3) * cos(0.5792);
	rCrit = rCritHeight < rCritHeightAngle ? rCritHeight : rCritHeightAngle;

	servos[0] = theta;
    if (r < rCrit || (r >= rCrit && height >= ARM_L1)) {
        if (r < rCrit) {
    		h = sqrt(pow(r, 2) + pow(yDisp, 2));
	        t2a = atan(yDisp / r);
            t2b = acos((pow(ARM_L3, 2) - pow(ARM_L2, 2) - pow(h, 2)) / (-2 * ARM_L2 * h));
    		t3a = acos((pow(h, 2) - pow(ARM_L2, 2) - pow(ARM_L3, 2)) / (-2 * ARM_L2 * ARM_L3));
    		
    		servos[1] = PI - t2a - t2b;
    		servos[2] = PI - t3a;
    		servos[3] = PI - servos[1] - servos[2];
        } else {
            
    		h = sqrt(pow(r, 2) + pow(yDisp, 2));
            t2a = atan(yDisp / r);
    		t2b = acos((pow(ARM_L4, 2) - pow(h, 2) - pow(ARM_L2 + ARM_L3, 2)) / (-2 * h * (ARM_L2 + ARM_L3)));
    		t4a = acos((pow(h, 2) - pow(ARM_L2 + ARM_L3, 2) - pow(ARM_L4, 2)) / (-2 * (ARM_L2 + ARM_L3) * ARM_L4));
    		
    		servos[1] = PI - t2a - t2b;
    		servos[2] = 0;
    		servos[3] = PI - t4a;
        }
    } else {
        // r >= rCrit && height < ARM_L1
		h = sqrt(pow(r, 2) + pow(yDisp, 2));
		t2a = atan(yDisp / r);
		t2b = acos((pow(ARM_L4, 2) - pow(h, 2) - pow(ARM_L2 + ARM_L3, 2)) / (-2 * h * (ARM_L2 + ARM_L3)));
		t4a = acos((pow(h, 2) - pow(ARM_L2 + ARM_L3, 2) - pow(ARM_L4, 2)) / (-2 * (ARM_L2 + ARM_L3) * ARM_L4));
		
		servos[1] = (PI / 2) - t2a - t2b;
		servos[2] = 0;
		servos[3] = PI - t4a;
    }
	printf("%f, %f, %f\n", r, rCrit, rCritFar);
	/*if ((r <= rCrit && (height < ARM_L1)) || height) {
		double h, tc, ta, tb;
		
		tc = atan(yDisp/r);
		ta = acos((pow(ARM_L3, 2) - pow(ARM_L2, 2) - pow(h, 2)) / (-2 * ARM_L2 * h));
		tb = acos((pow(h, 2) - pow(ARM_L2, 2) - pow(ARM_L3, 2)) / (-2 * ARM_L2 * ARM_L3));

		servos[1] = PI/2 - tc - ta;
		servos[2] = PI - tb;
		servos[3] = PI - servos[1] - servos[2];
	} else if (r > rCrit && r <= rCritFar) {
		double h, t2a, t2b, t4a;
		
		if (height < ARM_L1) {
		    yDisp = ARM_L1 - height;
	    } else {
	        yDisp = height - ARM_L1;
        }
		printf("height: %f\n", height);
		printf("h: %f\n", h);
		t4a = acos((pow(h, 2) - pow(ARM_L2 + ARM_L3, 2) - pow(ARM_L4, 2)) / (-2 * (ARM_L2 + ARM_L3) * ARM_L4));
		printf("t4a: %f\n", t4a);
		t2a = asin(r / h);
		printf("t2a: %f\n", t2a);
		t2b = acos((pow(ARM_L4, 2) - pow(h, 2) - pow(ARM_L2 + ARM_L3, 2)) / (-2 * h * (ARM_L2 + ARM_L3)));
		printf("t2b: %f\n", t2b);

		servos[1] = PI - t2a - t2b;
		servos[2] = 0;
		servos[3] = PI - t4a;
	} else {
	    printf("FAILING SERVO ANGLE COMP!!!!!!!!!!!!!\n");
    }*/
	printf("servos - %f, %f, %f\n", servos[1], servos[2], servos[3]);
}

void openClawAngles(double *servos){
	servos[5] = M_PI/3;
}

void closeClawAngles(double *servos){
	servos[5] = M_PI/2;
}

int armIsMoving(){
	return (cur_speeds[0] > .1 || cur_speeds[1] > .1 || cur_speeds[2] > .1 || cur_speeds[3] > .1 || cur_speeds[4] > .1 || cur_speeds[5] > .1);
}

void* sendCommand(state_t* state, double theta, double r, double height, int clawOpen, double speed, double torque) 
{
    pthread_mutex_lock(&cmd_mutex);

    while(!new_cmd) { 
	pthread_cond_wait(&cmd_cv, &cmd_mutex);
    }

    neverMoved = 0;
    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = malloc(sizeof(dynamixel_command_t)*NUM_SERVOS);

    double positions[NUM_SERVOS];
    for(int i = 0; i < NUM_SERVOS; i++){
	positions[i] = cur_positions[i];
    }
    positions[4] = 0;
    //Inits positions to desired theta, r, height
    getServoAngles(positions, theta, r, height);

    if(clawOpen == 1){
	openClawAngles(positions);
    }else if(clawOpen == 0){
	closeClawAngles(positions);
    }	//Don't change claw if other value

    // Send LCM commands to arm.
    for (int id = 0; id < NUM_SERVOS; id++) {
	cmds.commands[id].utime = utime_now();
	cmds.commands[id].position_radians = positions[id];
	global_cmds.commands[id].position_radians = positions[id];
	cmds.commands[id].speed = speed;
	cmds.commands[id].max_torque = torque;
    }
    //Send it after get signal from status
    dynamixel_command_list_t_publish(state->lcm, state->command_channel, &cmds);
    new_cmd = 0;

    pthread_mutex_unlock(&cmd_mutex);

    free(cmds.commands);	//Maybe move this to executeCommand if there's a seg fault
    return NULL;
}

void openClaw(state_t* state, double theta, double r, double height){
	sendCommand(state, theta, r, height, 1, .7, .5);
}

void closeClaw(state_t* state, double theta, double r, double height){
	sendCommand(state, theta, r, height, 0, .7, .5);
}

void pickUpBall(state_t* state, double theta, double r){
    //printf("pickupBall\n");
    printf("1\n");
    sendCommand(state, theta, r, 7, 1, .05, .7);
    printf("2\n");
    sendCommand(state, theta, r, 2, 1, .05, .7);
    printf("3\n");
    sendCommand(state, theta, r, 2, 0, .05, .7);
    printf("4\n");
    sendCommand(state, theta, r, 7, 0, .05, .7);	
    printf("Finish\n");
}

void dropBall(state_t* state, double theta, double r){
    //printf("pickupBall\n");
    printf("d1\n");
    sendCommand(state, theta, r, 15, 0, .05, .7);
    printf("d2\n");
    sendCommand(state, theta, r, 15, 1, .05, .7);
    printf("Finish Drop\n");
}


//Delegates messages to status_handler or click_handler
//Through the use of cv's
//We are doing this to allow multiple threads to wait on send commands
//based off signals from statuses
static void lcm_delegator( const lcm_recv_buf_t *rbuf,
                           const char *channel,
                           const dynamixel_status_list_t *msg,
                           void *user)
{
    state_t* state = user;
    if(!strcmp(channel, state->status_channel)) {
        pthread_mutex_lock(&status_mutex);

	status_msg = dynamixel_status_list_t_copy(msg);
	status_rbuf = rbuf;
        pthread_cond_signal(&status_cv);

        pthread_mutex_unlock(&status_mutex);
    } else if(!strcmp(channel, state->gui_channel)) {
        pthread_mutex_lock(&command_mutex);

	command_msg = dynamixel_status_list_t_copy(msg);
	command_rbuf = rbuf;
        pthread_cond_signal(&command_cv);

        pthread_mutex_unlock(&command_mutex);
    } 
}

static void ball_positions_handler( const lcm_recv_buf_t *rbuf,
                           const char *channel,
                           const ball_list_t *msg,
                           void *user)
{
    state_t* state = user;
    state->num_balls = msg->len;
    int i;
    for (i = 0; i < state->num_balls; i++) {
        state->balls[i].x = msg->balls[i].x;
        state->balls[i].y = msg->balls[i].y;
        state->balls[i].num_pxs = msg->balls[i].num_pxs;
    } 
}

static void arm_action_handler( const lcm_recv_buf_t *rbuf,
                           const char *channel,
                           const arm_action_t *msg,
                           void *user)
{
    state_t* state = user;
    if (msg->getBalls == 1) {
        printf("Received message to get teh ballzz %d %d\n", state->gettingBalls, msg->getBalls);
        state->balls_left = state->num_balls;
        state->gettingBalls = 1;
        
    } else {
        printf("K, stop getting teh ballz\n");
        state->gettingBalls = 0;
    }
}

void* lcm_handle_loop(void *data)
{
    state_t *state = data;
    //Get updated status
    dynamixel_status_list_t_subscribe(state->lcm,
                                      state->status_channel,
                                      lcm_delegator,
                                      state);
    //receive clicks
    dynamixel_status_list_t_subscribe(state->lcm,
	    state->gui_channel,
	    lcm_delegator,
	    state
    );
    
    //receive ball positions
    ball_list_t_subscribe(state->lcm,
	    state->ball_channel,
	    ball_positions_handler,
	    state
    );
    
    //receive arm actions
    arm_action_t_subscribe(state->lcm,
	    state->action_channel,
	    arm_action_handler,
	    state
    );

    int hz = 15;
    while (1) {
        // Set up the LCM file descriptor for waiting. This lets us monitor it
        // until somethign is "ready" to happen. In this case, we are ready to
        // receive a message.
        int lcm_fd = lcm_get_fileno(state->lcm);
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(lcm_fd, &fds);
		
        // Handle message if appropriate
        struct timeval timeout = {
            0,              // Seconds
            1000000/hz      // Microseconds
        };
		
        int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);
		
        if (0 == status) {
            continue;
        } else {
            // LCM has events ready to be processed
            lcm_handle(state->lcm);
        }
    }
	
    return NULL;
}

//Only messages from bird's eye view will be sent
void click_handler(const lcm_recv_buf_t *rbuf,
                           const dynamixel_status_list_t *msg,
                           void *user)
{
    state_t* state = user;

    double x, y;
    dynamixel_status_t stat = msg->statuses[0];
    x = stat.speed;
    y = stat.load;

    double r = sqrt(pow(x, 2) + pow(y, 2));
    double theta = atan(y/x);

    if(x < 0 && y > 0){
	theta += M_PI;
    }
    if(x < 0 && y < 0){
	theta -= M_PI;
    }
    printf("r: %f\n", r);
	pickUpBall(state, theta, r);
}

void* commandListener(void *data){
	state_t* state = data;
	while(1) {
	    //printf("%d\n", state->gettingBalls);
	    if (state->gettingBalls == 1 && state->balls_left > 0) {
	        
	        printf("Balls left: %d\n", state->balls_left);
	        state->balls_left--;
	        if (state->balls_left <= 0) {
	            state->gettingBalls = 0;
            }
	        pthread_mutex_lock(&command_mutex);

	        double x, y;
            x = state->balls[0].x;
            y = -1 * state->balls[0].y;

            double r = sqrt(pow(x, 2) + pow(y, 2));
            double theta = atan(y/x);

            if(x < 0 && y > 0){
	        theta += M_PI;
            }
            if(x < 0 && y < 0){
	        theta -= M_PI;
            }
            printf("x: %f, y: %f, r: %f\n", x, y, r);
	        pickUpBall(state, theta, r);
	        dropBall(state, (3 * PI) / 4, 28);

            pthread_mutex_unlock(&command_mutex);
        }
	    /*pthread_mutex_lock(&command_mutex);
	    pthread_cond_wait(&command_cv, &command_mutex);

	    click_handler(command_rbuf, command_msg, state);

	    pthread_mutex_unlock(&command_mutex);*/
	}
	return NULL;
}

double getError(double a, double b) {
    return fabs(a - b);
}

void status_handler(const lcm_recv_buf_t *rbuf,
                           const dynamixel_status_list_t *msg,
                           void *user)
{
    double position_radians[NUM_SERVOS];
    int id;
    for (id = 0; id < msg->len; id++) {
	dynamixel_status_t stat = msg->statuses[id];
	position_radians[id] = stat.position_radians;
	cur_speeds[id] = stat.speed;
	cur_positions[id] = position_radians[id];
	//Initialize global_cmds
	if(neverMoved) {
	    global_cmds.commands[id].position_radians =
		stat.position_radians;
	}
	//printf("[id %02d]=%3.3f ",id, stat.speed);
    }
    int satisfied = 1;
    for (id = 0; id < NUM_SERVOS; id++) {
	//If all servos aren't in position
	double error = getError(
		global_cmds.commands[id].position_radians,
		position_radians[id]
	);
	if(error > RADIAN_ERROR) {
	    //printf("NOT SATISFIED\n");
	    //printf("global pos %d: %f, cur pos %f\n", id,
		    //global_cmds.commands[id].position_radians,
		    //position_radians[id]);
	    satisfied = 0;
	    break;
	}
    }

    pthread_mutex_lock(&cmd_mutex);

    if(satisfied && !new_cmd){
	printf("signaling\n");
	new_cmd = 1;
	pthread_cond_broadcast(&cmd_cv);
    }
    /*
    if(satisfied){
	//printf("signaling\n");
	new_cmd = 1;
	pthread_cond_broadcast(&cmd_cv);
    }
    */

    //Set false after first time
    if(neverMoved) {
	new_cmd = 1;
	neverMoved = 0;
    }
    pthread_mutex_unlock(&cmd_mutex);

    gui_update_servo_pos(position_radians);
}

void* statusListener(void *data){
    state_t* state = data;
    while(1) {
	pthread_mutex_lock(&status_mutex);
	pthread_cond_wait(&status_cv, &status_mutex);

	//printf("..handling status\n");
	status_handler(status_rbuf, status_msg, state);

	pthread_mutex_unlock(&status_mutex);
	}
	return NULL;
}

// This subscribes to the status messages sent out by the arm, displaying servo
// state in the terminal. It also sends messages to the arm ordering it to the
// "home" position (all servos at 0 radians).
int main(int argc, char **argv)
{
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_string(gopt, '\0', "status-channel", "ARM_STATUS", "LCM status channel");
    getopt_add_string(gopt, '\0', "command-channel", "ARM_COMMAND", "LCM command channel");
	//getopt_add_string(gopt, '\0', "command-mail-channel","COMMAND_MAIL", "LCM command mail channel");
	getopt_add_string(gopt, '\0', "gui-channel", "ARM_GUI", "GUI channel");
	getopt_add_string(gopt, '\0', "ball-channel", "ARM_BALLS", "Ball positions channel");
	getopt_add_string(gopt, '\0', "action-channel", "ARM_ACTION", "Arm action channel");
	getopt_add_bool(gopt, 'c', "camera", 0, "laptop");
	command_msg = malloc(sizeof(dynamixel_status_list_t));
	command_msg->statuses = malloc(sizeof(dynamixel_status_t));

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        exit(-1);
    }
    
    new_cmd = 0;
    state_t *state = malloc(sizeof(state_t));
    state->gettingBalls = 0;
    state->lcm = lcm_create(NULL);
    state->command_channel = getopt_get_string(gopt, "command-channel");
	//state->command_mail_channel = getopt_get_string(gopt, "command-mail-channel");
    state->status_channel = getopt_get_string(gopt, "status-channel");
    state->lcm_channel = "lcm-channel";

    global_cmds.len = NUM_SERVOS;
    global_cmds.commands = malloc(sizeof(dynamixel_command_t)*NUM_SERVOS);

    printf("Status_ch: %s\n",state->status_channel);
	state->gui_channel = getopt_get_string(gopt, "gui-channel");
	state->ball_channel = getopt_get_string(gopt, "ball-channel");
	state->action_channel = getopt_get_string(gopt, "action-channel");

    //Handles incoming lcm messages and redirects to commandListener
    //or status Listener
    pthread_create(&state->lcm_handle_thread, NULL, lcm_handle_loop, state);

    pthread_create(&state->command_thread, NULL, commandListener, state);
    pthread_create(&state->status_thread, NULL, statusListener, state);

    //GUI
    pthread_create(&state->gui_thread, NULL, gui_create(argc, argv), state);

    // Probably not needed, given how this operates
    pthread_join(state->lcm_handle_thread, NULL);
    //pthread_join(state->command_thread, NULL);

    lcm_destroy(state->lcm);
    free(state);
    getopt_destroy(gopt);

return 0;
}
