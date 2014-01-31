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
#define RADIAN_ERROR 0.05

typedef struct state state_t;
struct state
{
    // LCM
    lcm_t *lcm;
    const char *command_channel;
	//const char *command_mail_channel;
    const char *lcm_channel;
    const char *status_channel;
    const char *gui_channel;

    pthread_t lcm_handle_thread;
    pthread_t command_thread;
    pthread_t status_thread;
    pthread_t command_mail_thread;
    pthread_t gui_thread;

/*
    lcm_recv_buf_t *lcm_rbuf;
    const char *lcm_channel;
    const dynamixel_status_list_t *lcm_msg;
    void *lcm_user;
*/
};

double cur_speeds[NUM_SERVOS];
double cur_positions[NUM_SERVOS];

pthread_mutex_t sample_mutex;
pthread_cond_t sample_cv;

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
	const double yDisp = ARM_L4 + height - ARM_L1;
	const double rCrit = sqrt(pow(ARM_L2 + ARM_L3, 2) - pow(yDisp, 2));

	servos[0] = theta;

	if (r <= rCrit) {
		double h, tc, ta, tb;

		h = sqrt(pow(r, 2) + pow(yDisp, 2));
		tc = atan(yDisp/r);
		ta = acos((pow(ARM_L3, 2) - pow(ARM_L2, 2) - pow(h, 2)) / (-2 * ARM_L2 * h));
		tb = acos((pow(h, 2) - pow(ARM_L2, 2) - pow(ARM_L3, 2)) / (-2 * ARM_L2 * ARM_L3));

		servos[1] = PI/2 - tc - ta;
		servos[2] = PI - tb;
		servos[3] = PI - servos[1] - servos[2];
	} else {
		// Implement stretch math
	}
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

/*static void executeCommand(const lcm_recv_buf_t *rbuf,
                           const char *channel,
                           const dynamixel_command_list_t *cmds,
                           void *user){
	while(armIsMoving()){
		usleep(1000);
	}

	dynamixel_command_list_t_publish(state->lcm, state->command_channel, &cmds);
{*/

void* sendCommand(state_t* state, double theta, double r, double height, int clawOpen, double speed, double torque) 
{
    pthread_mutex_lock(&sample_mutex);

    pthread_cond_wait(&sample_cv, &sample_mutex);

    neverMoved = 0;
    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = malloc(sizeof(dynamixel_command_t)*NUM_SERVOS);

    double positions[NUM_SERVOS];
    for(int i = 0; i < NUM_SERVOS; i++){
	positions[i] = cur_positions[i];
    }

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

    pthread_mutex_unlock(&sample_mutex);

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
    sendCommand(state, theta, r, 8, 1, .7, .5);
    printf("2\n");
    sendCommand(state, theta, r, 1, 1, .5, .5);
    printf("3\n");
    sendCommand(state, theta, r, 1, 0, .7, .5);
    printf("4\n");
    sendCommand(state, theta, r, 8, 0, .7, .5);	
    printf("Finish\n");
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
    if(r < 16.0){
	pickUpBall(state, theta, r);
    }
}

void* commandListener(void *data){
	state_t* state = data;
	while(1) {
	    pthread_mutex_lock(&command_mutex);
	    pthread_cond_wait(&command_cv, &command_mutex);

	    //printf("... handling command\n");
	    click_handler(command_rbuf, command_msg, state);

	    pthread_mutex_unlock(&command_mutex);
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
    //Set false after first time
    if(neverMoved) {
	neverMoved = 0;
    }
    int satisfied = 1;
    for (id = 0; id < NUM_SERVOS; id++) {
	//If all servos aren't in position
	double error = getError(
		global_cmds.commands[id].position_radians,
		position_radians[id]
	);
	if(error > RADIAN_ERROR) {
	    printf("NOT SATISFIED\n");
	    printf("global pos %d: %f, cur pos %f\n", id,
		    global_cmds.commands[id].position_radians,
		    position_radians[id]);
	    satisfied = 0;
	    break;
	}
    }

    pthread_mutex_lock(&sample_mutex);
    if(satisfied){
	//printf("signaling\n");
	pthread_cond_signal(&sample_cv);
    }
    pthread_mutex_unlock(&sample_mutex);

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



void* command_test(void *data){
	state_t *state = data;
	sendCommand(state, M_PI, 13, 8, 1, .1, .5);
	/*int hz = 30;

   	state_t *state = data;

    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = malloc(sizeof(dynamixel_command_t)*NUM_SERVOS);

	double positions[NUM_SERVOS];

	getServoAngles(positions, M_PI, 13, 8);
	openClaw(positions);

    // Send LCM commands to arm. Normally, you would update positions, etc,
    // but here, we will just home the arm.
    for (int id = 0; id < NUM_SERVOS; id++) {
        cmds.commands[id].utime = utime_now();
        cmds.commands[id].position_radians = positions[id];
        cmds.commands[id].speed = 0.1;
		cmds.commands[id].max_torque = .0;
    }
    dynamixel_command_list_t_publish(state->lcm, state->command_channel, &cmds);

    usleep(1000000/hz);

	
    free(cmds.commands);*/

    return NULL;
}

void* command_home(void *data){
	int hz = 30;

    state_t *state = data;

    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = malloc(sizeof(dynamixel_command_t)*NUM_SERVOS);

	double positions[NUM_SERVOS] = {-M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0, 2*M_PI/3};

    // Send LCM commands to arm. Normally, you would update positions, etc,
    // but here, we will just home the arm.
    for (int id = 0; id < NUM_SERVOS; id++) {
        cmds.commands[id].utime = utime_now();
        cmds.commands[id].position_radians = positions[id];
        cmds.commands[id].speed = 0.2;
		cmds.commands[id].max_torque = 0;
    }
    dynamixel_command_list_t_publish(state->lcm, state->command_channel, &cmds);

    usleep(1000000/hz);

	
    free(cmds.commands);

    return NULL;
}

void* command_loop(void *data)
{
    int hz = 30;

    state_t *state = data;

    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = malloc(sizeof(dynamixel_command_t)*NUM_SERVOS);

	//double positions[NUM_SERVOS] = {-M_PI/2, -3*M_PI/4, 2.7*M_PI/4, M_PI/2, 0, 0};
    while (1) {
        // Send LCM commands to arm. Normally, you would update positions, etc,
        // but here, we will just home the arm.
        for (int id = 0; id < NUM_SERVOS; id++) {
            cmds.commands[id].utime = utime_now();
            cmds.commands[id].position_radians = 0;
            cmds.commands[id].speed = 0.5;
			cmds.commands[id].max_torque = .5;
        }
        dynamixel_command_list_t_publish(state->lcm, state->command_channel, &cmds);

        usleep(1000000/hz);
    }
	
    free(cmds.commands);

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
	getopt_add_bool(gopt, 'c', "camera", 0, "laptop");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        exit(-1);
    }

    state_t *state = malloc(sizeof(state_t));
    state->lcm = lcm_create(NULL);
    state->command_channel = getopt_get_string(gopt, "command-channel");
	//state->command_mail_channel = getopt_get_string(gopt, "command-mail-channel");
    state->status_channel = getopt_get_string(gopt, "status-channel");
    state->lcm_channel = "lcm-channel";

    global_cmds.len = NUM_SERVOS;
    global_cmds.commands = malloc(sizeof(dynamixel_command_t)*NUM_SERVOS);

    printf("Status_ch: %s\n",state->status_channel);
	state->gui_channel = getopt_get_string(gopt, "gui-channel");

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
