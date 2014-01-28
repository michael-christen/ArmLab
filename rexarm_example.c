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
#define ARM_L1 11
#define ARM_L2 10
#define ARM_L3 10
#define ARM_L4 18

typedef struct state state_t;
struct state
{
    // LCM
    lcm_t *lcm;
    const char *command_channel;
    const char *status_channel;
	const char *gui_channel;

    pthread_t status_thread;
    pthread_t command_thread;
	pthread_t gui_thread;
};

static int64_t utime_now()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void getServoAngles(double *servos, double theta, double r, double height) {
	const double yDisp = ARM_L4 + height - ARM_L1;
	const double rCrit = sqrt(pow(ARM_L2 + ARM_L3, 2) + pow(yDisp, 2));

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

void openClaw(double *servos){
	servos[5] = M_PI/3;
}

void closeClaw(double *servos){
	servos[5] = M_PI/2;
}

void sendCommand(state_t* state, double theta, double r, double height, int clawOpen, double speed, double torque) {
	
	dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = malloc(sizeof(dynamixel_command_t)*NUM_SERVOS);

	double positions[NUM_SERVOS];

	getServoAngles(positions, theta, r, height);

	if(clawOpen == 0){
		openClaw(positions);
	}else if(clawOpen == 1){
		closeClaw(positions);
	}	//Don't change claw if other value

    // Send LCM commands to arm. Normally, you would update positions, etc,
    // but here, we will just home the arm.
    for (int id = 0; id < NUM_SERVOS; id++) {
        cmds.commands[id].utime = utime_now();
        cmds.commands[id].position_radians = positions[id];
        cmds.commands[id].speed = speed;
		cmds.commands[id].max_torque = torque;
    }
    dynamixel_command_list_t_publish(state->lcm, state->command_channel, &cmds);

	free(cmds.commands);
}

static void status_handler(const lcm_recv_buf_t *rbuf,
                           const char *channel,
                           const dynamixel_status_list_t *msg,
                           void *user)
{
    // Print out servo positions
	double position_radians[NUM_SERVOS];
   for (int id = 0; id < msg->len; id++) {
        dynamixel_status_t stat = msg->statuses[id];
		position_radians[id] = stat.position_radians;
        //printf("[id %02d]=%3.3f ",id, stat.position_radians);
    }
	
    //printf("\n");

	gui_update_servo_pos(position_radians);
}

static void click_handler(const lcm_recv_buf_t *rbuf,
                           const char *channel,
                           const dynamixel_status_list_t *msg,
                           void *user){

	state_t* state = user;

	double x, y, display_h, display_w;
	dynamixel_status_t stat = msg->statuses[0];
	x = stat.speed;
	y = stat.load;
	display_h = stat.voltage - 50;
	display_w = stat.temperature;

	if(x < display_w/2.0){
		//Bird's-eye view

		double origx = display_w/4.0;
		double origy = 3*display_h/4.0;

		double deltay = (y - origy)/5.0;
		double deltax = (x - origx)/5.0;

		double r = sqrt(pow(deltax, 2) + pow(deltay, 2));
		double theta = atan(deltay/deltax);

		if(deltax < 0 && deltay > 0){
			theta += M_PI;
		}
		if(deltax < 0 && deltay < 0){
			theta -= M_PI;
		}
		double height = 8;

		printf("%f\n", r);
		if(r < 13.0){
			sendCommand(state, theta, r, height, 2, .1, .4);
		}

	}else{
		//Camera
	}
	//printf("%i, %i\n", x, y);
}

void* status_loop(void *data)
{
    state_t *state = data;
    dynamixel_status_list_t_subscribe(state->lcm,
                                      state->status_channel,
                                      status_handler,
                                      state);

	dynamixel_status_list_t_subscribe(state->lcm,
                                      state->gui_channel,
                                      click_handler,
                                      state);

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
	getopt_add_string(gopt, '\0', "gui-channel", "ARM_GUI", "GUI channel");
	getopt_add_string(gopt, 'm', "mode", "VIEW_MODE", "click | view | camera");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        exit(-1);
    }

    state_t *state = malloc(sizeof(state_t));
    state->lcm = lcm_create(NULL);
    state->command_channel = getopt_get_string(gopt, "command-channel");
    state->status_channel = getopt_get_string(gopt, "status-channel");
	state->gui_channel = getopt_get_string(gopt, "gui-channel");

    pthread_create(&state->status_thread, NULL, status_loop, state);
    //pthread_create(&state->command_thread, NULL, command_test, state);
	pthread_create(&state->gui_thread, NULL, gui_create(argc, argv), state);

    // Probably not needed, given how this operates
    pthread_join(state->status_thread, NULL);
    pthread_join(state->command_thread, NULL);

    lcm_destroy(state->lcm);
    free(state);
    getopt_destroy(gopt);

return 0;
}
