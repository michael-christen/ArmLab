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
#define MAX_RADIUS 34
#define MAX_CMD_DUR 3.0
#define UPDATE_INTERVAL 0.5

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

    volatile int gettingBalls, num_balls, goToHome;
    double cur_x, cur_y;
};

double cur_speeds[NUM_SERVOS];
double cur_positions[NUM_SERVOS];

double RADIAN_ERROR = 0.05;

pthread_mutex_t cmd_mutex;
pthread_cond_t cmd_cv;
int start_cmd, end_cmd;
clock_t cmd_begin, cmd_check, cmd_last_update;

pthread_mutex_t command_mutex;
pthread_cond_t command_cv, command_exit_cv;

const lcm_recv_buf_t *command_rbuf, *status_rbuf;
dynamixel_status_list_t *command_msg, *status_msg;

pthread_mutex_t status_mutex;
pthread_cond_t status_cv, status_exit_cv;

dynamixel_command_list_t global_cmds;
int neverMoved = 1;
int hasMoved;

double dropHeight = 12;
double pickupHeight = 6;

double UL_scaling_factor = 4.594595;

static int64_t utime_now()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void getServoAngles(double *servos, double theta, double r, double height) {
    if (r == 0) {
        int i;
	servos[0] = theta;
        for (i = 1; i < NUM_SERVOS; i++) {
            servos[i] = 0;
        }
        return;
    }
    
    // Set base servo angle
    servos[0] = theta;

	if (height < ARM_L1) {
        double rCritDueToHeight, rCritDueToAngle, rCrit;

        rCritDueToHeight = sqrt(pow(ARM_L2 + ARM_L3, 2) - pow(ARM_L4 + height - ARM_L1, 2));
        //rCritDueToAngle = (ARM_L4 + height - ARM_L1) * tan(0.99);
	rCritDueToAngle = (ARM_L2 + ARM_L3) * cos(0.5792);
	rCrit = rCritDueToHeight < rCritDueToAngle ? rCritDueToHeight :
	    rCritDueToAngle;
        //rCrit = rCritDueToAngle > rCritDueToHeight ? rCritDueToAngle : rCritDueToHeight;
        printf("r: %f, height: %f\n", r, height);
        printf("rCritHeight: %f, rCritAngle: %f, rCrit: %f\n", rCritDueToHeight, rCritDueToAngle, rCrit);

        if (r < rCrit) {
            // height < ARM_L1 && r < rCrit
            printf("Angle case 0\n");
            double yDisp, h, t2a, t2b, t3a;

            yDisp = ARM_L4 + height - ARM_L1;
            h = sqrt(pow(r, 2) + pow(yDisp, 2));
            t2a = atan(yDisp / r);
            t2b = acos((pow(ARM_L3, 2) - pow(ARM_L2, 2) - pow(h, 2)) / (-2 * ARM_L2 * h));
            t3a = acos((pow(h, 2) - pow(ARM_L2, 2) - pow(ARM_L3, 2)) / (-2 * ARM_L2 * ARM_L3));
            
            servos[1] = (PI / 2) - t2a - t2b;
            servos[2] = PI - t3a;
            servos[3] = PI - servos[1] - servos[2];
        } else {
            // height < ARM_L1 && r >= rCrit
            printf("Angle case 1\n");
            double yDisp, h, t2a, t2b, t4a;

            yDisp = ARM_L1 - height;
            h = sqrt(pow(r, 2) + pow(yDisp, 2));
            t2a = atan(r / yDisp);
            t2b = acos((pow(ARM_L4, 2) - pow(ARM_L2 + ARM_L3, 2) - pow(h, 2)) / (-2 * (ARM_L2 + ARM_L3) * h));
            t4a = acos((pow(h, 2) - pow(ARM_L2 + ARM_L3, 2) - pow(ARM_L4, 2)) / (-2 * (ARM_L2 + ARM_L3) * ARM_L4));
            
            servos[1] = PI - t2a - t2b;
            servos[2] = 0;
            servos[3] = PI - t4a;
        }
    } else {
        double rCrit = sqrt(pow(ARM_L2 + ARM_L3, 2) - pow(ARM_L4, 2));

        if (r < rCrit) {
            // height >= ARM_L1 && r < rCrit
            printf("Angle case 2\n");
            double yDisp, t2a, t2b, t2c, tha, h, ha, t3a;

            yDisp = height - ARM_L1;
            ha = sqrt(pow(r, 2) + pow(yDisp, 2));
            t2a = atan(yDisp / r);
            tha = (PI / 2) + t2a;
            h = sqrt(pow(ha, 2) + pow(ARM_L4, 2) - (2 * ha * ARM_L4 * cos(tha)));
            t2b = acos((pow(ARM_L4, 2) - pow(h, 2) - pow(ha, 2)) / (-2 * h * ha));
            t2c = acos((pow(ARM_L3, 2) - pow(ARM_L2, 2) - pow(ha, 2)) / (-2 * ARM_L2 * h));
            t3a = asin((h * sin(t2c)) / ARM_L3);

            servos[1] = (PI / 2) - t2a - t2b - t2c;
            servos[2] = PI - t3a;
            servos[3] = PI - servos[1] - servos[2];
        } else {
            // height >= ARM_L1 && r >= rCrit
            printf("Angle case 3\n");
            double yDisp, h, t2a, t2b, t4a;

            yDisp = height - ARM_L1;
            h = sqrt(pow(r, 2) + pow(yDisp, 2));
            t2a = atan(yDisp / r);
            t2b = acos((pow(ARM_L4, 2) - pow(ARM_L2 + ARM_L3, 2) - pow(h, 2)) / (-2 * (ARM_L2 + ARM_L3) * h));
            t4a = acos((pow(h, 2) - pow(ARM_L2 + ARM_L3, 2) - pow(ARM_L4, 2)) / (-2 * (ARM_L2 + ARM_L3) * ARM_L4));
            
            servos[1] = (PI / 2) - t2a - t2b;
            servos[2] = 0;
            servos[3] = PI - t4a;
        }
    }
	
	if(servos[0] < 0){
		servos[0] += M_PI-.1;
		for(int i = 1; i < 4; i++){
			servos[i] = -servos[i];
		}
	}else if(servos[0] < (M_PI-.1)){
		servos[0] += .1;
	}

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


    state->cur_x = cos(theta) * r;
    state->cur_y = sin(theta) * r;

    neverMoved = 0;
    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = malloc(sizeof(dynamixel_command_t)*NUM_SERVOS);

    double positions[NUM_SERVOS];
    for(int i = 0; i < NUM_SERVOS; i++){
	positions[i] = cur_positions[i];
	    RADIAN_ERROR += 0.01;
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

    //Wait for other to finish
    cmd_begin = clock();
    cmd_last_update = cmd_begin;
    //Show that you want to be signalled
    end_cmd = 1;
    int firstTimeThrough = 1;
    while(!start_cmd) { 
	if(!firstTimeThrough) {
	    if(cmds.commands[0].max_torque < 1){
		printf("going to increase torque\n");
		for(int i = 0; i < NUM_SERVOS; ++i) {
		    cmds.commands[i].max_torque += 0.1;
		}
		printf("torque: %f\n",cmds.commands[0].max_torque);
		dynamixel_command_list_t_publish(state->lcm,
			state->command_channel, &cmds);
	    }
	    RADIAN_ERROR += 0.01;
	}
	firstTimeThrough = 0;
	pthread_cond_wait(&cmd_cv, &cmd_mutex);
    }
    RADIAN_ERROR = 0.05;

    start_cmd = 0;
    end_cmd = 0;

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
    double speed = 1.0;
    double speedSlow = 0.3;
    double speedSlowest = 0.05;
    double torque = 0.7;
    double interumTheta = 2.8;

    if (hasMoved == 1) {
        // This is an interum step after the arm has dropped off a ball at the basket
        // This moves the arm just clear of the bucket, and lowers the height of the
        // arm to prevent the arm colliding with itself
        printf("0\n");
        sendCommand(state, interumTheta, r, dropHeight, 1, speed, torque);
    } else {
        hasMoved = 1;
    }
    
    printf("1\n");
    sendCommand(state, theta, r, pickupHeight, 1, speed, torque);
    printf("2\n");
    sendCommand(state, theta, r, 7, 1, speed, torque);
    printf("3\n");
    sendCommand(state, theta, r, 4, 1, speedSlow, torque);
    printf("4\n");
    sendCommand(state, theta, r, 1, 1, speedSlowest, torque);
    printf("5\n");
    sendCommand(state, theta, r, 1, 0, speedSlow * 2, torque);
    printf("6\n");
    sendCommand(state, theta, r, pickupHeight, 0, speed, torque);	
    printf("Finish\n");
}

void dropBall(state_t* state){
    //printf("pickupBall\n");
    double theta;
    double r = 25;
    double speed = 1.0;
    double torque = 0.7;
    
    printf("cur theta: %f\n", cur_positions[0]);
    if (cur_positions[0] > 0 && cur_positions[3] > 0) {
        theta = 3.1;
    } else {
        theta = -3.1;
    }
    
    printf("d1 %f\n", theta);
    sendCommand(state, theta, r, dropHeight, 0, speed, torque);
    printf("d2\n");
    sendCommand(state, theta, r, dropHeight, 1, speed, torque);
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
        state->gettingBalls = 1;
        hasMoved = 0;        
    } else {
        printf("K, stop getting teh ballz\n");
        state->gettingBalls = 0;
        stop_getting_balls();
    }
    
    if (msg->goToHome == 1) {
        state->goToHome = 1;
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

double calc_dist(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}

int getNextBall(state_t * state, ball_info_t * rtnBall) {
    int num_balls = state->num_balls;
    double x, y, r;
    int isBall = 0;
    assert(num_balls > 0);
    ball_info_t max_ball = state->balls[0];
    double min_dist = calc_dist(state->cur_x, state->cur_y, max_ball.x, max_ball.y);
    double cur_dist;

	int positivey = 0;
	for(int i = 0; i < num_balls; i++){
		x = state->balls[i].x;
		y = -1 * state->balls[i].y;
		r = calc_dist(x,y,0,0);
		if(y >= 0 && !(r > MAX_RADIUS || fabs(x > 29.5) || fabs(y) > 29.5 || (y > -9 && y < 9 && x < 0))){
			positivey = 1;
			break;
		}
	}

    for(int i = 0; i < num_balls; ++i) {
	x = state->balls[i].x;
	y = -1 * state->balls[i].y;
	if(positivey && y < 0){
		continue;
	}
	r = calc_dist(x,y,0,0);
	if (!(r > MAX_RADIUS || fabs(x > 29.5) || fabs(y) > 29.5 || (y > -9 && y < 9 && x < 0))) {
	    cur_dist = calc_dist(state->cur_x, state->cur_y,
		    state->balls[i].x, state->balls[i].y);
	    if( cur_dist < min_dist || !isBall) {
		max_ball = state->balls[i];
	    }
	    isBall = 1;
	}
    }
    if(isBall) {
	rtnBall->x = max_ball.x;
	rtnBall->y = max_ball.y;
	rtnBall->num_pxs = max_ball.num_pxs;
    }
    return isBall;
}

void* commandListener(void *data){
	state_t* state = data;
	while(1) {
	    //printf("%d\n", state->gettingBalls);
	    if (state->gettingBalls == 1) {
		pthread_mutex_lock(&command_mutex);

		double x, y;
		ball_info_t curBall;
	        int isBall = getNextBall(state, &curBall);
		x = curBall.x;
		y = -1 * curBall.y;

		double r = calc_dist(x,y,0,0);
		double theta = atan(y/x);

		if(!isBall) {
		    // No valid balls left
		    printf("No more valid balls\n");
		    state->gettingBalls = 0;
		    stop_getting_balls();
		}
		else {
		    if(x < 0 && y > 0){
			theta += M_PI;
		    }
		    if(x < 0 && y < 0){
			theta -= M_PI;
		    }
		    printf("x: %f, y: %f, r: %f\n", x, y, r);
		    pickUpBall(state, theta, r);
		    dropBall(state);
		}
		pthread_mutex_unlock(&command_mutex);

	    }
	    if (state->goToHome == 1) {
		pthread_mutex_lock(&command_mutex);

		sendCommand(state, 3.1, 0, 0, 0, 0.3, 0.7);

		pthread_mutex_unlock(&command_mutex);
		state->goToHome = 0;   
	    }
	}
	return NULL;
}

double getError(double a, double b) {
    double diff = a - b;
    double reg_diff = fmod(diff,2*M_PI);
    //printf("diff: %f, reg_diff: %f\n",diff, reg_diff);
    return fabs(reg_diff);
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
    double dur_cmd;

    pthread_mutex_lock(&cmd_mutex);

    cmd_check = clock();
    dur_cmd = ((double)cmd_check - (double)cmd_begin)
	/CLOCKS_PER_SEC;
    double diff_update = ((double)cmd_check -
	    (double)cmd_last_update)/CLOCKS_PER_SEC;
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

    //If there or stuck (exceeding time)
    //if((satisfied) && !start_cmd && end_cmd){
    if((satisfied || dur_cmd > MAX_CMD_DUR) && !start_cmd && end_cmd){
	printf("signaling after %f secs\n", dur_cmd);
	start_cmd = 1;
	pthread_cond_broadcast(&cmd_cv);
    } else if(!start_cmd && end_cmd && diff_update > UPDATE_INTERVAL) {
	cmd_last_update = clock();
	//Alerts to increase torque
	pthread_cond_broadcast(&cmd_cv);
    }

    //Set false after first time
    if(neverMoved) {
	start_cmd = 1;
	end_cmd = 0;
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
    
    start_cmd = 0;
    end_cmd = 0;
    state_t *state = malloc(sizeof(state_t));
    state->gettingBalls = 0;
    state->lcm = lcm_create(NULL);
    state->command_channel = getopt_get_string(gopt, "command-channel");
	//state->command_mail_channel = getopt_get_string(gopt, "command-mail-channel");
    state->status_channel = getopt_get_string(gopt, "status-channel");
    state->lcm_channel = "lcm-channel";
    state->cur_x = 0;
    state->cur_y = 0;

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
