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
#include "../lcmtypes/dynamixel_command_list_t.h"
#include "../lcmtypes/dynamixel_command_t.h"
#include "../lcmtypes/dynamixel_status_list_t.h"
#include "../lcmtypes/dynamixel_status_t.h"

#include "dynamixel_device.h"
#include "dynamixel_serial_bus.h"

#include "../common/getopt.h"
#include "../common/math_util.h"

#define NUM_SERVOS 6

#define dmax(A, B) (A > B ? A : B)
#define dmin(A, B) (A < B ? A : B)
#define dabs(A) (A < 0 ? -A : A)

typedef struct arm_state arm_state_t;
struct arm_state
{
    // Arm communication
    dynamixel_bus_t *bus;
    dynamixel_device_t *servos[NUM_SERVOS];

    // LCM
    lcm_t *lcm;
    const dynamixel_command_list_t *cmds;
    const char *command_channel;
    const char *status_channel;

    // Threading
    pthread_mutex_t status_mutex;
    pthread_t status_thread;
    pthread_mutex_t driver_mutex;
    pthread_t driver_thread;
};

static arm_state_t* arm_state_create(const char *busname, const int baud)
{
    arm_state_t *arm_state = malloc(sizeof(arm_state_t));
    arm_state->bus = serial_bus_create(busname, baud);
    //arm_state->servos = (dynamixel_device_t**) malloc(NUM_SERVOS*sizeof(dynamixel_device_t*));

    // Create the servos
    for (int id = 0; id < NUM_SERVOS; id++) {
        arm_state->servos[id] = arm_state->bus->get_servo(arm_state->bus, id);
        if (arm_state->servos[id] != NULL) {
            printf("Found %s servo id = %d\n",
                   arm_state->servos[id]->get_name(arm_state->servos[id]),
                   id);
        } else {
            printf("Could not find servo id = %d\n", id);
            exit(-1);
        }
    }

    return arm_state;
}

static void arm_state_destroy(arm_state_t* arm_state)
{
    for (int i = 0; i < NUM_SERVOS; i++) {
        arm_state->servos[i]->destroy(arm_state->servos[i]);
    }
    arm_state->bus->destroy(arm_state->bus);
    lcm_destroy(arm_state->lcm);
    free(arm_state);
}

static int64_t utime_now()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void* status_loop(void *args)
{
    printf("NFO: Starting status loop.\n");
    int hz = 15; // Target message rate
    arm_state_t *arm_state = args;

    dynamixel_status_list_t stats;
    stats.len = NUM_SERVOS;
    stats.statuses = malloc(NUM_SERVOS*sizeof(dynamixel_status_t));
    while (1) {
        int64_t utime = utime_now();

        for (int id = 0; id < NUM_SERVOS; id++) {
            stats.statuses[id].utime = utime_now();

            dynamixel_device_status_t *stat = arm_state->servos[id]->get_status(arm_state->servos[id]);

            stats.statuses[id].error_flags = stat->error_flags;
            stats.statuses[id].position_radians = stat->position_radians;
            stats.statuses[id].speed = stat->speed;
            stats.statuses[id].load = stat->load;
            stats.statuses[id].voltage = stat->voltage;
            stats.statuses[id].temperature = stat->temperature;

            dynamixel_device_status_destroy(stat);
        }

        // Publish
        dynamixel_status_list_t_publish(arm_state->lcm, arm_state->status_channel, &stats);

        // Attempt to send messages at a fixed rate
        int64_t max_delay = (1000000 / hz);
        int64_t now = utime_now();
        int64_t delay = imin64(now - utime, max_delay);
        utime = now;
        usleep(max_delay - delay);
    }

    return NULL;
}

// === LCM Handler ==============
static void command_handler(const lcm_recv_buf_t *rbuf,
                            const char *channel,
                            const dynamixel_command_list_t *msg,
                            void *user)
{
    arm_state_t *arm_state = user;
    arm_state->cmds = msg;
}

void* driver_loop(void *args)
{
    printf("NFO: Starting driver loop.\n");
    arm_state_t *arm_state = args;

    int hz = 100;

    dynamixel_command_list_t last_cmds;
    last_cmds.len = NUM_SERVOS;
    last_cmds.commands = malloc(NUM_SERVOS*sizeof(dynamixel_command_t));
    for (int id = 0; id < NUM_SERVOS; id++) {
        last_cmds.commands[id].utime = 0;
        last_cmds.commands[id].position_radians = 0;
        last_cmds.commands[id].speed = 0;
        last_cmds.commands[id].max_torque = 0;
    }

    // Handle messages as they come in from the arm
    while (1) {
        // Set up the LCM file descriptor for waiting
        int lcm_fd = lcm_get_fileno(arm_state->lcm);
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
            lcm_handle(arm_state->lcm);
        }

        // Get commands from somewhere
        const dynamixel_command_list_t *cmds = arm_state->cmds;

        for (int id = 0; id < cmds->len; id++) {
            dynamixel_command_t cmd = cmds->commands[id];
            dynamixel_command_t last_cmd = last_cmds.commands[id];

            int update = ((cmd.utime - last_cmd.utime) > 1000000 ||
                          last_cmd.position_radians != cmd.position_radians ||
                          last_cmd.speed != cmd.speed ||
                          last_cmd.max_torque != cmd.max_torque);

            if (update) {
                arm_state->servos[id]->set_goal(arm_state->servos[id],
                                                cmd.position_radians,
                                                dmax(0.0, dmin(1.0, cmd.speed)),
                                                dmax(0.0, dmin(1.0, cmd.max_torque)));
                last_cmds.commands[id] = cmd;
            }
        }
    }

    return NULL;
}

int main(int argc, char **argv)
{
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_string(gopt, 'd', "device", "/dev/ttyUSB0", "Device name");
    getopt_add_int(gopt, '\0', "baud", "1000000", "Device baud rate");
    getopt_add_string(gopt, '\0', "status-channel", "ARM_STATUS", "LCM status channel");
    getopt_add_string(gopt, '\0', "command-channel", "ARM_COMMAND", "LCM command channel");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
        exit(-1);
    }

    arm_state_t *arm_state = arm_state_create(getopt_get_string(gopt, "device"),
                                              getopt_get_int(gopt, "baud"));

    // LCM Initialization
    arm_state->lcm = lcm_create(NULL);
    arm_state->command_channel = getopt_get_string(gopt, "command-channel");
    arm_state->status_channel = getopt_get_string(gopt, "status-channel");
    if (!arm_state->lcm)
        return -1;
    dynamixel_command_list_t_subscribe(arm_state->lcm,
                                       arm_state->command_channel,
                                       command_handler,
                                       arm_state);

    pthread_create(&arm_state->status_thread, NULL, status_loop, arm_state);
    pthread_create(&arm_state->driver_thread, NULL, driver_loop, arm_state);

    // Probably not needed, given how this operates
    pthread_join(arm_state->status_thread, NULL);
    pthread_join(arm_state->driver_thread, NULL);

    // Cleanup
    arm_state_destroy(arm_state);
    getopt_destroy(gopt);
}
