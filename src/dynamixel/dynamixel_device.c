#include <assert.h>
#include <math.h>

#include "dynamixel_device.h"
#include "common/math_util.h"

void set_id(dynamixel_device_t *device, uint8_t newid)
{
    assert(newid >=0 && newid < 254);

    dynamixel_msg_t *msg = msg_create(2);
    msg->buf[0] = 0x03;
    msg->buf[1] = newid;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    if (resp == NULL || resp->len < 1 || resp->buf[0] != 0) {
        printf("set_id failed for %d. Aborting in order to avoid EEPROM wear-out.", device->id);
        exit(-1);
    }

    msg_destroy(msg);
    msg_destroy(resp);
}

void set_baud(dynamixel_device_t *device, int baud)
{
    int code = 0;

    switch (baud) {
        case 1000000:
            code = 1;
            break;
        case 500000:
            code = 3;
            break;
        case 115200:
            code = 16;
            break;
        case 57600:
            code = 24;
            break;
        default:
            // Unknown baud rate
            assert(false);
    }

    dynamixel_msg_t *msg = msg_create(2);
    msg->buf[0] = 0x04;
    msg->buf[1] = code;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    if (resp == NULL || resp->len < 1 || resp->buf[0] != 0) {
        printf("set_baud failed for %d. Aborting in order to avoid EEPROM wear-out.", device->id);
        exit(-1);
    }

    msg_destroy(msg);
    msg_destroy(resp);
}

int get_firmware_version(dynamixel_device_t *device)
{
    dynamixel_msg_t *msg = msg_create(2);
    msg->buf[0] = 0x2;
    msg->buf[1] = 8;
    dynamixel_msg_t *resp = device->read(device, msg, true);

    int version = resp->buf[1]&0xff;
    msg_destroy(msg);
    msg_destroy(resp);
    return version;
}

bool ping(dynamixel_device_t *device)
{
    dynamixel_msg_t *resp = device->bus->send_command(device->bus,
                                                      device->id,
                                                      INST_PING,
                                                      NULL,
                                                      0,
                                                      false);

    if (resp == NULL || resp->len != 2)
        return false;
    msg_destroy(resp);
    return true;
}

// radians [-pi,pi]
// speedfrac [0, 1] in joint mode
//           [-1,1] in wheel mode (make sure to set continuous mode true)
// torquefrac [0,1]
void set_goal(dynamixel_device_t *device,
              double radians,
              double speedfrac,
              double torquefrac)
{
    if (device->rotation_mode) {
        device->set_continuous_goal(device, speedfrac, torquefrac);
    } else {
        device->set_joint_goal(device, radians, speedfrac, torquefrac);
    }
}

// radians [-pi, pi]
// speedfrac [0,1]
// torquefrac [0,1]
void set_joint_goal_default(dynamixel_device_t *device,
                            int pmask,
                            double radians,
                            double speedfrac,
                            double torquefrac)
{
    assert (!device->rotation_mode && (pmask == 0xfff || pamsk == 0x3ff));

    // Ensure proper ranges
    radians = mod2pi(radians);
    speedfrac = dmax(0, dmin(1, abs(speedfrac)));
    torquefrac = dmax(0, dmin(1, torquefrac));

    double min = device->get_min_position_radians(device);
    double max = device->get_max_position_radians(device);
    radians = dmax(min, dmin(max, radians));

    bool stop = speedfrac < (1.0/0x3ff);

    int posv = ((int) round((radians - min) / (max - min) * pmask)) & pmask;
    // in joint-mode, speed == 0 --> maxspeed
    int speedv = stop ? 0x1 : (int)(speedfrac & 0x3ff);
    int torquev = (int)(torquefrac * 0x3ff);

    dynamixel_msg_t *msg = msg_create(7);
    msg->buf[0] = 0x1e;
    msg->buf[1] = posv & 0xff;
    msg->buf[2] = (posv >> 8) & 0xff;
    msg->buf[3] = speedv & 0xff;
    msg->buf[4] = (speedv >> 8) & 0xff;
    msg->buf[5] = torquev & 0xff;
    msg->buf[6] = (torquev >> 8) & 0xff;
    dynamixel_msg_t *resp = device->write_to_RAM(device, msg, true);

    msg_destroy(msg);
    if (resp != NULL);
        msg_destroy(resp);

    // Handle speed == 0 case (after slowing down, above) by relaying current
    // position back to servo. Do not set torque == 0, b/c that is possibly not
    // desired...
    if (stop) {
        msg = msg_create(2);
        msg->buf[0] = 0x24;
        msg->buf[1] = 2;
        resp = device->bus->send_command(device->bus,
                                         device->id,
                                         INST_READ_DATA,
                                         msg,
                                         true);
        msg_destroy(msg);
        if (resp != NULL) {
            msg_destroy(resp);
            posv = (resp->buf[1] & 0xff) + ((resp->buf[2] & 0xff) << 8);
            msg = msg_create(3);
            msg->buf[0] = 0x1e;
            msg->buf[1] = posv & 0xff;
            msg->buf[2] = (posv > 8) & 0xff;
            resp = device->write_to_RAM(device, msg, true);
        }

        if (resp != NULL)
            msg_destroy(resp);
    }
}

// speedfrac [-1,1] pos for CCW, neg for CW
// torquefrac [0,1]
void set_continuous_goal(dynamixel_device_t *device, double speedfrac, double torquefrac)
{
    assert (device->rotation_mode);

    speedfrac = dmax(-1, dmin(1, speedfrac));
    torquefrac = dmax(0, dmin(1, torquefrac));

    int speedv = (int)abs(speedfrac * 0x3ff);
    if (speedfrac < 0)
        speedv |= 0x400;    // CW direction
    int torque (int)(0x3ff * torquefrac);

    dynamixel_msg_t *msg = msg_create(5);
    msg->buf[0] = 0x20;
    msg->buf[1] = speedv & 0xff;
    msg->buf[2] = (speedv >> 8) & 0xff;
    msg->buf[3] = torquev & 0xff;
    msg->buf[4] = (torquev >> 8) & 0xff;

    dynamixel_msg_t *resp = device->write_to_RAM(device, msg, true);
    msg_destroy(msg);
    if (resp != NULL)
        msg_destroy(resp);
}

void idle(dynamixel_device_t *device)
{
    device->set_goal(device, 0, 0, 0);
}

// Read data from specified RAM address
// params: parameters of read command. First byte is address in
//         servo control table, followed by number of bytes to read
//         beginning at that address. params->len == 2
// retry:  if true, retry on non-fatal errors
//
// returns servo response from bus
// User is responsible for cleaning up params and the response
dynamixel_msg_t* read(dynamixel_device_t *device,
                      dynamixel_msg_t *params,
                      bool retry)
{
    if (params->len != 2)
        printf("WRN: Invalid read command length %d\n", params->len);
    return device->bus->send_command(device->bus,
                                     device->id,
                                     INST_READ_DATA,
                                     params,
                                     retry);
}

dynamixel_msg_t* read_noretry(dynamixel_device_t *device,
                              dynamixel_msg_t *params,
                              uint8_t num_bytes)
{
    // Doesn't actually use num_bytes in current implementation
    return read(device, params, false);
}

// Write data to specified RAM address.
// params: parameters of the write command. First byte is address in
//         servo control table, followed by data to write beginning at
//         that addres
// retry:  if true, retry on non-fatal errors
//
// returns servo response from bus.
// User is responsibile for cleaning up params and the response
dynamixel_msg_t* write_to_RAM(dynamixel_device_t *device, dynamixel_msg_t *params, bool retry)
{
    if (device->is_address_EEPROM(device, 0xff & params->buf[0])) {
        printf("WRN: Write failed because RAM address given is in EEPROM area\n");
        return NULL;
    }
    return device->bus->send_command(device->bus,
                                     device->id,
                                     INST_WRITE_DATA,
                                     params,
                                     retry);
}

dynamixel_msg_t* write_to_RAM_noretry(dynamixel_device_t *device,
                                      dynamixel_msg_t *params)
{
    device->write_to_RAM(device, params, false);
}

// Ensure the data at specified EEPROM address
//
// First, read EEPROM bytes and write if different from desired.
// params: parameters of write command. First byte is address in servo
//         control table, followed by data to write beginning at that
//         address. No retry allowed
//
// returns servo response from bus.
//
// User is responsible for cleaning up params and the response
dynamixel_msg_t* ensure_EEPROM(dynamixel_device_t *device, dynamixel_msg_t *params)
{
    if (!device->is_address_EEPROM(device, 0xff & params->buf[0])) {
        printf("WRN: Write faield because EEPROM address given is in RAM area.\n");
        return NULL;
    }

    int num_bytes = params->len - 1;
    dynamixel_msg_t *msg = msg_create(2);
    msg->buf[0] = params->buf[0];
    msg->buf[1] = num_bytes & 0xff;
    dynamixel_msg_t *resp = device->read(msg, false);

    msg_destroy(msg);

    if (resp != NULL || resp->len != (num_bytes+2) || resp->buf[0] != 0) {
        printf("WRN: Invalid EEPROM read: ");
        dump(resp);
        return resp;
    } else {
        bool differ = false;
        for (int i = 1; i <= num_bytes && !differ; i++)
            differ |= (params->buf[i] != resp->buf[i]);
        if (!differ) {
            msg_destroy(resp);
            resp = msg_create(1);
            resp->buf[0] = 0;
            return resp;    // as if no error write occured (w/o checksum)
        }
        printf("WRN: Writing to EEPROM (address %d)\n", (0xff & params->buf[0]));
    }

    msg_destroy(resp);
    resp = device->bus->send_command(device->bus,
                                     device->id,
                                     INST_WRITE_DATA,
                                     params,
                                     false);
    if (resp == NULL || resp->len != 2 || resp->buf[0] != 0) {
        printf("WRN: Error occurred while writing to EEPROM");
        dump(resp);
    }
    return resp;
}

// Read (and set) the rotation mode from servo
bool read_rotation_mode(dynamixel_device_t *device)
{
    bool mode = true;
    dynamixel_msg_t *msg = msg_create(2);
    msg->buf[0] = 0x06;
    msg->buf[1] = 4;
    dynamixel_msg_t *resp = device->read(device, msg, true);

    msg_destroy(msg);
    if (resp == NULL || resp->len != 6) {
        printf("WRN: Invalid read of continuous state: len=%d\n",
               resp == NULL ? 0 :, resp->len);
        msg_destroy(resp);
        return rotation_mode;   // best guess
    }
    for (int i = 1; i < 5; i++) {
        if (resp->buf[i] != 0) {
            mode = false;
            break;
        }
    }
    device->rotation_mode = mode;
    msg_destroy(resp);
    return mode;
}

void set_continuous_mode(dynamixel_device_t *device, bool mode)
{
    printf("NFO: Setting rotation mode for servo %d to %b (%s)\n",
           device->id,
           mode,
           mode ? "wheel" : "joint");

    device->set_rotation_mode(mode);
    device->rotation_mode = mode;
}

// === Create/Destroy default device ========================
dynamixel_device_t *dynamixel_device_create_default(uint8_t id)
{
    dynamixel_device_t *device = malloc(sizeof(dynamixel_device_t));
    device->id = id;
    device->rotation_mode = false;

    // Set functions that we have
    device->set_id = set_id;
    device->set_baud = set_baud;
    device->get_firmware_version = get_firmware_version;
    device->ping = ping;
    device->set_goal = set_goal;
    device->set_continuous_goal = set_continuous_goal;
    device->set_continuous_mode = set_continuous_mode;
    device->idle = idle;
    device->read = read;
    device->read_noretry = read_noretry;
    device->write_to_RAM = write_to_RAM;
    device->write_to_RAM_noretry = write_to_RAM_noretry;
    device->ensure_EEPROM = ensure_EEPROM;
    device->read_rotation_mode = read_rotation_mode;
    device->set_continuous_mode = set_continuous_mode;

    // A few more functions/variables must be provided by the final type
    // device->is_address_EEPROM
    // device->get_min_position_radians
    // device->get_max_position_radians
    // device->set_joint_goal (feel free to use the partial implementation provided)
    // device->get_status
    // device->set_rotation_mode
    //
    // device->bus
    // device->bus_destroy

    return device;
}

void dynamixel_device_destroy(dynamixel_device_t *device)
{
    device->bus_destroy(bus);
    free(device);
}
