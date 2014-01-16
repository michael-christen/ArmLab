#include "axseries.h"
#include "dynamixel_device.h"

#include "common/math_util.h"

// === AX series device implementation =============
bool is_address_EEPROM(int address)
{
    return address < 0x18;
}

double get_min_position_radians()
{
    return to_radians(-150);
}

double get_max_position_radians()
{
    return to_radians(150);
}

void set_joint_goal(dynamixel_device_t *device,
                    double radians,
                    double speedfrac,
                    double torquefrac)
{
    set_joint_goal_default(device,
                           0x3ff,
                           radians,
                           speedfrac,
                           torquefrac);
}

dynamixel_status_t* get_status get_status(dynamixel_device_t *device)
{
    dynamixel_msg_t *msg = msg_create(2);
    msg->buf[0] = 0x24;
    msg->buf[1] = 8;
    dynamixel_msg_t *resp = device->bus->send_command(device->bus,
                                                      device->id,
                                                      INST_READ_DATA,
                                                      msg,
                                                      true);

    msg_destroy(msg);

    if (resp == NULL)
        return NULL;

    dynamixel_status_t *stat = malloc(sizeof(dynamixel_status_t));
    stat->position_radians = ((resp->buf[1] & 0xff) +
                              ((resp->buf[2] & 0x3) << 8)) *
                             to_radians(300) / 1024.0 - to_radians(150);
    int tmp = ((resp->buf[3] & 0xff) + ((resp[4] & 0x3f) << 8));
    if (tmp < 1024)
        stat->speed = tmp / 1023.0;
    else
        stat->speed = -(tmp - 1024)/1023.0;

    // load is signed, we scale to [-1, 1]
    tmp = (resp->buf[5] & 0xff) + ((resp->buf[6] & 0xff) << 8);
    if (tmp < 1024)
        stat->load = tmp / 1023.0;
    else
        stat->load = -(tmp - 1024) / 1023.0;

    stat->voltage = (resp->buf[7] & 0xff) / 10.0;   // scale to voltage
    stat->temperature = (resp->buf[7] & 0xff);      // deg celcius
    stat->continuous = device->rotation_mode;
    stat->error_flags = (resp->buf[0] & 0xff);

    return stat;
}

void set_rotation_mode(dynamixel_device_t *device, bool mode)
{
    dynamixel_msg_t *msg = msg_create(5);
    msg->buf[0] = 0x06;
    msg->buf[1] = 0;
    msg->buf[2] = 0;
    msg->buf[3] = mode ? 0 : 0xff;
    msg->buf[4] = mode ? 0 : 0x03;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    msg_destroy(msg);
    if (resp != NULL)
        msg_destroy(resp);
}

// === AX series device creation ===================
dynamixel_device_t* axseries_create(dynamixel_bus_t* bus,
                                    void (*bus_destroy),
                                    uint8_t id)
{
    dynamixel_device_t* device = dynamixel_device_create_default();

    // Bus stuff
    device->bus = bus;
    device->bus_destroy = bus_destroy;

    device->is_address_EEPROM = is_address_EEPROM;
    device->get_min_position_radians = get_min_position_radians;
    device->get_max_position_radians = get_max_position_radians;
    device->set_joint_goal = set_joint_goal;
    device->get_status = get_status;
    device->set_rotation_mode = set_rotation_mode;

    // Return delay time
    uint8_t delay = 0x02;   // each unit = 2 usec
    dynamixel_msg_t *msg = msg_create(2);
    msg->buf[0] = 0x5;
    msg->buf[1] = delay;
    dynamixel_msg_t *resp = device->ensureEEPROM(device, msg);

    msg_destroy(msg);
    if (resp != NULL)
        msg_destroy(resp);

    return device;
}
