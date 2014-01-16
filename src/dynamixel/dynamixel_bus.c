#include "dynamixel_bus.h"
#include "axseries.h"
#include "mxseries.h"

// === Messages (passed to and from the dynamixel devices over the bus ===

dynamixel_msg_t* msg_create(int len)
{
    dynamixel_msg_t *msg = malloc(sizeof(dynamixel_msg_t));
    msg->len = len;
    msg->buf = malloc(len*sizeof(uint8_t));
    return msg;
}

void msg_destroy(dynamixel_msg_t* msg)
{
    free(msg->buf);
    free(msg);
}

// === Bus Default Implementation =============

void set_retry_enable(dynamixel_bus_t *bus, bool retry_enable)
{
    bus->retry_enable = retry_enable;
}

void get_servo_model(dynamixel_bus_t *bus, uint8_t id)
{
    dynamixel_msg_t *msg = msg_create(2);
    msg->buf[0] = 0x00;
    msg->buf[1] = 3;
    dynamixel_msg_t *resp = bus->send_command(bus,
                                              id,
                                              INST_READ_DATA,
                                              msg,
                                              false);
    msg_destroy(msg);
    if (resp == NULL)
        return -1;

    int v = (resp->buf[1] & 0xff) + ((resp->buf[2] & 0xff) << 8);
    msg_destroy(resp);
    return v;
}

dynamixel_device_t* get_servo(dynamixel_bus_t *bus, uint8_t id)
{
    int model = bus->get_servo_model(bus, id);
    if (model < 0)
        return NULL;

    switch (model) {
        case 0x000c: // definitely for AX12+. Do other AX12 variants have same ID?
            return axseries_create(bus, id);
        case 0x001d: // MX28
        case 0x0136: // MX64
        case 0x0140: // MX106
            return mxseries_create(bus, id);
        default:
            break;
    }

    printf("WRN: Bus did not recognize unknown servo type %04x at id %d\n",
           model, id);

    return NULL;
}

dynamixel_bus_t* dynamixel_bus_create_default()
{
    dynamixel_bus_t *bus = malloc(sizeof(dynamixel_bus_t));
    bus->retry_enable = true;

    // Set default functions
    bus->set_retry_enable = set_retry_enable;
    bus->get_servo_model = get_servo_model;
    bus->get_servo = get_servo;

    return bus;
}

void dynamixel_bus_destroy_default()
{
    free(bus);
}
