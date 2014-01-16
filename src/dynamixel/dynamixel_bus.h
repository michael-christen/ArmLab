#ifndef DYNAMIXEL_BUS_H
#define DYNAMIXEL_BUS_H

#include <stdint.h>

#define INST_PING           0x01
#define INST_READ_DATA      0x02
#define INST_WRITE_DATA     0x03
#define INST_REG_WRITE      0x04
#define INST_ACTION         0x05
#define INST_RESET_DATA     0x06
#define INST_SYNC_WRITE     0x83

// Forward declarations
struct dynamixel_device;
struct dynamixel_device dynamixel_device_t;

typedef struct dynamixel_msg dynamixel_msg_t
struct dynamixel_msg
{
    int len;
    uint8_t *buf;
};

typedef struct dynamixel_bus dynamixel_bus_t
struct dynamixel_bus
{
    bool retry_enable;  // =true XXX

    dynamixel_msg_t* (*send_command)(dynamixel_bus_t *bus,
                                     int id,
                                     int instruction,
                                     dynamixel_msg_t *msg,
                                     bool retry);

    void (*set_retry_enable)(dynamixel_bus_t *bus, bool retry_enable);
    void (*get_servo_model)(dynamixel_bus_t *bus, uint8_t id);

    dynamixel_device_t* (*get_servo)(dynamixel_bus_t *bus, uint8_t id);
};

// === Message creation and destruction ===========
dynamixel_msg_t msg_create(int len);
void msg_destroy(dynamixel_msg_t* msg);

// === Default bus stuff ==========================
void set_retry_enable(dynamixel_bus_t *bus, bool retry_enable);
void get_servo_model(dynamixel_bus_t *bus, uint8_t id);
dynamixel_device_t* get_servo(dynamixel_bus_t *bus, uint8_t id);

dynamixel_bus_t* dynamixel_bus_create_default();
void dynamixel_bus_destroy_default();

#endif
