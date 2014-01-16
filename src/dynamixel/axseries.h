#ifndef AXSERIES_H
#define AXSERIES_H

#include <stdint.h>

// Forward decs
struct dynamixel_device;
typedef struct dynamixel_device dynamixel_device_t;
struct dynamixel_bus;
typedef struct dynamixel_bus dynamixel_bus_t;

dynamixel_device_t* axseries_create(dynamixel_bus_t* bus,
                                    void (*bus_destroy),
                                    uint8_t id);

#endif
