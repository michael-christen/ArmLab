#ifndef MXSERIES_H
#define MXSERIES_H

#include <stdint.h>

// Forward decs
struct dynamixel_device;
typedef struct dynamixel_device dynamixel_device_t;
struct dynamixel_bus;
typedef struct dynamixel_bus dynamixel_bus_t;

dynamixel_device_t* dynamixel_mxseries_create(dynamixel_bus_t* bus,
                                             uint8_t id);

#endif
