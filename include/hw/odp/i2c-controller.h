/*
 * ODP socket-backed I2C controller (master)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef HW_ODP_I2C_CONTROLLER_H
#define HW_ODP_I2C_CONTROLLER_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_ODP_I2C_CONTROLLER "odp-i2c-controller"
OBJECT_DECLARE_SIMPLE_TYPE(OdpI2CControllerState, ODP_I2C_CONTROLLER)

#endif /* HW_ODP_I2C_CONTROLLER_H */
