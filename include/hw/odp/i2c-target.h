/*
 * ODP socket-backed I2C target (slave)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef HW_ODP_I2C_TARGET_H
#define HW_ODP_I2C_TARGET_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ODP_I2C_TARGET "odp-i2c-target"
OBJECT_DECLARE_SIMPLE_TYPE(OdpI2CTargetState, ODP_I2C_TARGET)

#endif /* HW_ODP_I2C_TARGET_H */
