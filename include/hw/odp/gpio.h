/*
 * ODP socket-backed virtual GPIO controller
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef HW_ODP_GPIO_H
#define HW_ODP_GPIO_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_ODP_GPIO "odp-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(OdpGpioState, ODP_GPIO)

#endif /* HW_ODP_GPIO_H */
