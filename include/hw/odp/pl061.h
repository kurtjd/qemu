/*
 * Socket-backed Arm PrimeCell PL061 GPIO
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef HW_ODP_PL061_H
#define HW_ODP_PL061_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ODP_PL061 "odp-pl061"
OBJECT_DECLARE_SIMPLE_TYPE(OdpPl061State, ODP_PL061)

#endif /* HW_ODP_PL061_H */
