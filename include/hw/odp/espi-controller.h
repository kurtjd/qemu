#ifndef HW_ODP_ESPI_CONTROLLER_H
#define HW_ODP_ESPI_CONTROLLER_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_ODP_ESPI_CONTROLLER "odp-espi-controller"
#define ODP_ESPI_IRQ_COUNT 2
OBJECT_DECLARE_SIMPLE_TYPE(OdpESPIControllerState, ODP_ESPI_CONTROLLER)

#endif
