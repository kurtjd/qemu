/*
 * QEMU RISC-V Embedded Controller (EC) board interface
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 */

#ifndef HW_RISCV_EC_H
#define HW_RISCV_EC_H

#include "hw/core/boards.h"
#include "hw/riscv/riscv_hart.h"

#define EC_CPUS_MAX 1

#define TYPE_RISCV_EC_MACHINE MACHINE_TYPE_NAME("ec")
typedef struct RISCVECState RISCVECState;
DECLARE_INSTANCE_CHECKER(RISCVECState, RISCV_EC_MACHINE,
                         TYPE_RISCV_EC_MACHINE)

struct RISCVECState {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    RISCVHartArrayState soc;
    DeviceState *plic;
    DeviceState *i2c0;
    DeviceState *i2c_target;
    const MemMapEntry *memmap;
    Notifier machine_done;
};

enum {
    EC_CLINT,
    EC_PLIC,
    EC_UART0,
    EC_I2C0,
    EC_I2C_TARGET,
    EC_RAM,
};

enum {
    EC_UART0_IRQ = 1,
    EC_I2C0_IRQ = 2,
    EC_I2C_TARGET_IRQ = 3,
};

#define EC_IRQCHIP_NUM_SOURCES  32
#define EC_IRQCHIP_NUM_PRIO_BITS 3

#define EC_PLIC_PRIORITY_BASE   0x00
#define EC_PLIC_PENDING_BASE    0x1000
#define EC_PLIC_ENABLE_BASE     0x2000
#define EC_PLIC_ENABLE_STRIDE   0x80
#define EC_PLIC_CONTEXT_BASE    0x200000
#define EC_PLIC_CONTEXT_STRIDE  0x1000

/* 1 context: M-mode only for 1 hart */
#define EC_PLIC_SIZE \
    (EC_PLIC_CONTEXT_BASE + 1 * EC_PLIC_CONTEXT_STRIDE)

#endif
