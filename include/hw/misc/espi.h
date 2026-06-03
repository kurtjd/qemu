/*
 * QEMU eSPI Device — Fake eSPI controller/target with shadow memory + VWires
 *
 * Provides transparent shared memory between two QEMU VMs via a chardev
 * socket backend, plus virtual wire signaling with interrupts. Symmetric
 * device — same code used for both host and EC sides.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ESPI_H
#define HW_MISC_ESPI_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_ESPI "espi"
OBJECT_DECLARE_SIMPLE_TYPE(ESPIState, ESPI)

/* Control register offsets (16 bytes total) */
#define ESPI_REG_INT_STATUS     0x00    /* R/W1C */
#define ESPI_REG_INT_ENABLE     0x04    /* RW    */
#define ESPI_REG_VWIRE_OUT      0x08    /* RW    */
#define ESPI_REG_VWIRE_IN       0x0C    /* R     */
#define ESPI_REG_PERIPH_ADDR    0x10    /* R — last remote write offset */

#define ESPI_CTRL_REG_SIZE      0x14    /* 20 bytes */

/* INT_STATUS / INT_ENABLE bits */
#define ESPI_INT_PERIPH_PENDING BIT(0)  /* Other side wrote to shared mem */
#define ESPI_INT_VWIRE_PENDING  BIT(1)  /* Other side changed a VWire */

/* Wire protocol opcodes (chardev socket messages) */
#define ESPI_OP_MEM_WRITE       0x01
#define ESPI_OP_VWIRE           0x02

/* Wire protocol message size: op(1) + size(1) + offset(2) + data(4) = 8 bytes */
#define ESPI_MSG_SIZE           8

struct ESPIState {
    /* private */
    SysBusDevice parent_obj;

    /* MMIO regions */
    MemoryRegion ctrl_regs;     /* Control registers (fixed address) */
    MemoryRegion shmem;         /* Shared memory (user-configured address) */

    /* IRQ */
    qemu_irq irq;

    /* Chardev backend (socket to other VM) */
    CharFrontend chr;

    /* Control registers */
    uint32_t int_status;
    uint32_t int_enable;
    uint32_t vwire_out;
    uint32_t vwire_in;
    uint32_t periph_addr;

    /* Shared memory mirror */
    uint8_t *shmem_buf;

    /* QOM properties */
    uint32_t shmem_size;
    uint64_t shmem_base;

    /* Socket message receive buffer (partial read handling) */
    uint8_t msg_buf[ESPI_MSG_SIZE];
    int msg_buffered_bytes;
};

#endif /* HW_MISC_ESPI_H */
